#!/usr/bin/env python3
"""Announce /speak text with eSpeak on the local speaker, and play a goal
celebration sound.

The brain_node publishes human-readable text on the /speak topic
(std_msgs/String) whenever it wants the robot to talk. This node consumes
those messages and runs the espeak command-line tool so the words are spoken
through the robot's speaker. Speech is serialized in a worker thread so
messages never overlap.

The node also watches /robocup/game_controller for the goal event: a goal is
not an explicit GameController state, it is signalled by our team's score
increasing. On a score increase the node plays sui.mp3 on the local speaker,
serialized in the same worker thread so it never overlaps speech. The team id
is read from the brain's config/config.yaml (with the config_local.yaml
override) and matches the brain_node's own game.team_id resolution.
"""

import os
import queue
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from game_controller_interface.msg import GameControlData

try:
    import yaml
except ImportError:
    yaml = None

TOPIC = '/speak'
GAMECONTROLLER_TOPIC = '/robocup/game_controller'
QUEUE_SIZE = 16

# Sentinel pushed into the worker queue to play the goal audio file.
GOAL_AUDIO_MARKER = '__GOAL_AUDIO__'

# Audio players tried in order for the mp3 file; mpg123 is installed by
# scripts/install.sh, the others are used when available.
MP3_PLAYERS = (
    ('mpv', ['mpv', '--no-video', '--really-quiet']),
    ('mpg123', ['mpg123', '-q']),
    ('ffplay', ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet']),
)


def _deep_merge(base, override):
    """Recursively merge override dicts over base (rclcpp file-merge semantics)."""
    merged = dict(base)
    for key, value in override.items():
        if (key in merged and isinstance(merged[key], dict) and
                isinstance(value, dict)):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _load_brain_team_id():
    """Read game.team_id from the brain's config.yaml with config_local.yaml override.

    Returns (team_id, error) where error is empty on success.
    """
    if yaml is None:
        return None, 'python3-yaml is not installed'
    try:
        brain_share = get_package_share_directory('brain')
    except PackageNotFoundError:
        return None, 'brain package share directory not found'
    config_file = os.path.join(brain_share, 'config', 'config.yaml')
    local_file = os.path.join(brain_share, 'config', 'config_local.yaml')
    if not os.path.isfile(config_file):
        return None, 'brain config not found: %s' % config_file
    try:
        with open(config_file, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        if os.path.isfile(local_file):
            with open(local_file, encoding='utf-8') as f:
                data = _deep_merge(data, yaml.safe_load(f) or {})
    except Exception as exc:
        return None, 'failed to read brain config: %s' % exc
    try:
        team_id = data['brain_node']['ros__parameters']['game']['team_id']
    except (KeyError, TypeError):
        return None, 'brain config lacks brain_node.ros__parameters.game.team_id'
    try:
        return int(team_id), ''
    except (TypeError, ValueError):
        return None, 'brain config team_id is not an integer: %r' % team_id


def _find_goal_audio():
    """Locate sui.mp3: installed package share, source tree, then cwd."""
    candidates = []
    try:
        candidates.append(os.path.join(get_package_share_directory('espeak_speaker'), 'sui.mp3'))
    except PackageNotFoundError:
        pass
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sui.mp3'))
    candidates.append(os.path.join(os.getcwd(), 'sui.mp3'))
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return None


class EspeakNode(Node):
    def __init__(self):
        super().__init__('espeak_node')
        self._speech_queue = queue.Queue(maxsize=QUEUE_SIZE)
        self._worker = threading.Thread(target=self._speak_worker, daemon=True)
        self._worker.start()
        self._sub = self.create_subscription(String, TOPIC, self._on_speak, 10)
        self.get_logger().info('listening on %s' % TOPIC)

        self._team_id = self._resolve_team_id()
        self._goal_audio_path = _find_goal_audio()
        self._mp3_players = self._select_mp3_player()
        self._last_score = None
        if self._team_id is not None and self._goal_audio_path:
            self._sub_gc = self.create_subscription(
                GameControlData, GAMECONTROLLER_TOPIC, self._on_game_control, 10)
            self.get_logger().info(
                'goal audio enabled on %s (team %d, file %s, player %s)' %
                (GAMECONTROLLER_TOPIC, self._team_id,
                 self._goal_audio_path,
                 self._mp3_players[0] if self._mp3_players else 'none'))
        else:
            self.get_logger().warning(
                'goal audio disabled (team_id=%r audio=%r)' %
                (self._team_id, self._goal_audio_path))

    def _resolve_team_id(self):
        declared = int(self.declare_parameter('team_id', -1).value)
        if declared >= 0:
            self.get_logger().info('team_id=%d (explicit parameter)' % declared)
            return declared
        team_id, error = _load_brain_team_id()
        if team_id is None:
            self.get_logger().warning('team_id from brain config failed: %s' % error)
            return None
        self.get_logger().info('team_id=%d (from brain config)' % team_id)
        return team_id

    def _select_mp3_player(self):
        available = []
        for name, command in MP3_PLAYERS:
            if shutil.which(name):
                available.append((name, command))
        if not available:
            self.get_logger().warning('no mp3 player found; install mpg123 (see scripts/install.sh)')
        return available

    def _on_speak(self, msg):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._speech_queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning('speech queue full, dropping "%s"' % text)

    def _on_game_control(self, msg):
        if self._team_id is None:
            return
        our_score = None
        for team in msg.teams:
            if team.team_number == self._team_id:
                our_score = int(team.score)
                break
        if our_score is None:
            return
        if self._last_score is not None and our_score > self._last_score:
            self.get_logger().info('goal detected (score %d -> %d)' %
                                   (self._last_score, our_score))
            try:
                self._speech_queue.put_nowait(GOAL_AUDIO_MARKER)
            except queue.Full:
                self.get_logger().warning('speech queue full, skipping goal audio')
        self._last_score = our_score

    def _speak_worker(self):
        while True:
            item = self._speech_queue.get()
            if item == GOAL_AUDIO_MARKER:
                self._run_mp3()
            else:
                self._run_espeak(item)

    @staticmethod
    def _run_espeak(text):
        espeak = shutil.which('espeak')
        if not espeak:
            return
        try:
            subprocess.run([espeak, text], timeout=30)
        except Exception as exc:
            print('espeak failed: %s' % exc, flush=True)

    def _run_mp3(self):
        if not self._goal_audio_path or not self._mp3_players:
            return
        _, command = self._mp3_players[0]
        cmd = command + [self._goal_audio_path]
        try:
            subprocess.run(cmd, timeout=60, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except Exception as exc:
            print('mp3 playback failed: %s' % exc, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = EspeakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
