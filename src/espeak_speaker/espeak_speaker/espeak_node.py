#!/usr/bin/env python3
"""Announce /speak text with eSpeak on the local speaker.

The brain_node publishes human-readable text on the /speak topic
(std_msgs/String) whenever it wants the robot to talk. This node consumes
those messages and runs the espeak command-line tool so the words are spoken
through the robot's speaker. Speech is serialized in a worker thread so
messages never overlap.
"""

import queue
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = '/speak'
QUEUE_SIZE = 16


class EspeakNode(Node):
    def __init__(self):
        super().__init__('espeak_node')
        self._speech_queue = queue.Queue(maxsize=QUEUE_SIZE)
        self._worker = threading.Thread(target=self._speak_worker, daemon=True)
        self._worker.start()
        self._sub = self.create_subscription(String, TOPIC, self._on_speak, 10)
        self.get_logger().info('listening on %s' % TOPIC)

    def _on_speak(self, msg):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._speech_queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning('speech queue full, dropping "%s"' % text)

    def _speak_worker(self):
        while True:
            text = self._speech_queue.get()
            self._run_espeak(text)

    @staticmethod
    def _run_espeak(text):
        espeak = shutil.which('espeak')
        if not espeak:
            return
        try:
            subprocess.run([espeak, text], timeout=30)
        except Exception as exc:
            print('espeak failed: %s' % exc, flush=True)


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
