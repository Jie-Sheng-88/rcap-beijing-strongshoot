# RoboEdge 5v5 RCAP2026

Humanoid robot soccer system for the RoboCup Asian-Pacific 2026 5v5 competition.

## Repository Layout

```
Robotedge-5v5-RCAP2026/
├── configs/
│   └── fastdds.xml                  # FastDDS configuration
├── distribution/
│   ├── build_package.sh
│   ├── build_uninstall.sh
│   ├── install.sh
│   └── uninstall.sh
├── scripts/
│   ├── build.sh / build_brain.sh / build_debug.sh
│   ├── start.sh / stop.sh / start_brain.sh
│   ├── start_vision.sh / start_game_controller.sh
│   ├── start_joystick.sh / start_realsense.sh
│   ├── calibrate.sh / start_calibration.sh
│   ├── assist.sh / chase.sh
│   ├── install.sh / install_rerun_cpp.sh
│   └── test_visual_kick.py
├── third_party/                     # Offline LFS dependencies
├── utils/
│   ├── install_auto_start_assist.sh
│   └── service/
├── src/
│   ├── brain/                       # Core strategy package
│   ├── booster_msgs/                # Custom ROS 2 message definitions
│   ├── booster_ros2_interface/      # Booster SDK ROS 2 wrappers
│   ├── robocup_ros2_interface/      # Competition interface
│   ├── game_controller/             # GameController UDP bridge
│   └── vision/                      # Object detection & segmentation
└── README.md
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS 2 Kilted (C++)                          │
├──────────┬──────────┬──────────┬──────────┬────────────────────────┤
│  Brain   │  Vision  │   Game   │ Booster  │  robocup_ros2          │
│ (strategy│ (detect) │Controller│  ROS2    │  _interface            │
│  core)   │          │ (UDP)    │          │                        │
├──────────┴──────────┴──────────┴──────────┴────────────────────────┤
│                        Booster SDK (B1 loco API)                   │
└─────────────────────────────────────────────────────────────────────┘
```

## Packages

### `brain` — Strategy Core

The central decision-making node. Contains the behavior tree, multi-robot coordination, localization, and all tactical logic.

**Source files** (`src/brain/src/`):

| File | Purpose |
|---|---|
| `main.cpp` | Entry point, ROS 2 node spin |
| `brain.cpp` | Core `Brain` class: tick loop, perception processing, vision pipeline |
| `brain_tree.cpp` | Behavior tree registration and tick |
| `brain_communication.cpp` | UDP team communication and GameController bridge |
| `brain_config.cpp` | YAML parameter loading and validation |
| `brain_data.cpp` | Runtime state management, frame transforms |
| `brain_log.cpp` | Rerun logging |
| `locator.cpp` | Particle-filter localization from field markings |
| `robot_client.cpp` | Velocity commands and motion API |
| `obstacle_avoidance_logger.cpp` | Diagnostic logging for avoidance decisions |
| `odom_diagnostic_logger.cpp` | Odometry diagnostic sampling |

**Headers** (`src/brain/include/`):

| File | Purpose |
|---|---|
| `brain.h` | Brain class definition |
| `brain_tree.h` | All behavior tree node classes |
| `brain_data.h` | Runtime state, team coordination data |
| `brain_config.h` | Configuration parameters |
| `brain_communication.h` | Communication threads |
| `types.h` | Core structs: `Pose2D`, `GameObject`, `FieldDimensions`, `TMStatus`, enums |
| `team_communication_msg.h` | UDP packet layout for inter-robot communication |
| `RoboCupGameControlData.h` | Official GameController packet format |
| `freekick_policy.h` | Free-kick planning, pass target selection, phase machine |
| `assist_strategy_policy.h` | Slot-based assist positioning logic |
| `fall_recovery_policy.h` | IMU-based fall detection and recovery helpers |
| `locator.h` | Particle-filter localization interface |
| `robot_path_planner_policy.h` | Local path planning around obstacles |
| `robot_obstacle_policy.h` | Robot footprint and obstacle models |
| `ball_search_policy.h` | Ball search heuristics |
| `head_scan_policy.h` | Camera head scanning patterns |
| `fallen_robot_avoidance_policy.h` | Fallen robot handling |
| `stablizer.h` | Motion stabilization |
| `buffer.h` | Circular buffer for sensor data |
| `posProjector.h` | Pixel-to-field projection |

**Behavior trees** (`src/brain/behavior_trees/`):

| File | Purpose |
|---|---|
| `game.xml` | Main match tree (INITIAL → READY → SET → PLAY) |
| `chase.xml` | Ball-chasing demo tree |
| `demo.xml` | Demo/showcase tree |
| `subtrees/subtree_striker_play.xml` | Striker tactical subtree |
| `subtrees/subtree_goal_keeper_play.xml` | Goalkeeper tactical subtree |
| `subtrees/subtree_cam_find_and_track_ball.xml` | Camera ball tracking |
| `subtrees/subtree_find_ball.xml` | Ball search with robot locomotion |
| `subtrees/subtree_locate.xml` | Localization subtree |
| `subtrees/subtree_auto_standup_and_locate.xml` | Fall recovery + re-localization |
| `subtrees/subtree_striker_freekick.xml` | Striker free-kick behavior |
| `subtrees/subtree_goal_keeper_freekick.xml` | Goalkeeper free-kick behavior |

### `game_controller` — GameController UDP Bridge

Receives the official RoboCup GameController UDP broadcast and republishes it as a ROS 2 topic.

| File | Purpose |
|---|---|
| `src/game_controller_node.cpp` | UDP listener, packet parsing, ROS 2 publish |
| `src/main.cpp` | Node entry point |
| `include/game_controller_node.h` | Node class definition |
| `include/RoboCupGameControlData.h` | Packet struct (158 bytes) |

### `vision` — Object Detection

Runs neural-network inference on camera frames to detect balls, robots, field markings, and goalposts.

| Directory | Purpose |
|---|---|
| `src/` | Detection pipeline |
| `include/booster_vision/` | Vision headers |
| `model/` | ONNX/TensorRT models |
| `config/` | Camera and detection parameters |
| `scripts/` | Utility scripts |

### `booster_ros2_interface` — Booster SDK Wrapper

ROS 2 wrappers for the Booster Robotics SDK: odometry, low-level state, remote controller, RPC.

### `booster_msgs` — Custom Messages

Defines `RpcReqMsg` and `RpcRespMsg` for SDK RPC calls (e.g., mode queries).

### `robocup_ros2_interface` — Competition Interface

High-level competition interface utilities.

---

## Robot States

### Game State (from GameController)

The GameController broadcasts the match state. Brain reads it from the `game_controller_interface/msg/GameControlData` topic.

| State | Constant | Meaning |
|---|---|---|
| `INITIAL` | `STATE_INITIAL (0)` | Pre-match; robots wait outside the field |
| `READY` | `STATE_READY (1)` | Robots move to assigned starting positions |
| `SET` | `STATE_SET (2)` | Hold position; wait for referee whistle |
| `PLAY` | `STATE_PLAYING (3)` | Active play |
| `FINISHED` | `STATE_FINISHED (4)` | Match or half ended |

### Set-Play Sub-States

| Sub-State | Constant | Meaning |
|---|---|---|
| `NONE` | `SET_PLAY_NONE (0)` | Normal play |
| `DIRECT_FREE_KICK` | `SET_PLAY_DIRECT_FREE_KICK (1)` | Direct free kick |
| `INDIRECT_FREE_KICK` | `SET_PLAY_INDIRECT_FREE_KICK (2)` | Indirect free kick |
| `PENALTY_KICK` | `SET_PLAY_PENALTY_KICK (3)` | Penalty kick |
| `THROW_IN` | `SET_PLAY_THROW_IN (4)` | Throw-in |
| `GOAL_KICK` | `SET_PLAY_GOAL_KICK (5)` | Goal kick |
| `CORNER_KICK` | `SET_PLAY_CORNER_KICK (6)` | Corner kick |

### Game Phase

| Phase | Constant | Meaning |
|---|---|---|
| `NORMAL` | `GAME_PHASE_NORMAL (0)` | Regular match |
| `PENALTY_SHOOT_OUT` | `GAME_PHASE_PENALTY_SHOOT_OUT (1)` | Penalty shootout |
| `EXTRA_TIME` | `GAME_PHASE_EXTRA_TIME (2)` | Extra time |
| `TIMEOUT` | `GAME_PHASE_TIMEOUT (3)` | Timeout |

### Brain Control State (internal)

Set by joystick commands via `control_state` blackboard variable:

| Value | Trigger | Behavior |
|---|---|---|
| `1` | `LT+X` | Manual mode — joystick direct control |
| `2` | `LT+A` | Pickup recovery — recalibrate at entry position |
| `3` | `LT+B` | Auto/strategy mode — full autonomous play |

### Robot Roles

| Role | Description |
|---|---|
| `striker` | Attacking player; chases ball, kicks, assists |
| `goal_keeper` | Defending player; guards goal, blocks shots |

Roles are assigned via `game.player_role` in `config.yaml` and can be dynamically switched via `RoleSwitchIfNeeded` based on teammate availability.

### Robot Recovery States (Fall Recovery)

Defined in `RobotRecoveryState` enum:

| State | Value | Meaning |
|---|---|---|
| `IS_READY` | 0 | Standing, operational |
| `IS_FALLING` | 1 | IMU detects tilt beyond threshold |
| `HAS_FALLEN` | 2 | Robot is on the ground |
| `IS_GETTING_UP` | 3 | SDK recovery sequence active |

Recovery mode transition phases (`RecoveryModeTransitionPhase`):

| Phase | Meaning |
|---|---|
| `Idle` | No recovery in progress |
| `WaitingForPrepare` | Sent prepare command, waiting for SDK |
| `PrepareDwell` | Dwell time after prepare acknowledged |
| `WaitingForWalking` | Transitioning from prepare to walking mode |
| `WaitingForSoccer` | Transitioning from walking to soccer mode |

### Striker Decision States

The `StrikerDecide` node outputs a `decision` string that drives the subtree:

| Decision | Meaning | Action |
|---|---|---|
| `find` | Ball not visible | `RobotFindBall` — search with head scan + locomotion |
| `chase` | Ball visible, far away | `Chase` — move toward ball |
| `adjust` | Ball nearby, needs alignment | `Adjust` — fine positioning around ball |
| `kick` | Aligned, ready to score | `Kick` — simple touch kick |
| `cross` | Aligned, crossing angle | `Kick` at lower power |
| `assist` | Not the ball owner | `Assist` — move to support position |
| `auto_visual_kick` | RL kick eligible | `RLVisionKick` — learned visual kick policy |

### Goalkeeper Decision States

The `GoalieDecide` node outputs:

| Decision | Meaning | Action |
|---|---|---|
| `find` | Ball not visible | Move to goal-blocking position |
| `retreat` | Ball far, defensive | Move to goal-blocking position |
| `chase` | Ball close, dangerous | `Chase` aggressively |
| `adjust` | Ball nearby, needs alignment | `Adjust` positioning |
| `kick` | Aligned | `Kick` clear the ball |

### Goalkeeper Modes

| Mode | Meaning |
|---|---|
| `attack` | Ball is close; chase and clear |
| `guard` | Ball is far or unknown; hold goal position |

---

## State Transitions

### Match Flow (game.xml)

```
INITIAL ──► READY ──► SET ──► PLAY ──► END
                │         │        │
                ▼         ▼        ▼
         (localize)  (hold pos) (striker/goalie)
```

```
┌─────────────────────────────────────────────────────────────────┐
│                      Main Behavior Tree                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  control_state==1 (Manual)                                      │
│    └─► SimpleChase / Kick (joystick assist)                     │
│                                                                 │
│  control_state==2 (Pickup Recovery)                             │
│    └─► RobocupWalk → CamScanField → CamFindAndTrackBall         │
│        → SelfLocateEnterField                                   │
│                                                                 │
│  control_state==3 (Auto)                                        │
│    ├─► Penalized: SetVelocity → CamScanField → Locate           │
│    └─► Unpenalized:                                             │
│        ├─► TIMEOUT: SetVelocity → CamFindAndTrackBall           │
│        ├─► INITIAL: CamScanField → CamFindAndTrackBall          │
│        │   → SelfLocateEnterField                               │
│        ├─► READY: MoveHead → GoToReadyPosition → Locate         │
│        ├─► SET: CamFindAndTrackBall → SetVelocity → Locate      │
│        ├─► PLAY: StrikerPlay / GoalKeeperPlay                   │
│        ├─► END: SetVelocity                                     │
│        └─► FREE_KICK + STOP: CamFindAndTrackBall → Locate       │
│            FREE_KICK + EXECUTE: StrikerPlay / GoalKeeperPlay    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Striker Play Flow

```
┌──────────────────────────────────────────────────────────────┐
│                      StrikerPlay                              │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  1. SelfLocate (trust_direction)                              │
│  2. Locate (if decision != 'find')                           │
│  3. wait_for_opponent_kickoff?                                │
│     YES → CamFindAndTrackBall → FindBall → SetVelocity       │
│     NO  → continue                                            │
│  4. ball_out?                                                 │
│     YES → CamFindAndTrackBall → GoBackInField → Locate        │
│     NO  → continue                                            │
│  5. StrikerDecide → decision:                                 │
│     find      → FindBall (robot head + locomotion search)     │
│     chase     → Chase (approach ball)                         │
│     adjust    → Adjust (align for kick)                       │
│     kick      → Kick (touch kick)                             │
│     cross     → Kick (lower power cross)                      │
│     assist    → Assist (support position)                     │
│     auto_visual_kick → RLVisionKick (learned policy)          │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

### Goalkeeper Play Flow

```
┌──────────────────────────────────────────────────────────────┐
│                    GoalKeeperPlay                              │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  1. SelfLocate (trust_direction)                              │
│  2. Locate (if decision != 'find')                           │
│  3. wait_for_opponent_kickoff?                                │
│     YES → CamFindAndTrackBall → SetVelocity                   │
│     NO  → continue                                            │
│  4. ball_out?                                                 │
│     YES → CamFindAndTrackBall → GoBackInField → Locate        │
│     NO  → continue                                            │
│  5. goalie_mode:                                              │
│     attack → GoalieDecide:                                    │
│       find/retreat → GoToGoalBlockingPosition                  │
│       chase → Chase (aggressive)                              │
│       adjust → Adjust                                         │
│       kick → Kick                                             │
│     guard → CamFindAndTrackBall → GoToReadyPosition           │
│       → GoToGoalBlockingPosition                              │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

### Fall Recovery Flow

```
┌──────────────────────────────────────────────────────────────┐
│                   AutoGetUpAndLocate                           │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  IS_READY                                                     │
│    ├─► IMU tilt >= 0.48 rad → IS_FALLING                      │
│    └─► Localization pre-hold activated                        │
│                                                               │
│  IS_FALLING                                                   │
│    ├─► On ground detected → HAS_FALLEN                        │
│    └─► Upright recovery (1s stable) → IS_READY                │
│                                                               │
│  HAS_FALLEN                                                   │
│    └─► beginRecoveryPrepareSequence()                         │
│        → WaitingForPrepare → PrepareDwell                     │
│        → WaitingForWalking → WaitingForSoccer                 │
│        → IS_GETTING_UP                                        │
│                                                               │
│  IS_GETTING_UP                                                │
│    └─► Post-getup odom settling (3.5-7.5s)                    │
│        → Localization hold → Re-localize                      │
│        → IS_READY                                             │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

### Free-Kick Phase Machine

Defined in `freekick_policy.h`:

```
Inactive → Preparing → Released → FirstTouchKicking
                                      │
                                      ├─► (Direct) → Complete
                                      └─► (Indirect) → AwaitingSecondTouch
                                                          → SecondTouchKicking
                                                              → Complete
```

---

## Multi-Robot Communication

### Architecture

```
┌──────────┐     UDP Multicast      ┌──────────┐
│ Robot 1  │◄──────────────────────►│ Robot 2  │
│ (Brain)  │  239.255.255.250       │ (Brain)  │
└────┬─────┘                        └────┬─────┘
     │                                    │
     │  UDP Unicast                       │  UDP Unicast
     │  (team communication)              │  (team communication)
     │                                    │
     ▼                                    ▼
┌──────────┐                        ┌──────────┐
│ Robot N  │                        │ Robot N  │
└──────────┘                        └──────────┘
```

### Communication Channels

| Channel | Protocol | Port | Purpose |
|---|---|---|---|
| GameController | UDP Broadcast | 3838 | Receive match state |
| GameController Return | UDP Unicast | 3939 | Send robot status to GC |
| Team Discovery | UDP Multicast (239.255.255.250) | Dynamic | Find teammates |
| Team Communication | UDP Unicast | Dynamic | Exchange state at 20 Hz |

### Team Communication Packet (`TeamCommunicationMsg`)

Fields sent every ~50ms (20 Hz):

| Field | Type | Description |
|---|---|---|
| `validation` | int | Magic number `31204` to identify our packets |
| `protocolVersion` | uint16 | Protocol version (currently 5) |
| `bootId` | uint64 | Process identity (rejects stale packets from before restart) |
| `sequence` | uint32 | Packet sequence number |
| `playerId` | int | Robot ID (1-5) |
| `playerRole` | int | 1=striker, 2=goalkeeper |
| `isAlive` | bool | On field and unpenalized |
| `isLead` | bool | Currently controlling the ball |
| `ballDetected` | bool | Camera sees the ball |
| `ballLocationKnown` | bool | Ball position is known (local or teammate) |
| `ballConfidence` | double | Detection confidence |
| `ballRange` | double | Distance to ball |
| `cost` | double | Estimated seconds to reach kickable position |
| `ballPosToField` | Point | Ball position in field frame |
| `robotPoseToField` | Pose2D | Robot pose in field frame |
| `kickDir` | double | Planned kick direction |
| `thetaRb` | double | Robot-to-ball angle in field frame |
| `ballOwnerId` | int | ID of robot owning the ball |
| `leaderTerm` | uint32 | Leader election term |
| `formationOwnerId` | int | Formation leader ID |
| `formationRevision` | uint32 | Formation version counter |
| `assistSlots[MAX_NUM_PLAYERS]` | int[] | Player-to-slot assignment |
| `assistSlot` | int | This robot's assigned slot |
| `assistPhase` | int | Assist movement phase |
| `assistTarget` | Point2D | Assist position target |
| `kickoffActive` | bool | Kickoff coordination active |
| `kickoffOwnerId` | int | Kickoff ball owner |
| `freeKick*` | various | Free-kick plan fields |
| `cmdId` / `cmd` | int | Command ID and command (100 = take ball) |

### Discovery Protocol

1. Each robot broadcasts `TeamDiscoveryMsg` (validation `41203`) via multicast every 1s
2. Receiving a discovery packet registers the teammate's IP and player ID
3. Teammates that don't send any packet for 20s are removed
4. After discovery, unicast communication begins

### Leader Election & Ball Ownership

- `ballOwnerId`: The robot that currently "owns" the ball
- `leaderTerm`: Monotonically increasing term number; higher wins
- Cost-based election: each robot estimates its cost to reach the ball; lowest cost wins
- The ball owner broadcasts `cmd=100` to claim the ball; other strikers switch to assist

### Formation System

The ball owner acts as formation leader and broadcasts:
- `formationOwnerId`: Leader ID
- `formationRevision`: Incremented when formation changes
- `formationShadowSide`: Which side support players should position on
- `assistSlots[]`: Maps each player to an assist slot

Assist slots (from `assist_strategy_policy.h`):

| Slot | Name | Description |
|---|---|---|
| 0 | NONE | Not assigned |
| 1 | COVER_MID | Central cover position behind the ball |
| 2 | SHADOW_SUPPORT | Wide support on the shadow side |
| 3 | ANCHOR_COVER | Deep defensive anchor |
| 4 | WIDE_OUTLET | Wide outlet on the opposite side |

### Free-Kick Coordination

During free kicks, the ball owner proposes a plan:
- `freeKickProposerBootId`: Process that proposed the plan
- `freeKickEventId`: Monotonic event counter
- `freeKickKickerId`: Who takes the first kick
- `freeKickReceiverId`: Who receives the pass (indirect only)
- `freeKickType`: Direct or Indirect
- `freeKickPhase`: Phase of execution

Plan authority is resolved by: higher `leaderTerm` wins → higher `kickerId` wins → higher `proposerBootId` wins → higher `eventId` wins.

---

## Localization

The `Locator` class uses a particle filter to determine robot position from detected field markings.

### Process

1. Vision detects field markings (T-crossings, L-crossings, X-crossings, penalty points)
2. Markings are transformed from robot frame to field frame using current odometry
3. Each particle hypothesis is scored by residual distance to nearest expected marking
4. Iterative resampling converges to the best pose estimate

### Localization Modes (SelfLocate variants)

| Mode | When Used |
|---|---|
| `enter_field` | Initial field entry from outside |
| `trust_direction` | During play; trust current heading |
| `local` | Local refinement using nearby markings |
| `1P` | Single penalty-point localization |
| `1M` | Single marking localization |
| `2X` | Double X-crossing localization |
| `2T` | Double T-crossing localization |
| `LT` | L+T crossing combination |
| `PT` | Penalty+T crossing combination |
| `border` | Border/touchline localization |

---

## Configuration

Configured via `config/config.yaml` (ROS 2 parameters), overridden by `config/config_local.yaml`.

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `game.team_id` | — | Team number |
| `game.player_id` | — | Robot ID (1-5) |
| `game.field_type` | — | `adult_size` (14×9m) or `kid_size` (9×6m) |
| `game.player_role` | — | `striker` or `goal_keeper` |
| `robot.robot_height` | — | Height in meters for distance estimation |
| `enable_com` | true | Enable team communication |
| `robot.vx_factor` | — | Odometry velocity correction |
| `robot.yaw_offset` | — | Yaw correction for distance estimates |

---

## Build & Run

### Prerequisites

Install the Booster Robotics SDK:

```bash
cd /path/to/sdk_release
sudo ./install.sh
```

### Build

```bash
source /opt/ros/kilted/setup.bash
bash scripts/install.sh
colcon build --symlink-install --parallel-workers "$(nproc)" --cmake-clean-cache
```

### Run

```bash
# Start all nodes
bash scripts/start.sh

# Start individual nodes
bash scripts/start_brain.sh
bash scripts/start_vision.sh
bash scripts/start_game_controller.sh

# Stop all nodes
bash scripts/stop.sh
```

---

## Field Dimensions

| Type | Length | Width | Goal Width | Penalty Dist |
|---|---|---|---|---|
| Kidsize | 9 m | 6 m | 2.6 m | 1.5 m |
| Adultsize | 14.16 m | 9.22 m | 2.6 m | 2.242 m |
| RoboLeague | 22.003 m | 14.126 m | 2.6 m | 3.635 m |

---

## Dependencies

## ROS 2 Kilted Installation and Build

```bash
bash scripts/install.sh

source /opt/ros/kilted/setup.bash
colcon build --symlink-install --parallel-workers "$(nproc)" --cmake-clean-cache
```

`install.sh` uses offline dependencies managed by Git LFS under `third_party/`:

```bash
git lfs install --skip-repo
git lfs pull
```

If the LFS objects were not downloaded during clone, the installer runs `git lfs pull` automatically.

The installer checks system packages, `rosdep`, and the Rerun C++ SDK under `/opt/rerun_sdk` before installing anything. It skips rebuilding when Rerun 0.20.3 is already present and invokes Git LFS only when an offline archive is still an LFS pointer. `rosdep` uses the Tsinghua mirror by default; override it with `ROSDEP_MIRROR`. Use `RERUN_FORCE_REINSTALL=1 bash scripts/install.sh` to reinstall Rerun. If all dependencies are already installed, skip `rosdep` with:

```bash
ROSDEP_SKIP=1 bash scripts/install.sh
```
