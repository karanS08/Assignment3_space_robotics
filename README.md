# Exploring a Martian Cave — 49274 Space Robotics Team Project (Spring 2026)

A simulated Mars rover that explores an unknown cave on its own, finds and maps artefacts, inspects stop signs at close
range, analyses the cave geometry, and samples a real NASA water-concentration map adaptively.
ROS 2 Humble, Gazebo Fortress, Nav2 and slam_toolbox.

**Team:** Andrew Than (Perception) · Christina Li (Planning) · Karan Sharma (Advanced 2 and 6, report)

| Task | What it does | Where |
|---|---|---|
| Perception 1 | Saves a camera frame every 3 s into a folder per artefact type | `cave_explorer.py` → `save_dataset_image` |
| Perception 2 | Stop signs (red-octagon shape), alien / crystals / mushroom / sphere (colour + trained box check), ice (optional) | `perception/` |
| Perception 3 | Bearing + depth + pose → map position, merged by type; labelled RViz markers | `perception/localiser.py` |
| Planning 1 | Frontier exploration by shortest path, stuck recovery, spawn look-around | `cave_explorer.py`, `planning/safe_navigation.py` |
| Planning 2 | Approach goal 2.5 m from a stop sign, then close-range confirmation | `cave_explorer.py` |
| Planning 3 | State machine EXPLORE → PREEMPT → INSPECT → CONFIRM, timeouts, retry, abandon | `cave_explorer.py` |
| Advanced 2 | Cave geometry: widest area, centre-line, narrow passages (RViz overlay) | `advanced/cave_geometry*.py` |
| Advanced 6 | Adaptive sampling of a NASA Mars Odyssey water map with a Gaussian process | `advanced/adaptive_sampling*.py`, `advanced/measurement_field.py` |

## Setup

Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress (<https://gazebosim.org/docs/fortress/install_ubuntu/>).

```bash
sudo apt update
sudo apt install ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo ros-humble-robot-localization
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-xacro
sudo apt install python3-sklearn python3-scipy python3-opencv      # sampling node, offline scripts, vision
```

Clone into a workspace and build:

```bash
cd ~/ros_ws/src && git clone https://github.com/karanS08/Assignment3_space_robotics.git
cd ~/ros_ws
colcon build --symlink-install --packages-select cave_explorer
source install/setup.bash
```

Use a **new terminal for every command below** and `source ~/ros_ws/install/setup.bash` in each.

## Running

### 1. Autonomous exploration, artefact detection and inspection (Perception, Planning)

```bash
ros2 launch cave_explorer cave_explorer_startup.launch.py      # terminal 1: Gazebo + RViz (slow to start)
ros2 launch cave_explorer cave_explorer_navigation.launch.py   # terminal 2: SLAM + Nav2
ros2 launch cave_explorer cave_explorer_autonomy.launch.py     # terminal 3: the rover's behaviour
```

Wait for `Managed nodes are active` in terminal 2 and for the rover and map to appear in RViz before starting terminal 3.
Then the rover:

1. turns once on the spot (about 25 s) to look around, then explores by frontiers;
2. draws boxes on the **Detections** panel, with the current task written across the top;
3. shows each placed artefact as a labelled marker, for example `alien (5)`, on the map;
4. on a stop sign, stops exploring, drives to 2.5 m from it, confirms it close up, and turns its sphere green
   (yellow pending, green inspected, red abandoned);
5. stops moving when no frontier is left, after about 5 minutes. A full run takes about 6 minutes.

### 2. Cave geometry (Advanced 2), optional, runs alongside

```bash
ros2 launch cave_explorer cave_explorer_geometry.launch.py     # terminal 4, passive: only reads /map
```

RViz shows the distance-to-wall overlay, the gold centre-line, the widest open area (cyan) and the narrowest passage (red).

### 3. Adaptive sampling (Advanced 6): replaces step 1's terminal 3

It sends its own navigation goals, so stop the autonomy launch first.

```bash
ros2 launch cave_explorer cave_explorer_sampling.launch.py strategy:=adaptive num_samples:=12
ros2 launch cave_explorer cave_explorer_sampling.launch.py strategy:=random   num_samples:=12     # baseline
```

| Argument | Default | Meaning |
|---|---|---|
| `strategy` | `adaptive` | `adaptive` (Gaussian process) or `random` (baseline) |
| `num_samples` | `30` | Readings to take. About 22 s each: 12 samples take about 4.5 minutes |
| `seed` | `0` | Fixes the measurement noise and random choices |
| `travel_weight` | `0.3` | Cost of driving distance in the choice of the next place |
| `log_directory` | `~/cave_sampling_logs` | Where the CSV and results map are saved |

The Detections panel shows each reading, the error so far and where the rover is heading. When the last sample is taken a
three-panel results map (true field, estimate, uncertainty) is saved next to the CSV, opened in an image viewer and
published on the Detections image.

### Options of the autonomy node

| Parameter | Default | Meaning |
|---|---|---|
| `initial_scan` | `true` | Turn once on the spot before exploring |
| `save_dataset` | `true` | Save a frame every 3 s into `~/artifact_dataset/<type>/` (`stop_sign`, `alien`, `crystals`, `mushroom`, `sphere`, `ice`, `none`) |
| `dataset_directory` | `~/artifact_dataset` | Where those folders go |
| `detect_ice` | `false` | Ice detection (slow, one false marker seen in a run). Switch on while running: `ros2 param set /cave_explorer_node detect_ice true` |

The other parameters are defaults in `cave_explorer.py`; edit them there.

### Outputs

| What | Where |
|---|---|
| Sorted camera frames | `~/artifact_dataset/<type>/image_*.png` |
| Sampling log and results map | `~/cave_sampling_logs/<strategy>_<date>.csv` and `.png` |
| Annotated camera image | topic `detections_image` (RViz panel *Detections*) |
| Artefact markers | topic `marker_array_artifacts` |

## Troubleshooting

- **`cv_bridge` or NumPy errors in the autonomy node.** A newer NumPy in your own Python packages (`~/.local`) clashes with
  `cv_bridge`. The autonomy launch file already sets `PYTHONNOUSERSITE=1` for that node; if you start it another way, set it yourself.
- **The sampling node needs `scikit-learn`.** Do not set `PYTHONNOUSERSITE` for it unless scikit-learn is a system package.
- **Navigation goals are rejected at the start.** Nav2 was still starting; the node retries after 3 s.
- **The rover is wedged on a crystal.** The laser sits about 1 m above the floor and cannot see the crystals' low base.
  The stuck watchdog usually frees it; if not, restart the autonomy launch (the map is kept).
- **Restarting:** Gazebo is slow to start, so restart only the autonomy or sampling launch while testing.
- Kill leftovers between runs: `pkill -f "ign gazebo"; pkill -f rviz2; ros2 daemon stop`.

## Repository layout

```
.
├── README.md
├── cave_explorer/                  ROS 2 package
│   ├── cave_explorer/
│   │   ├── cave_explorer.py        Main node: exploration, inspection, state machine, detection, markers, dataset saving
│   │   ├── perception/             artefact_detector.py (colour), stop_sign_detector.py, ice_detector.py,
│   │   │                           box_classifier.py (+ .npz model), localiser.py
│   │   ├── planning/               safe_navigation.py (frontiers, Dijkstra, stuck watchdog; no ROS dependency)
│   │   ├── advanced/               cave_geometry*.py, adaptive_sampling*.py, measurement_field.py
│   │   └── common/                 template placeholders (unused)
│   ├── config/  launch/  urdf/  worlds/  data/        RViz, Nav2 and SLAM settings, launch files, world, NASA data, ice model
│   └── package.xml  setup.py
├── scripts/                        Offline tools (below)
├── docs/                           Specification, rubric and the team plan
├── dataset/  models/               Not committed (see the README in each)
└── report/sections/                Report text; the compiled report is on the report-draft branch
```

`perception/dataset_collector.py`, `perception/detector.py` and `cave_explorer_planning_1_2_3.py` are earlier single-file
versions kept for reference; the running code is in `cave_explorer.py` and the modules above.

### Offline scripts (`scripts/`, no simulator needed unless stated)

| Script | Purpose |
|---|---|
| `geometry_offline_check.py` | Check the cave geometry analysis on a map built from the world file |
| `sampling_offline_check.py` | Compare adaptive and random sampling offline |
| `detector_capture.py`, `stop_capture.py` | Record labelled camera and depth images from the running simulator |
| `detector_eval.py`, `stopsign_octagon_eval.py` | Score the colour detectors and the stop-sign detector on those captures |
| `localisation_eval.py` | Score map positions and merging on those captures |
| `box_classifier_train.py`, `ice_train.py` | Train the box check and the ice detector (needs scikit-learn) |
| `detector_colour_stats.py`, `ice_truth.py`, `offline_cave_map.py`, `wall_views.py`, `depth_localiser.py` | Helpers |

The simulator test harness (`scripts/sim/`: build a workspace, run one part for a fixed time and record it, and the analysis
scripts) is on the `report-draft` branch.

## Branches

| Branch | Content |
|---|---|
| `main` | The submitted code |
| `perception`, `planning`, `advanced` | Each member's branch, kept level with `main` |
| `report-draft` | The report (LaTeX in `report/latex/`) and the test harness |

Tag `locked-pre-yolo` marks the version tested before the last additions; `main` is newer.
