# Exploring a Martian Cave — 49274 Space Robotics Team Project (Spring 2026)

Software for a simulated Mars rover that autonomously explores a cave, detects and localises artefacts, inspects them at close range, and performs advanced science tasks.

**Team:** Andrew Than · Christina Li · Karan Sharma

| Key date | What |
|---|---|
| Tue 13 Oct, 9:00pm | Feature freeze |
| Sun 18 Oct, 12:00pm | Code freeze |
| **Mon 19 Oct, 9:30am** | **Live demonstration** |
| **Fri 23 Oct, 6:00pm** | **Team submission** (official deadline 11:59pm) |

## Start here — Andrew and Christina

Do these in order. Steps 1–4 are due **Tue 6 Oct, 9:00pm** (Gate 0).

1. **Read the three team documents** (about 20 minutes):
   - [`docs/RESPONSIBILITIES.md`](docs/RESPONSIBILITIES.md) — find your name: your tasks, your branch, your folder, your report section.
   - [`docs/PLAN.md`](docs/PLAN.md) — the rules (section 2), your column in the gates table (section 4), and what gets cut if a gate is missed (section 5).
   - [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md) — the "By person" section: what you **owe** teammates and what you **are owed**, with dates.
2. **Object now or accept.** The plan, gates and hand-over dates are a proposal from Karan. If a date or a task is not workable for you, say so in the team chat **before Gate 0**. After that they are treated as agreed and binding.
3. **Get the project running:** follow [Setup](#setup) and [Running](#running) below until all three launch files work on your machine.
4. **Check in:** switch to your branch, add your student number to `report/sections/members.md`, and push.
   ```bash
   git checkout perception   # Christina: planning
   ```
5. **Put your dates in your calendar:** every gate from your column in `PLAN.md`, and every "owes" date from `DEPENDENCIES.md`.
6. **Start your first gate** (due Thu 8 Oct):
   - **Andrew:** image saver working; at least 30 images each for 4 artefact types, plus negatives.
   - **Christina:** frontiers shown in RViz and the robot driving to one; the run-mode hook in `main_loop` (hand-over D3).

### Every day

- Post your status by **9:00pm**: finished, next, blocked.
- Check `DEPENDENCIES.md` for anything you owe in the next two days. If you will be late, tell the receiver by **12:00pm** on the due date.
- Not received something you are owed? Use the stand-in listed for it and keep working — do not wait.
- Review open pull requests within 24 hours.

### Reference

- [`docs/team_project_2026.md`](docs/team_project_2026.md) — the task specification
- [`docs/rubrics.md`](docs/rubrics.md) — how it is marked

## Who owns what

| Member | Tasks | Branch | Folder |
|---|---|---|---|
| Andrew Than | Perception 1–3 | `perception` | `cave_explorer/cave_explorer/perception/` |
| Christina Li | Planning 1–3 | `planning` | `cave_explorer/cave_explorer/planning/` |
| Karan Sharma | Advanced 2 + 6, report | `advanced` | `cave_explorer/cave_explorer/advanced/`, `report/` |

## Repository layout

```
.
├── README.md
├── docs/                         Spec, rubric, plan, responsibilities, dependencies
├── cave_explorer/                ROS 2 package (template from Canvas)
│   ├── cave_explorer/
│   │   ├── cave_explorer.py      Main node — SHARED, change by pull request only
│   │   ├── common/               SHARED interfaces between parts
│   │   │   └── artefact.py       Artefact record passed from Perception to Planning
│   │   ├── perception/           Andrew
│   │   │   ├── dataset_collector.py   Perception 1
│   │   │   ├── detector.py            Perception 2
│   │   │   └── localiser.py           Perception 3
│   │   ├── planning/             Christina
│   │   │   ├── frontier_explorer.py   Planning 1
│   │   │   ├── inspection.py          Planning 2
│   │   │   └── behaviour_manager.py   Planning 3
│   │   └── advanced/             Karan
│   │       ├── cave_geometry.py       Advanced 2
│   │       ├── measurement_field.py   Advanced 6 (ground truth, evaluation only)
│   │       └── adaptive_sampling.py   Advanced 6
│   ├── config/  launch/  urdf/  worlds/
│   ├── package.xml
│   └── setup.py
├── dataset/                      Training images (not committed — see dataset/README.md)
├── models/                       Trained model weights (not committed — see models/README.md)
└── report/
    ├── sections/                 One file per owner, plus shared sections
    └── figures/                  Screenshots and plots saved at every gate
```

## Setup

Requires Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress (<https://gazebosim.org/docs/fortress/install_ubuntu/>).

```bash
sudo apt update
sudo apt install ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-xacro
```

Clone into your workspace and build:

```bash
cd ~/ros_ws/src
git clone <repo-url> mars_cave_project
cd ~/ros_ws
colcon build --symlink-install --packages-select cave_explorer
source install/setup.bash
```

## Running

One terminal per launch file:

```bash
ros2 launch cave_explorer cave_explorer_startup.launch.py      # Gazebo + RViz
ros2 launch cave_explorer cave_explorer_navigation.launch.py   # SLAM + Nav2
ros2 launch cave_explorer cave_explorer_autonomy.launch.py     # our node
```

Gazebo is slow to start: restart only the second and third launch files while testing.

## Workflow

1. Work on your own branch (`perception`, `planning`, `advanced`). Short-lived feature branches off it are fine.
2. Before opening a pull request: rebase on `main`, run `colcon build`, and run all three launch files.
3. Open a pull request to `main`. It needs **one approval from another member**. Reviews are due within 24 hours.
4. Changes to shared files (`cave_explorer.py`, `common/`, `launch/`, `config/`, `setup.py`, `package.xml`) need approval from the owner of every part affected.
5. `main` must always build and run. If you break it, fixing it is your first job.
6. Do not commit dataset images, model weights, videos, or `build/ install/ log/` — they are in `.gitignore`.
7. Post your daily status by 9:00pm.

The full rules are in [`docs/PLAN.md`](docs/PLAN.md).
