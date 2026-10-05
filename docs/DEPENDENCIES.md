# Dependencies — who needs what from whom, and by when

Hand-overs between team members. These dates are as binding as the gates in [`PLAN.md`](PLAN.md): all are due at **9:00pm** on the date shown.

## Rules

1. **Delivered means usable.** A hand-over counts only when it is merged to `main` (or, for non-code items, posted in the agreed place) and the receiver has confirmed it works for them.
2. **Nobody waits.** Every dependency has a stand-in the receiver uses until the real thing arrives. Being blocked is not a reason to miss a gate.
3. **Warn early.** If you will be late, tell the receiver by **12:00pm on the due date**, with a new time. Silence until 9:00pm counts as a missed gate.
4. **Interfaces are frozen after 6 Oct.** Changing `common/artefact.py`, the goal-dispatch method or the run-mode parameter after that needs approval from both provider and receiver.
5. **New software dependencies** (for example `scikit-learn`, a YOLO package) are added to `package.xml` and the README setup section in the same pull request that first uses them.

## Code and data hand-overs

| ID | From → To | What is handed over | Form | Due | Needed for | Stand-in until delivered |
|---|---|---|---|---|---|---|
| D1 | Karan → Andrew, Christina | Shared repository with branches | GitHub repo, collaborator invites | **Delivered 5 Oct** (invites still to send) | Everything | — |
| D2 | Karan → Andrew, Christina | `Artefact` record agreed and frozen | `common/artefact.py` on `main` | Tue 6 Oct | D4, D6, Planning 2–3 | Draft already in the repo |
| D3 | Christina → Andrew, Karan | Run-mode launch parameter and the hook in `main_loop` where each behaviour (`PlannerType`) is selected | Pull request to `cave_explorer.py` and the autonomy launch file | Thu 8 Oct | Karan's sampling behaviour; Andrew's data-collection runs | Own `PlannerType` on own branch, calling `planner_go_to_pose2d` directly |
| D4 | Andrew → Christina | **Artefact list v0:** stop signs only, rough map-frame position (bearing + depth), published while the robot runs | List of `Artefact` on the node, markers in RViz | Fri 9 Oct | Planning 2 at G2 (10 Oct) | Hard-coded stop-sign coordinates read from `worlds/mars_cave.sdf` |
| D5 | Karan → Christina | Clearance lookup from the distance transform: is a map point free, and how far from the nearest wall | Function in `advanced/cave_geometry.py` | Fri 9 Oct | Validating standoff goals; scoring frontiers (optional use) | Direct occupancy-grid cell check |
| D6 | Andrew → Christina | Decision: which 2–3 artefact types are reliable enough for inspection | Message in team chat + line in `report/sections/perception.md` | Sat 10 Oct | Planning 2–3 scope | Stop sign only |
| D7 | Christina → Andrew, Karan | Exploration that covers the cave unattended | `PlannerType` on `main` | Sat 10 Oct | Andrew: varied dataset and detection testing without teleop. Karan: full maps for geometry and sampling evaluation | Template random-goal behaviour or `teleop_twist_keyboard` |
| D8 | Andrew → Christina | **Artefact list v1:** all reliable types, duplicates merged, stable `id` per artefact, and a way to tell whether an artefact is currently in view | Same as D4, on `main` | Mon 12 Oct | Planning 3 switching, lost-artefact retry, visited tracking — one day of testing before feature freeze | Artefact list v0 (D4) |
| D9 | Andrew → Christina, Karan | Trained model weights, dataset link, and install steps for any new packages | Team drive link recorded in `models/README.md` and `dataset/README.md`; README setup updated | Tue 13 Oct | Everyone can run `main` for integration (14 Oct) | Stop-sign cascade detector from the template |
| D10 | Christina → Karan | Visited / pending artefact state readable from the node | `visited` flag on `Artefact` | Tue 13 Oct | Integration run; results section | — |
| D11 | Karan → Andrew, Christina | Sampling mode runnable from a single launch argument, without disturbing explore + inspect mode | On `main` | Tue 13 Oct | Integration run and demo run order | — |

## Report and video hand-overs

| ID | From → To | What is handed over | Form | Due |
|---|---|---|---|---|
| R1 | Andrew, Christina → Karan | Student numbers | `report/sections/members.md` | Tue 6 Oct |
| R2 | Karan → Andrew, Christina | Report skeleton and section files | `report/` | **Delivered 5 Oct** |
| R3 | Everyone → Karan | Evidence for the gate just passed: at least one screenshot or clip and three lines of notes | `report/figures/`, own file in `report/sections/` | Every gate: 8, 10, 13 Oct |
| R4 | Andrew → Karan | Image counts per artefact type and example images | `dataset/README.md`, `report/figures/` | Tue 13 Oct |
| R5 | Everyone → Karan | Generative-AI use, logged when it happens | `report/sections/ai_disclosure.md` | Ongoing; final check Tue 20 Oct |
| R6 | Everyone → Karan | Backup videos for own tasks | Team drive | Sat 17 Oct (Review 2) |
| R7 | Andrew, Christina → Karan | Complete drafts of own task sections; final video clips | `report/sections/perception.md`, `planning.md`; team drive | Tue 20 Oct |
| R8 | Karan → Andrew, Christina | Assembled report and video for review; contributions table for agreement | PDF + video link | Wed 21 Oct |
| R9 | Andrew, Christina → Karan | Written review comments; sign-off on the contributions table | Comments on the PDF | Thu 22 Oct |
| R10 | Karan → Canvas | Report, video link, code zip | Canvas submission | Fri 23 Oct, **6:00pm** |

## By person

### Andrew Than

- **Owes:** D4 (9 Oct), D6 (10 Oct), D8 (12 Oct), D9 (13 Oct), R1 (6 Oct), R3 (each gate), R4 (13 Oct), R6 (17 Oct), R7 (20 Oct), R9 (22 Oct)
- **Is owed:** D2 from Karan (6 Oct), D3 from Christina (8 Oct), D7 from Christina (10 Oct), R8 from Karan (21 Oct)

### Christina Li

- **Owes:** D3 (8 Oct), D7 (10 Oct), D10 (13 Oct), R1 (6 Oct), R3 (each gate), R6 (17 Oct), R7 (20 Oct), R9 (22 Oct)
- **Is owed:** D2 from Karan (6 Oct), D4 from Andrew (9 Oct), D5 from Karan (9 Oct), D6 from Andrew (10 Oct), D8 from Andrew (12 Oct), D9 from Andrew (13 Oct), R8 from Karan (21 Oct)

### Karan Sharma

- **Owes:** D2 (6 Oct), D5 (9 Oct), D11 (13 Oct), R8 (21 Oct), R10 (23 Oct); collaborator invites for D1 immediately
- **Is owed:** D3 from Christina (8 Oct), D7 from Christina (10 Oct), D9 from Andrew (13 Oct), D10 from Christina (13 Oct), R1–R7 and R9 from everyone

## The critical chain

```
D2 Artefact record (Karan, 6 Oct)
   └─► D4 artefact list v0 (Andrew, 9 Oct)
          └─► Planning 2 at G2 (Christina, 10 Oct)
                 └─► D8 artefact list v1 (Andrew, 12 Oct)
                        └─► Planning 3 at G3 (Christina, 13 Oct)
                               └─► G4 integration run (all, 14 Oct)
```

Perception → Planning is the only chain where one late hand-over directly costs another member marks: 70 of 100 points sit on it. D4 and D8 are therefore the two dates to protect above all others. Karan's Advanced tasks depend only on D3 and D7, both of which have a working stand-in in the template code.

## Extra work this adds to the gates

- **Andrew:** artefact list v0 (D4) is due on 9 Oct, earlier than full localisation at G3. It only needs stop signs and a rough position.
- **Christina:** the run-mode hook (D3) is due at G1 alongside frontier detection.
- **Karan:** the clearance lookup (D5) is due one day after the distance transform at G1.
