# Team Plan — 49274 Team Project: Exploring a Martian Cave

Source documents: [`team_project_2026.md`](team_project_2026.md) (task spec), [`rubrics.md`](rubrics.md) (marking), [`RESPONSIBILITIES.md`](RESPONSIBILITIES.md) (who owns what), [`DEPENDENCIES.md`](DEPENDENCIES.md) (hand-overs between members and their dates).

- **Team:** Andrew Than, Christina Li, Karan Sharma
- **Live demo:** Mon 19 Oct 2026, 9:30am (25% of subject)
- **Report + video + code:** Fri 23 Oct 2026, 11:59pm (15% of subject)
- **Plan written:** Mon 5 Oct 2026 — 14 days to demo.

## 1. Ownership

| Part | Owner | Tasks | Rubric weight |
|---|---|---|---:|
| A — Perception | **Andrew Than** | Perception 1, 2, 3 | 35 |
| B — Planning | **Christina Li** | Planning 1, 2, 3 | 35 |
| C — Advanced + Report | **Karan Sharma** | Advanced 6 (adaptive scientific sampling), Advanced 2 (cave geometry analysis), report lead, final video assembly | 30 |

Each rubric row is marked as a block ("must complete all three tasks"), so every part has exactly one accountable owner. Owner does not mean sole contributor: every member must be able to explain the whole system at the demo.

## 2. Rules (non-negotiable)

1. **Gates are hard deadlines.** Every gate in section 4 is due at **9:00pm** on its date. A gate is met only if its "done when" condition is shown working on `main` (or in a posted screen recording before 13 Oct).
2. **Daily status.** Every member posts by **9:00pm every day**: what was finished, what is next, what is blocking. A missed status counts as a missed gate.
3. **Missed gate → scope cut, not extra time.** The cut for each gate is fixed in advance in section 5. Deadlines do not move.
4. **`main` always builds and runs.** `colcon build` and all three launch files must work on `main` at all times. Whoever breaks it fixes it before doing anything else.
5. **No direct commits to `main`.** Work on your part's branch; merge by pull request with **one approval from another member**. Reviews are due within 24 hours of the request.
6. **Feature freeze: Tue 13 Oct, 9:00pm.** After this, no new features — only integration, bug fixes, tuning and evidence collection.
7. **Code freeze: Sun 18 Oct, 12:00pm.** After this, nothing is merged unless all three members agree it fixes a demo-breaking bug.
8. **Shared code changes need agreement.** Changes to `cave_explorer.py`, `common/`, launch files or config need a pull request approved by the owners of every part they affect.
9. **Evidence as you go.** Every gate produces at least one screenshot or clip saved to `report/figures/` and three or more lines of notes in the owner's `report/sections/` file. Nothing is reconstructed in report week.
10. **Own code only.** External code and libraries are acknowledged in the report section; generative-AI use is logged in `report/sections/ai_disclosure.md` when it happens.

## 3. Work breakdown

### Part A — Perception (Andrew)

Artefact models in `worlds/models/artifacts`: blue cube, green alien, green crystals, ice formations, mossy boulder, blue mushrooms, toy-story alien, white sphere — plus the stop sign already detected by the template.

1. **Perception 1 — dataset**
   - Extend `image_callback` to save frames on a trigger, not every frame.
   - Multiple angles and distances per type, plus negatives. Record counts per type.
2. **Perception 2 — detection**
   - Easy, colour-distinct types first (blue cube, green alien, green crystals, blue mushrooms), then the hard ones (white sphere, ice, mossy boulder).
   - Keep a simple baseline detector as fallback if a trained model is used.
   - Annotated bounding boxes on `detections_image`.
3. **Perception 3 — localisation**
   - Bearing from bounding-box centre + range from depth → `map` frame via TF.
   - Merge repeated detections per type by distance threshold with a running average.
   - Markers at the artefact position, not the robot position.

### Part B — Planning (Christina)

1. **Planning 1 — exploration**
   - Frontier detection on `/map`, clustering, scoring by size and travel cost, new `PlannerType` in `main_loop`.
   - Blacklist unreachable frontiers; RViz markers for candidates and chosen frontier.
2. **Planning 2 — close-range inspection**
   - Standoff goal facing the artefact, validated against the map. Developed against stop signs first.
3. **Planning 3 — behaviour switching**
   - State machine `EXPLORE → APPROACH → INSPECT → EXPLORE` with timeout, one retry, then abandon.
   - Visited set; markers coloured by visited / pending.

### Part C — Advanced + Report (Karan)

1. **Advanced 2 — cave geometry analysis**
   - Distance transform recomputed on map updates; widest accessible area; narrow passages from low-clearance ridge/saddle points; RViz overlay.
2. **Advanced 6 — adaptive scientific sampling**
   - Hidden synthetic field behind a noisy `measure(x, y)`; `GaussianProcessRegressor` on (x, y); uncertainty-driven goal selection discounted by travel distance; baseline with equal sample count; RMSE-vs-samples comparison.
   - The sampler never reads the ground-truth field or its parameters.
3. **Report lead**
   - Skeleton in the required structure by 8 Oct. Owners draft their own task sections; Karan edits, unifies, and writes overview, results, teamwork reflection, contributions table and AI disclosure. Assembles the final video.

### Shared interfaces (fixed at Gate 0)

- **Artefact list** — `common/artefact.py`. Perception produces it; Planning and Advanced consume it. The `visited` flag is written only by Planning.
- **Goal dispatch** — all goals go through `planner_go_to_pose2d`; each behaviour is a `PlannerType`; only `main_loop` selects the active behaviour.
- **Run mode** — a launch parameter selects explore + inspect or adaptive sampling, so each part can be demonstrated separately.

## 4. Schedule and gates

All gates due 9:00pm.

| Gate | Date | Andrew — done when | Christina — done when | Karan — done when |
|---|---|---|---|---|
| **G0 Setup** | Tue 6 Oct | All three launch files run on own machine; cloned repo; branch pushed | Same | Same; remote repo created; interfaces in `common/` agreed and merged |
| **G1** | Thu 8 Oct | Image saver working; ≥30 images each for 4 artefact types + negatives | Frontiers detected and shown in RViz; robot drives to a chosen frontier | Distance transform published and shown in RViz; report skeleton in `report/` |
| **G2** | Sat 10 Oct | 4 easy types + stop sign detected with bounding boxes in RViz | Exploration covers the cave unattended in one run; inspection goal reached for a stop sign | Advanced 2 complete: widest area and one bottleneck marked in RViz; `measure()` + GP fit working offline |
| **G3 Feature freeze** | Tue 13 Oct | Perception 1–3 complete on `main`: remaining types attempted, artefact markers at correct map positions, duplicates merged | Planning 1–3 complete on `main`: switching with timeout/retry and visited markers, tested on stop signs | Advanced 6 complete on `main`: robot samples adaptively; baseline run recorded; RMSE comparison plotted |
| **G4 Integration** | Wed 14 Oct | Whole team: one unattended run on `main` — explore, detect, localise, inspect at least 2 artefact types; then a sampling-mode run. Every bug found is written to the issue list | | |
| **R1 Review 1** | Thu 15 Oct | Whole team, 2 hours: full run-through against the task spec and rubric line by line. Each member reviews another member's code and explains it back (Andrew → Planning, Christina → Advanced, Karan → Perception). Output: prioritised fix list with owners | | |
| **Fix day** | Fri 16 Oct | Fix list items only, highest priority first. All fixes merged by 9:00pm | | |
| **R2 Review 2** | Sat 17 Oct | Whole team, 2 hours: timed mock demonstration from a cold start, with the other two members asking questions as markers. Three consecutive successful runs required. Backup videos recorded for every task | | |
| **Code freeze** | Sun 18 Oct, 12:00pm | Final dress rehearsal on the demo machine from a fresh clone and clean build. Demo run order and who speaks to what written down | | |
| **Demo** | **Mon 19 Oct, 9:30am** | Arrive 9:00am; simulator launched and verified before the session | | |

Five days sit between feature freeze and the demonstration: one integration day, two formal reviews, one fix day and one rehearsal day.

### Report week

| Date | Due (9:00pm) |
|---|---|
| Mon 19 Oct | Each member writes down the questions the markers asked and any weaknesses exposed |
| Tue 20 Oct | Every owner's task sections fully drafted in `report/sections/`; clips for the video handed to Karan |
| Wed 21 Oct | Karan: complete assembled report and video. Contributions table agreed by all three |
| Thu 22 Oct | Andrew and Christina: full read-through with written comments. Video link tested from a logged-out browser |
| **Fri 23 Oct, 6:00pm** | **Submit** report, video link and code zip — six hours before the deadline. No large model files in the zip |

## 5. Pre-agreed scope cuts

| If this is missed | Then |
|---|---|
| G1 or G2 by anyone | Owner posts a recovery plan the same night; another member pairs with them the next day |
| Andrew G2 | Hard artefact types (white sphere, ice, mossy boulder) are dropped; effort goes to localisation of the easy types |
| Christina G2 | Exploration tuning stops at "covers the cave"; remaining time goes to inspection and switching |
| Karan G2 (Advanced 2 incomplete) | Advanced 2 is finished before any further Advanced 6 work |
| Karan G3 (Advanced 6 not working) | Advanced 6 is demonstrated as far as it works; Andrew starts Advanced 1 (robust perception) on 14 Oct as the replacement second Advanced task, provided Perception met G3 |
| Andrew G3 | Planning 3 is demonstrated on stop signs plus whichever types are reliable; no Advanced 1 |
| Christina G3 | Switching is demonstrated without retry logic; Karan assists on 14 Oct |
| G4 unattended run fails | R1 becomes a debugging session; R2 is still held on 17 Oct and is not moved |

Advanced 1 is also the stretch task: Andrew attempts it only if Perception meets G3 with nothing outstanding.

## 6. Risks

- **Karan carries 30 points alone plus the report.** Advanced 2 is front-loaded to G2; Advanced 1 is the pre-agreed replacement.
- **Planning 3 depends on Perception 3.** Planning develops against stop signs; the artefact-list format is fixed at G0.
- **Demo-day failure.** Fresh-clone rehearsal on 18 Oct, backup videos from 17 Oct, simulator launched before the session.
- **Demo questions go to everyone.** R1 cross-review makes each member explain a part they do not own.
