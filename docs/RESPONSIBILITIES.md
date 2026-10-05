# Responsibilities

Who owns what. The schedule, gates and rules are in [`PLAN.md`](PLAN.md).

## Summary

| Member | Role | Tasks | Marks owned | Branch | Code | Report section |
|---|---|---|---:|---|---|---|
| **Andrew Than** | Perception lead | Perception 1, 2, 3 (stretch: Advanced 1) | 35 | `perception` | `cave_explorer/cave_explorer/perception/`, `dataset/`, `models/` | `report/sections/perception.md` |
| **Christina Li** | Planning lead | Planning 1, 2, 3 | 35 | `planning` | `cave_explorer/cave_explorer/planning/` | `report/sections/planning.md` |
| **Karan Sharma** | Advanced lead, report lead, repo maintainer | Advanced 2, Advanced 6 | 30 | `advanced` | `cave_explorer/cave_explorer/advanced/`, `report/` | `report/sections/advanced.md` + all shared sections |

## Andrew Than — Perception

Accountable for the Perception row of the rubric (35 points).

- **Perception 1:** image-saving code, the dataset, and the per-type image counts.
- **Perception 2:** the detector for as many artefact types as feasible, with bounding boxes on `detections_image`.
- **Perception 3:** map-frame artefact positions, merging of repeated detections, RViz markers at the artefact.
- **Provides to the team:** the artefact list defined in `common/artefact.py`, kept up to date while the robot runs.
- **Report:** first draft of the Perception sections, with example images, detection evidence and localisation screenshots.
- **Video:** clips for Perception 1–3.
- **Stretch / fallback:** Advanced 1 (robust perception) under the conditions in `PLAN.md` section 5.

## Christina Li — Planning

Accountable for the Planning row of the rubric (35 points).

- **Planning 1:** the exploration strategy as a new `PlannerType`, with RViz visualisation of frontiers and chosen goal.
- **Planning 2:** approach-goal generation and navigation to a close-range viewpoint for 2–3 artefact types.
- **Planning 3:** the behaviour state machine, timeout / retry / abandon logic, the visited record and its markers.
- **Provides to the team:** the behaviour selection in `main_loop` and the run-mode launch parameter, so other behaviours (adaptive sampling) can take control of the robot.
- **Report:** first draft of the Planning sections.
- **Video:** clips for Planning 1–3.

## Karan Sharma — Advanced, report and repository

Accountable for both Advanced rows of the rubric (15 + 15 points) and for the submission.

- **Advanced 2:** online distance transform, widest accessible area, narrow-passage detection, RViz overlay.
- **Advanced 6:** measurement field, Gaussian Process model, adaptive sampling behaviour, baseline, comparison results.
- **Report lead:** report skeleton, editing and unifying all sections, project overview, results, teamwork reflection, statement of individual contributions, generative-AI disclosure.
- **Video:** clips for Advanced 2 and 6; assembles the final video and checks the link is viewable by the marker.
- **Repository maintainer:** keeps `main` building, chases pull-request reviews, maintains the issue / fix list from the reviews.
- **Submission:** uploads report, video link and code zip by Fri 23 Oct, 6:00pm.

## Shared by all three

- Post a daily status by 9:00pm.
- Review teammates' pull requests within 24 hours.
- Attend both reviews (15 and 17 Oct), the dress rehearsal (18 Oct) and the live demonstration (19 Oct).
- Be able to explain every part of the system, not only your own.
- Save evidence (screenshots, clips, notes) at every gate.
- Log any generative-AI use in `report/sections/ai_disclosure.md`.
- Complete the Spark Plus peer review.

## Shared files

These affect everyone. Change them only through a pull request approved by the owners of every part affected.

| File | Why it is shared |
|---|---|
| `cave_explorer/cave_explorer/cave_explorer.py` | The ROS node; wires all parts together |
| `cave_explorer/cave_explorer/common/` | Interfaces between parts |
| `cave_explorer/launch/`, `cave_explorer/config/` | Affects every run |
| `cave_explorer/setup.py`, `cave_explorer/package.xml` | Build and dependencies |

## Statement of individual contributions (draft — update as work is done)

| Task | Completed? | Team member | Contribution |
|---|---|---|---|
| Perception 1 | | Andrew Than | |
| Perception 2 | | Andrew Than | |
| Perception 3 | | Andrew Than | |
| Planning 1 | | Christina Li | |
| Planning 2 | | Christina Li | |
| Planning 3 | | Christina Li | |
| Advanced 2 | | Karan Sharma | |
| Advanced 6 | | Karan Sharma | |
| Report writing | | All | |
| Recorded videos | | All | |
