# Advanced

Owner: Karan Sharma.

## Advanced 2: Cave geometry analysis

Working notes (12 Oct) — to be written up properly.

- **Code:** `advanced/cave_geometry.py` (analysis, no ROS), `advanced/cave_geometry_node.py` (ROS node), `launch/cave_explorer_geometry.launch.py`.
- **Runs online:** re-analyses on every `/map` update. 0.02 s per update on the full cave grid (208 x 253 cells at 0.3 m).
- **Distance transform:** Euclidean distance from every cell to the nearest wall. A second transform, distance to the nearest wall *or unknown* cell, is used for the widest area so unexplored space is never counted as open.
- **Accessible space:** free cells with at least the robot radius (0.5 m) of clearance, connected to the robot's position.
- **Medial axis (ridge):** cells where the gradient magnitude of the distance transform drops below 0.8. Off the ridge the magnitude is 1; on it the nearest wall switches sides and the finite-difference gradient cancels.
- **Widest accessible area:** maximum of the confirmed-clearance transform over accessible cells; reported as centre and inscribed-circle radius.
- **Narrow passages:** local minima of clearance along the ridge. Each is tested by cutting the free space along the line joining its two closest wall points, inside a 5 m window. Accepted if the cut leaves two regions of at least 3 m^2 each and at least one side widens to 1.5 x the passage clearance. Labelled *doorway* (widens both sides) or *corridor* (one side).
- **Saddle check:** eigenvalues of the Hessian of the smoothed distance transform at the passage; opposite signs = saddle point.
- **RViz:** `cave_geometry/distance_transform` (grid, costmap colours) and `cave_geometry/markers` (ridge points, blue disc for widest area, red line and width label per passage, narrowest highlighted).
- **Result on the full cave** (walls rasterised from `mars_cave.sdf`, not yet from a SLAM map): widest area radius 5.9 m at [2.3, 33.3]; narrowest passage 3.1 m wide at [47.3, 3.8]; five passages between 3.1 and 4.7 m.
- **Evidence:** `figures/advanced2_offline_full_map.png`, `figures/advanced2_offline_partial_map.png`; reproduce with `python3 scripts/geometry_offline_check.py`.
- **Not yet done:** run in Gazebo with the real SLAM map; RViz screenshots; check against measured wall spacing in the world file.
- **Known limits:** the cave is fairly uniform in width (3-5 m), so "narrow" is relative. Passage widths are quantised by the 0.3 m map resolution. A long corridor of constant width is reported at its mouth, not along its length.

## Advanced 6: Adaptive scientific sampling
