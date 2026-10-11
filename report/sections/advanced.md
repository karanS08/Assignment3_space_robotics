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

Working notes (12 Oct) — to be written up properly.

- **Code:** `advanced/measurement_field.py` (the field), `advanced/adaptive_sampling.py` (GP and decision logic, no ROS), `advanced/adaptive_sampling_node.py` (ROS node), `launch/cave_explorer_sampling.launch.py`.
- **Measurement field:** real water concentration (wt % H2O) from the NASA Mars Odyssey Gamma Ray Spectrometer, a 12 x 12 bin patch around Arsia Mons stretched over the cave (x -5..60 m, y -5..50 m). Measurement noise uses the instrument's reported 1-sigma error. Details and limits in `cave_explorer/data/mars_odyssey_grs/README.md`.
- **Separation of ground truth:** `AdaptiveSampler` has no reference to the field. It receives one measurement at a time through `add_sample()`. Ground truth is read only in `evaluate()` to score the result.
- **Gaussian Process:** scikit-learn `GaussianProcessRegressor`, input (x, y) in the map frame, kernel constant x RBF + white noise. Hyperparameters are fitted once there are 5 samples; before that the prior kernel (length scale 10 m) is used.
- **Candidates:** a 1 m lattice of free cells reachable from the robot in the current map, at least 0.9 m from walls and unknown space, at least 2 m from existing samples, at least 4 m from goals that failed.
- **Adaptive choice:** maximise (GP standard deviation / max) - 0.3 x (path distance / max). Path distance is the shortest path through free space (Dijkstra on the grid), not straight-line.
- **Baseline:** uniformly random choice from the same candidates.
- **Doubles as exploration:** candidates come only from the known map, and uncertainty is highest far from samples, so the robot is drawn to the edge of the known area.
- **Failure handling:** aborted or timed-out goals (120 s) are blacklisted; the robot samples wherever it ended up and carries on.
- **RViz:** `adaptive_sampling/mean`, `adaptive_sampling/uncertainty` (grids), `adaptive_sampling/markers` (samples coloured by value, value labels, visiting order).
- **Log:** CSV per run in `~/cave_sampling_logs/` with position, value, distance travelled, RMSE and mean uncertainty after every sample.

Offline comparison (`python3 scripts/sampling_offline_check.py`): 10 runs per strategy, 40 samples, map revealed within 12 m of visited points, robot moved instantly, RMSE over the whole reachable cave.

| Samples | Adaptive RMSE | Random RMSE |
|---:|---|---|
| 5 | 0.591 +- 0.076 | 0.523 +- 0.016 |
| 10 | 0.545 +- 0.162 | 0.455 +- 0.103 |
| 20 | 0.300 +- 0.044 | 0.385 +- 0.094 |
| 30 | 0.282 +- 0.047 | 0.334 +- 0.073 |
| 40 | 0.234 +- 0.067 | 0.279 +- 0.043 |
| Mean travel | 434 m | 854 m |

- **Reading of the result:** adaptive is *worse* for roughly the first 13 samples: the travel penalty keeps it near the start, so the GP extrapolates over the unseen cave. After that it is better, and it reaches a given error in about a third of the travel distance (RMSE 0.30 at about 240 m against about 700 m for random). Differences at 30 and 40 samples are within about one standard deviation, so the sample-count advantage is modest; the travel advantage is the clear one.
- **Evidence:** `figures/advanced6_offline_rmse.png`, `figures/advanced6_offline_maps.png`.
- **Not yet done:** runs in Gazebo with real navigation (one adaptive, one random, same seed and sample count); RViz screenshots; plot of the logged CSVs.
- **Known limits:** offline simulation ignores driving time and Nav2 failures. Noise (0.2-0.6 wt %) is large against the field's range (2.3-5.0 wt %), which sets an error floor near 0.2. Online RMSE is computed over the cave explored so far, so its region grows during a run.
