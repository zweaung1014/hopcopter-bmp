# Steadier LiDAR cell heights (Step 2): plan

Status: **planned, not started** (2026-11-11). Step 1 (`standable.lidar_steep_noise` 0.02 in
the planner and hopcopter) is done; see CHANGELOG. **Fix the stairs replan loop first** (the last
section): the robot never gets onto the stairs, with or without Step 1, and Step 2 does not fix it.

## Goal

Make each cell of the LiDAR elevation map average more of its points, so its height stops
wobbling: cleaner step heights on the stairs and cleaner slopes. Target: a memory of about
**25 points** per cell (today about 5).

## Why the heights wobble

`lidar_elevation_mapper` fuses each new point into its cell with a Kalman update, but the cell's
variance has a floor, `MapperParams::min_cell_sigma` = 1 cm (`elevation_mapper.hpp`; ROS
parameter `height.min_cell_sigma`). With the floor reached, each new point gets a weight

    w = f² / (f² + σ_hit²)        f = min_cell_sigma
    σ_hit² = (0.02 sin el)² + (r · 0.002 cos el)² + 0.01²   (the mapper's own model)

so the cell is a moving average of its last N ≈ (2 − w) / w points. A typical point on the near
steps has σ_hit ≈ 1.5 cm (1.2–1.8 cm), which gives:

| floor `min_cell_sigma` | memory N (typical point) | far, skimming points | close, steep points |
|---|---|---|---|
| 10 mm (today) | ~5 | ~3 | ~7 |
| 4.9 mm | ~20 | ~10 | ~28 |
| **4.3 mm** | **~25** | ~12 | ~35 |
| 3.0 mm | ~50 | ~25 | ~70 |

A memory of 5 leaves each cell ±5–10 mm off, and a riser (two cells) ±7–14 mm. With the risers
only 7 mm over the old 60° line, that made 4 in 10 near risers read as a slope. Step 1 moved the
line; Step 2 narrows the spread.

## Expected effect (offline replay of the 2026-11-11 stairs run)

Near steps (x 1.0–2.2). "Bottom" = the map as the robot hopped at the foot of the stairs;
"climbed" = after flying mapper_check's stairs plan over it.

| memory | riser height spread, bottom / climbed | flat treads (±, 90%), bottom / climbed | near risers caught at 0.02 |
|---|---|---|---|
| ~5 (today) | ±7.4 / ±7.4 mm | ±3.5 / ±10.4 mm | 95% / 99% |
| ~20 | ±4.9 / ±3.9 mm | ±2.3 / ±5.1 mm | 97% / 99% |
| **~25** | **±4.6 / ±3.5 mm** | ±2.1 / ±4.5 mm | 97% / 99% |
| ~50 | ±3.8 / ±2.6 mm | ±1.6 / ±3.1 mm | 97% / 99% |

- The real map's spread is ~1.35× the replay's (±10 mm today), so expect about ±6 mm at memory 25.
- Slope (replay of the 2026-11-11 slope run): ramp error ±13 mm → ±6 mm; no false edges at any
  setting, with or without 0.02.

## The catch: wrong cells stop healing

Far steps seen only from below get erased (the cells beside each riser drift up or down its
face; see "Related" below). Today they **heal** once the robot gets close; with a long memory
they **don't**:

| after climbing past them | x 2.5 | x 2.8 | x 3.1 |
|---|---|---|---|
| memory ~5 (today) | 59% steep, Δz 0.175 | 57%, 0.175 | 59%, 0.174 |
| memory ~25 | 67%, 0.178 | **43%, 0.095** | **43%, 0.086** |

Cause, in `ElevationMapper::hit()` (`elevation_mapper.cpp`): a point more than 3σ below a cell's
height is ignored and counted in a "below-run"; 30 agreeing below-points in a row (std < 1.5 cm,
`kBelowRun`, `kBelowStd`) replace the height. But every point that fuses resets the run (unless a
neighbour explains it), and stray riser points keep fusing into a wrong cell. With a memory of 5
the wrong height drifts back within reach of the floor points by itself; with 25 it sits still,
so the floor points stay "too far below" and the run never completes.

**So Step 2 = the longer memory plus a healing fix, shipped together.**

## Plan

1. **Make the replay a repo tool first** (`hopcopter_perception`, e.g. `mapper_replay`), so every
   number here can be reproduced (see "Tools" below).
2. **Healing fix** in `ElevationMapper::hit()`, designed and measured in the replay before
   anything else. Candidates, in the order to try:
   - a. Don't reset the below-run when a point fuses; reset it only when the run itself stops
     agreeing (its std over `kBelowStd`). Stray points in between then can't erase the evidence.
   - b. Shorten the memory on surprise: when several points in a row land outside the cell's
     window on the same side, raise the cell's variance so the next points move it.
   - c. Count the run over a window (e.g. "most of the last 30 points are below and agree")
     instead of strictly in a row.
3. **Longer memory**: `MapperParams::min_cell_sigma` 0.01 → 0.0043 (memory ~25) in
   `elevation_mapper.hpp`, and update its comment (it says the floor exists for slopes; the
   slope replay showed no problem at 4.3 mm, and the comment should say so with the numbers).
   The node (`height.min_cell_sigma`) and `mapper_check` both take this default.
4. Docs: CHANGELOG (why 4.3 mm, the numbers above), CLAUDE.md if behaviour visible to the planner
   changes.

## Pass criteria

- Replay, far steps (x 2.5–3.1) after climbing: heal at least as well as today (≥ 55% steep,
  median Δz ≥ 0.17 m).
- Replay, near steps: ≥ 95% caught at 0.02, riser spread ≤ ±5 mm.
- Slope replay: no false edges, ramp error no worse than today.
- `ros2 run hopcopter_perception mapper_check`: all worlds pass.
- `ballistic_plan_check --lidar-maps <dir>` (maps from `mapper_check --map-out <dir>`): no new
  failures. Already failing before Step 2: the stairs LiDAR map (a far-step cell 10 cm high) and
  slanted_pads (no plan).
- Then a sim run on `stairs` and `slope_30deg`.

## Tools and data

- **Replay harness** (built 2026-11-11 in a session scratchpad, which may be gone). To rebuild:
  - mapper_check's raycaster (`World`, the 40 × 512 Mid-360, 2 cm range noise) cast from the
    Gazebo truth pose of a real run: `data/data_lio/lio_*.csv` columns `truth_x/y/z/roll/pitch/
    yaw` are the LiDAR frame at 10 Hz (roll ≈ 180°, upside down). Apply the truth attitude to
    the sensor's native −7…+52° rays.
  - Option: map through the FAST-LIO pose (`lio_*` columns) instead of the truth.
  - A copy of `elevation_mapper.cpp` that records each point's outcome in `hit()` (face
    ignored, attributed, raise, below, below-run, fused) to see which points set each cell.
  - "Then climb": append mapper_check's stairs flight after the replay.
  - Scores: per-riser Δz, % over the edge line, lower/upper cell error, flat-tread spread,
    the planner's steep mask (`Map2D5::steep_mask` rule) against the known map.
  - Last copy: `/tmp/claude-1000/-home-zweminhtetaung-CrazySim/1b115702-335e-47f2-b53c-bcdb4b19c79b/scratchpad/h/`.
- **Runs used:**
  - Stairs: `data/data_lio/lio_2026-11-11_18-25-03.csv`; mapper snapshots
    `data/data_elevation_map/snapshot_2026-11-11_18-24-31_*.json` (`elevation`, `raw`, `hits`,
    `known`).
  - Slope: `data/data_lio/lio_2026-11-11_18-20-39.csv`, snapshots `snapshot_2026-11-11_18-20-11_*`.

## Related (not in Step 2)

- **Stray riser points** (the far-step "ramp"): about 1 in 10 riser points escape the face
  filter. Most of them are the middle of three rings hitting a riser 1.7–1.9 m away, where the
  rings are 4.5–5 cm apart, under `face.min_dz` (5 cm). The filter also flags 60–75% of the real
  tread points beside a riser as face points. Far away, the escaped points outnumber the tread
  points 5:1 to 200:1, and the fusion lets them drag the cells beside the riser 6–12 cm up or
  down. Edge attribution then drops the tread points that would correct a sunk cell. It happens
  with zero range noise too. In the real run it also hit 31 of 136 near risers (at the
  staircase's sides, seen at an angle).
- The real map is 2–3× noisier on flat treads than the replay (±1.5 vs ±0.5 cm). Not explained;
  a run with `pose_source:=gazebo` would tell FAST-LIO timing from the Gazebo sensor.

## The stairs replan loop (2026-11-11/12): cause and fix

Status: **diagnosed, not fixed** (no code changed). Fix it before Step 2.

### What happens

The robot never gets onto the stairs. It makes 1–3 hops toward them, then hops in place until the
run is stopped, because every plan is thrown away about a second after it is published:

```
 1. Some cells beside the risers read wrong for a while: stray riser points pull the cell
    below a riser up (or the one above it down) by 6–13 cm, or the per-cell wobble dips a
    riser just under the 15.3 cm edge line. The cell above it looks like a fine landing.
 2. The planner searches (5–11 s) on one copy of the map and puts a landing there.
 3. The map corrects that riser: it is an edge again.
 4. The planner re-checks the rest of the plan on every map update (2 Hz). 0.3–1.7 s after
    publishing, it finds the bad landing, deletes the WHOLE path, and the robot hops in
    place: at least 1 s of hold, then another 5–11 s search ...
 5. ... on a map where other cells read wrong → back to 2.
```

Getting close is what corrects the map (far risers heal only from close up), and the loop keeps
the robot from getting close.

### Evidence

Every stairs plan in these LiDAR runs was deleted 0.3–1.7 s after it was published, except the
last plan of each run, still up when the run was stopped:

| run | planner | plans deleted | search | deleted after | reason |
|---|---|---|---|---|---|
| 11-11 18:24 | before Step 1 | 1 | 15.2 s | 0.43 s | the arc into waypoint 1 (a phantom cell) |
| 11-11 21:15 | Step 1 | 2 | 3.8, 11.1 s | 1.65 s\*, 1.25 s | waypoints 3, 4 no longer standable |
| 11-11 21:17 | Step 1 | 1 | 9.0 s | 0.47 s | waypoint 5 no longer standable |
| 11-12 00:17 | Step 1 (yaw-fix worktree) | 5 | 4.9–6.7 s | 0.62–1.24 s | waypoints 1, 2, 2, 5; the arc into 2 |
| 11-12 00:19 | Step 1 (yaw-fix worktree) | 5 | 4.7–5.5 s | 0.28–0.88 s | waypoints 1, 4, 5, 6; the arc into 1 |

\* Found after 0.65 s. `replan.min_interval` (5 s from the start of the last search) held the
DELETE another second, and the robot kept flying the plan meanwhile.

- All 11 "no longer standable" landings were cells right beside a riser (x 0.95, 1.05, 1.25,
  1.35, 1.95, 2.25).
- hopcopter (`hop_tracking/apex_decisions_2026-11-11_21-15-26.csv`, `..._21-16-53.csv`):
  - 21:15: warm-up, idle ×14, track ×2, idle ×14, track ×1, idle ×11 (farthest x 0.73 m).
  - 21:17: warm-up, idle ×13, track ×1, idle ×12 (farthest x 0.52 m).
  - The 00:17/00:19 runs wrote no hopcopter CSVs.

The three bad landings of 21:15 and 21:17 are below, in two maps: the one each plan was made on
(the planner's snapshot, layer `grid`), and the one the next search used, 1 s after the DELETE.
Heights are in cm: the cell below the riser / the landing above it / the riser.

| plan published | bad landing | true | when planned | at the next search |
|---|---|---|---|---|
| 21:16:09 | waypoint 3 (1.35, −2.85) | 18 / 36 / 18 | 25.2 / 36.1 / **10.8** | 17.1 / 35.6 / **18.5** |
| 21:16:23 | waypoint 4 (1.95, −2.55) | 54 / 72 / 18 | 66.7 / 72.4 / **5.7** | 54.4 / 71.2 / **16.8** |
| 21:17:14 | waypoint 5 (2.25, −1.05) | 72 / 90 / 18 | 73.2 / 88.5 / **15.30** | 72.0 / 89.6 / **17.6** |

- In the first two, stray riser points had pulled the cell below the riser up 7 and 13 cm, so the
  18 cm step read 11 and 6 cm.
- In the third, the riser read 0.2 mm under the 15.32 cm line.
- Where the map was right (the risers at x 1.0 and 1.6 read 17.4–18.8 cm), the plans landed
  mid-tread, as Step 1 intends.

### Cause

1. **The map near the risers is still correcting itself.** Stray riser points (see "Related"
   above) move the cells beside a riser 6–13 cm until enough tread points arrive: seconds later,
   or only once the robot is close. On top of that, each cell wobbles ±7–10 mm, which is what
   Step 2 is for.
2. **The planner treats each map as final.** It plans on one copy of the map. Then `recheckPlan()`
   (`ballistic_planner_node.cpp`) tests the remaining hops against every new map. A hop fails on:
   - a landing no longer standable;
   - ground moved more than `replan.height_tolerance` (5 cm);
   - an arc short of clearance.

   Two of each tread's three cells are beside a riser, so almost every plan up the stairs lands on
   one that is about to flip.
3. **One bad hop cancels the whole plan.** `replanStep()` publishes a DELETE even when the bad
   landing is the 5th hop. It waits for the robot to hop in place (`replan.hold_wait` 1 s,
   `replan.hold_timeout` 4 s), then searches again from there. The robot never flies the hops that
   were still good.

### Not caused by Step 1

- It happened before Step 1:
  - the 18:24 stairs plan lasted 0.43 s;
  - a slope_30deg plan at 14:47 lasted 1 s ("waypoint 5 (2.15, −1.65) is no longer standable").
- Two of the three corrected risers above read over the old 17.3 cm line too (18.5 and 17.6 cm).
- Step 1 can add a few triggers, such as a riser settling between 15.3 and 17.3 cm (waypoint 4's
  16.8 cm). Reverting it would put landings back beside risers, not end the loop.

### Fixes

**A. Cut the path instead of deleting it** (planner, `replanStep()`; do this first). This is "get
closer, then replan", built into the planner:
1. When the bad landing isn't the next one, the planner publishes the path up to the waypoint
   before it.
2. The robot keeps flying the good hops and stops there (hopcopter hops in place on a path's last
   waypoint).
3. The planner searches from that waypoint right away, without waiting for a hold, and then
   publishes the rest.

It would have kept the robot moving in 10 of these 14 plans (bad landing at waypoint 2 or later).
At 21:15 the second plan would have taken it to (1.45, −2.05), and at 21:17 the first to
(1.45, −0.75). Both are on the second tread.

To work out when building it:
- **Stopping on the cut waypoint.** The search requires a stop at the goal (landing tilt ≤ 20°:
  the last stance in `hopping_astar.cpp`, `stance(v_in, vertical, k2)`), but a mid-plan landing
  may arrive too fast for one. If so, cut a waypoint earlier.
- **Cut at once.** Today a failed re-check inside `replan.min_interval` only logs "Replan
  pending", and the robot keeps flying toward a known-bad landing. Rate-limit the search, not the
  cut.
- **The search blocks the node.** It runs in the 10 Hz replan timer under `rclcpp::spin`. That's
  fine here, since hopcopter flies the cut path on its own, but nothing re-checks the cut path's
  landings during the 5–11 s search.
- **Joining the new plan.** It starts at the cut waypoint, and hopcopter re-syncs to the waypoint
  nearest its predicted landing. Decide whether to splice it onto the rest of the cut path (one
  HopPlan) or publish it alone.
- **A bad next landing** (waypoint 1: 4 of the 14) still needs today's DELETE and hold.

**B. Stop the stray riser points** (mapper; the root cause, and the biggest change). The cells
beside risers would then read right the first time and stop flipping, and the far steps would
stop looking like ramps. The mechanism is in "Related" above. Candidates:
- close the face filter's gap: the middle of three rings is 4.5–5 cm from its neighbours, under
  `face.min_dz` (5 cm);
- make a cell's height ignore points far from its usual value (nearer "the most common height"
  than the average);
- weight the few tread points more.

**C. Faster search.** It won't fix the loop by itself: the 00:17/00:19 searches took 5–7 s, and
every plan still died within 1.3 s.

**D. Plan with a stricter test than the re-check** (hysteresis). For example, search with the edge
line a couple of cm lower than the re-check's (the slope's steepest measured jump is 10.7 cm), or
keep cells that were steep in any recent map out of the search.
- It would stop the search picking a cell that's about to flip, like 21:17's 15.30 cm riser.
- It doesn't help with the cells that stray points moved 7–13 cm (21:15's two), which look flat
  when planned.

**Order:** A, then B, then Step 2. Step 2's stairs sim check needs the loop gone. After B,
re-measure "the catch" above in the replay: with fewer stray points, fewer cells should need
healing.

### Data

- Planner logs: `~/.ros/log/ballistic_planner_node_<pid>_*.log`.

  | run | pid |
  |---|---|
  | 18:24 | 1643612 |
  | 21:15 | 1786206 |
  | 21:17 | 1787885 |
  | 00:17 | 2009160 |
  | 00:19 | 2013538 |
  | 14:47 (slope_30deg) | 1398566 |
- Planner snapshots: `data/data_ballistic_planner/vis_planned_path/snapshot_2026-11-11_<time>.json`
  for 21-16-09, 21-16-23, 21-16-35, 21-17-14 and 21-17-26.
  - Layers: `grid` (the map the plan was made on), `steep`, `standable`, `path`.
  - Indexing: cell `j * cols + i`, with i along x and j along y from `origin_x/y`.
- Mapper snapshots: `data/data_elevation_map/snapshot_2026-11-11_21-15-20_*` and `21-16-51_*`,
  taken every ~10 s of sim time.
- The 00:17 and 00:19 runs came from the yaw-fix session in `~/CrazySim-yawFix`. Its planner is
  `2e5152a3` with no changes (built 2026-11-11 23:10). Its hopcopter and firmware have the yaw
  changes.
