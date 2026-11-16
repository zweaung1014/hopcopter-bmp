# Cut the plan instead of deleting it (stairs replan loop, fix A): plan

Status: **implemented 2026-11-13**, not yet flown in the sim (see "Results"). Fix A of `docs/lidar_cell_memory_plan.md`
("The stairs replan loop"). Planner only; hopcopter needs no change (see "hopcopter" below).

## Goal

When the re-check finds a bad hop later in the plan, keep the hops before it:

1. **Cut.** Republish the plan up to the waypoint before the bad landing (the *cut waypoint*).
   The robot keeps flying the good hops, stops on the cut waypoint and hops in place there
   (hopcopter's `goal` mode on a path's last waypoint).
2. **Search at once** from the cut waypoint, in the background, while the robot flies the cut
   plan. No DELETE, no waiting for a hold.
3. **Publish the rest when both are ready:** the search has finished *and* the robot is hopping
   in place on the cut waypoint. Until then the robot keeps hopping in place there.

Expected effect on the logged stairs runs: 10 of 14 deleted plans had their bad landing at
waypoint 2 or later; at 21:15 and 21:17 the robot would have reached the second tread.

## Today (`ballistic_planner_node.cpp`)

- `recheckPlan()` (`:383`) returns a reason string for the first failing hop: landing not
  standable, ground moved > 5 cm (`replan.height_tolerance`) at the landing or its takeoff, or the
  arc short of clearance.
- `replanStep()` (`:440`, 10 Hz timer) publishes a DELETE alone, waits for an in-place hop
  (`replan.hold_wait` 1 s, `replan.hold_timeout` 4 s), then calls `planAndPublish()` from
  `decision_.aim`. A failure inside `replan.min_interval` (5 s) only logs "Replan pending".
- `planAndPublish()` (`:248`) searches **synchronously** inside the timer (5–11 s on the stairs),
  so the node handles no map or decision while it runs.
- `/hopcopter/apex_decision`'s `land_index` is the waypoint the next hop aims at; the flight in
  the air lands on `land_index − 1`.

## Design

### 1. Where to cut

`recheckPlan()` returns the index `b` of the first bad waypoint as well as the reason:

| failure | `b` |
|---|---|
| waypoint j not standable, or its ground moved | j |
| waypoint j−1's ground moved (the takeoff of hop j) | j − 1 |
| the arc into waypoint j short of clearance | j (never fly that arc) |

The cut waypoint is `k = b − 1`, moved earlier until both hold:

- **The robot can still stop there.** The hop into k was planned to continue, so its landing may
  arrive too fast for a vertical takeoff. Use the search's own stop test:
  `stance(v_in(k), (0,0,1), k2).tilt ≤ tau_max` (`hopping_astar.cpp:507`), with v_in(k) from
  `hops[k−1]` (`v_h`, `heading`, `v_g`; as `publishHopPlan()` builds `arrival_velocity`).
  hopcopter's braking guard has the same 20° cap.
- **The robot isn't already committed past it.** The takeoff toward `land_index` is fixed at the
  apex that chose it, so `k ≥ land_index` (the latest decision on this plan).

If no k qualifies (`b ≤ land_index`, 4 of the 14 logged plans), keep today's DELETE and hold.

### 2. The cut plan on the wire

- Marker: `path[0..k]` with the same apexes, new stamp. `publishTrajectory()` already sends the
  DELETE + ADD pair; hopcopter keeps flying through the DELETE because the ADD follows at once.
- HopPlan: `hops[0..k−1]`, with the last landing's axis and tilt replaced by the stop stance.
  hopcopter only logs HopPlan (`_plan_values`), but its log should show the stop.
- `active_` becomes the cut plan, with a new flag **`cut = true`**: reaching its last waypoint is
  not the goal. Today `mode == "goal"` on a non-frontier plan sets `goal_reached_` (`:480`);
  that must not happen on a cut plan.
- The re-check keeps running on the cut plan. If one of its own landings flips (on the stairs
  they are beside risers too), cut again, earlier.

### 3. The background search

Run the search off the timer thread so the node keeps re-checking the cut plan and reading
decisions while it runs:

- Split `planAndPublish()` into `search()` (map conversion, targets, A*; pure, takes a copy of
  the map message's shared_ptr and the start/goal) and `publishResult()` (HopPlan, marker, CSVs,
  snapshot, `active_`; timer thread only).
- One search at a time (`std::async` + `std::future`, polled in `replanStep()`), tagged with a
  generation number. A new goal, a new cut to an earlier waypoint, or a fallback to DELETE bumps
  the generation, and the result of an older search is discarded when it arrives.
- Optional: an atomic cancel flag checked once per expansion in `HoppingAStarPlanner::plan()`, to
  stop a stale search early. Default off, so `ballistic_parity_test` stays byte-identical.
- The search starts from the cut waypoint with the existing vertical-arrival start, which is
  exactly what it will be: the robot hops in place there before the new plan is published.

### 4. When the new plan goes out

The result waits until hopcopter reports `mode == "goal"` on the cut plan (`path_id` = its stamp),
i.e. the robot is hopping in place on the cut waypoint. Then:

1. Re-check the new plan on the latest map (it was made on a map up to ~10 s old).
2. Clean: publish it. Bad from waypoint 2 on: publish its cut and search again (step 1 again).
   Bad at waypoint 1: discard and search again; the robot keeps hopping in place.

Publishing only once the robot is on the cut waypoint keeps two things true: the search's
vertical-arrival start, and hopcopter's re-sync (a new path syncs to the waypoint nearest the
predicted landing and holds if that is more than `reach` 0.8 m away, `waypoint_tracker.py`).

### 5. Rate limits and other triggers

- **Cut at once**, never rate-limited (today `since_plan < replan.min_interval` holds the DELETE).
- The search after a cut starts at once too; one-at-a-time is the limit. Keep `failed_plans_`'s
  back-off for searches that find no path (the robot keeps hopping in place on the cut waypoint).
- Unchanged: "reached the frontier", "the goal is measured now", "the robot gave up (hold)",
  and the `b ≤ land_index` fallback all keep today's DELETE-and-hold, and `replan.min_interval`.
- hopcopter reports `hold` on a cut plan (it missed a waypoint twice): drop the pending search
  and fall back to today's hold-replan from `decision_.aim`.
- New parameter `replan.cut` (default true); false restores today's behaviour, for comparison.

## hopcopter

No change expected:
- a cut plan is an ordinary path whose last waypoint is the cut waypoint, so the tracker flies the
  good hops and then hops in place there (`goal`);
- `set_path` re-syncs to the nearest waypoint of the predicted landing, which on a prefix of the
  same path is where it already was.

Known limit: a cut that arrives just after the apex that committed the takeoff toward `b` (the
decision raced the map update) still lands once on `b`, as today; the next decision on the cut
plan hops back to the cut waypoint.

## Steps

1. `recheckPlan()` returns `{b, reason}`; the stop test as a helper beside it.
2. The cut: choose k, publish the cut plan, `active_.cut`, cut-again on a failing cut plan,
   `replan.cut`. With the search still synchronous, this alone is testable in the sim.
3. Background search: `search()` / `publishResult()`, the future and generation number.
4. The publish gate (step 4 of the design) and the re-check before publishing.
5. Logging: `replans_*.csv` outcomes `cut at k`, `search started`, `published`, `discarded
   (stale)`, `fallback delete`; the cut and the gate in the node's log.
6. Docs: CLAUDE.md (architecture §1 and §3b: "holds and replans" → cuts and replans),
   `docs/lidar_map_updates_and_replanning.md` §3.3–3.4, `lidar_cell_memory_plan.md` (fix A
   status), CHANGELOG, the node's header comment.

## Pass criteria

- `ros2 run ballistic_motion_planner ballistic_parity_test`: byte-identical.
- `ros2 run ballistic_motion_planner ballistic_plan_check`: all cases pass.
- Offline: the cut-index choice (stop test, `land_index` rule, fallback) on the logged stairs
  plans from the snapshots in `lidar_cell_memory_plan.md` ("Data"), e.g. as cases in
  `ballistic_plan_check`.
- Sim, stairs (LiDAR): no DELETE when `b > land_index`; the robot flies past its first tread and
  reaches at least the second. Not required: the goal (fix B and Step 2 may still be needed).
- Sim, regression on the LiDAR map: low_wall (there and back), slope_30deg, lilypads reach their
  goals as on 2026-11-07; cliff_gap and maze no worse.
- The known-map mode is unchanged (no `upper_bound` layer: no re-check at all).

## Open questions

- **Splicing instead of waiting.** Publishing the new plan before the robot stops (prefix + new
  plan as one path) would save the hops in place, but needs a search that starts from a moving
  arrival. Out of scope; the doc's "Replanning on the move" is the same problem.
- **The `goal`-mode stop on the cut waypoint** hops in place at `desired_rel_z` (0.8 m). If the
  arrival needs the braking tilt guard, the robot may drift off the cut waypoint. Within `reach`
  (0.8 m) the tracker keeps aiming `goal` at the waypoint, so it hops back; past it, it holds and
  the hold fallback applies. Check in the sim logs.

## Results (2026-11-13, before the sim)

- `ballistic_parity_test` and `ballistic_plan_check`: unchanged (only the node changed); all
  plan_check cases pass.
- The stop test on the 18 deleted stairs plans logged 11-11 and 11-12 (from the node's hop
  tables): every waypoint's stop tilt is 1–15°, so stopping never limits the cut there. 13 of
  the 18 would have been cut (at waypoints 1–5); the other 5 were bad at waypoint 1 and still
  DELETE.
- Node smoke test, no sim (a harness feeding the known stairs map as a LiDAR map and faking
  hopcopter's decisions; the script is not in the repo):
  - waypoint 3 made unseen while the robot aims at 1: cut at waypoint 2 after 0.34 s, no DELETE
    alone; nothing published during 15 s of `track` toward the cut end (the search took 4.8 s);
    the rest from waypoint 2 published 0.04 s after `goal` there, avoiding waypoint 3;
  - then waypoint 2 made unseen during the search: cut again at 1, the first search discarded
    as stale, the rest from waypoint 1 after `goal` there;
  - waypoint 1 made unseen: DELETE alone, as before.

