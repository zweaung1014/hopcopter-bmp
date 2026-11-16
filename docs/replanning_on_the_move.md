# Replanning on the move: reaching the goal without stopping to plan

Status: analysis and proposals, 2026-11-13. Nothing here is implemented, and no code was changed
to write it. It covers the LiDAR map (on a hand-made map the planner plans once per goal and the
robot never waits). The stairs are left aside.

The question: while the robot flies a plan, does the planner already plan the next part, on the
terrain that becomes visible as the robot moves? Or does the robot reach the end of the plan first,
and only then does the planner plan again?

## Summary

- **The planner does not plan ahead.** While the robot flies a plan, the planner only *re-checks*
  the hops not yet flown against each new map (2 Hz). A new plan is searched only after the robot
  has stopped: when it hops in place on the last waypoint of a frontier plan, or after the planner
  deletes the plan (the goal came into view, a landing went bad, the robot gave up). The search
  then runs to the end on the node's only thread, 2–30 s on the logged runs, while the robot hops
  in place. The new cut (`replan.cut`, `docs/replan_cut_plan.md`, not flown yet) is the one search
  that runs while the robot flies, and its plan still goes out only once the robot hops in place on
  the cut waypoint.
- **That is most of a run.** Between sending the goal and reaching it, the robot hopped in place
  for 55–90 % of the time on the logged maze, slope and cliff_gap runs (§2.1). On the maze run of
  2026-11-12 11:12, 38 of the 50 hops were in place: the 12 that moved took 11 s, the run took 45 s.
- **Three things make it wait**, and each needs its own fix:
  1. *when* the next search starts: only after the robot stops;
  2. *where* it starts: the search models its start as a hop in place (a vertical arrival), so a
     new plan can only be handed to a robot that has stopped;
  3. *how long* it takes: the A\*'s heuristic is the straight-line distance, while about half of a
     plan's cost is energy, so the search expands most of the reachable map.
- **The search can be ~100× faster.** Offline, on the worlds' known maps, a weighted A\* (ε = 2,
  with an energy lower bound in the heuristic) planned the maze in 17 ms instead of 11.4 s and
  cliff_gap in 13 ms instead of 24.5 s; the plans cost 0.1–8 % more and pass `ballistic_plan_check`'s
  flight checks (§4 C1). That change alone shrinks every wait to under one hop, and it makes planning
  ahead practical: a 50 ms search can run on every map update and be done long before the robot needs
  it.
- **Slowing down helps only at the margin.** A hop takes ~0.9 s whatever its length, so each extra,
  shorter hop before the unknown buys 0.9 s. That hides a 1–2 s search, not a 10–30 s one. It is worth
  having at occluders (the maze's wall), where the terrain behind is seen only from close by.
- **Recommended order** (§5): (1) the faster search; (2) run every search in the background and stop
  deleting plans that are still good; (3) splice the next plan onto the current one while the robot
  moves (a search from a moving start); (4) slow down and look over occluders where still needed.

## 1. How planning works today

### 1.1 Three loops

| Loop | Node | When | What |
|---|---|---|---|
| Map | `lidar_elevation_mapper` | scans in at 10 Hz, map out at 2 Hz | grows one persistent map |
| Plan | `ballistic_planner` | on a goal; a 10 Hz timer that acts on each new map | plans, re-checks, replans (`ballistic_planner_node.cpp`) |
| Waypoint | `hopcopter` | once per hop, at the apex | which waypoint the next hop aims at (`waypoint_tracker.py`) |

### 1.2 One goal, step by step

1. **Goal.** `goalCallback` (`ballistic_planner_node.cpp:250`) searches at once, inside the callback,
   from the robot's position on the latest map. If the goal is measured and standable, the plan goes
   to the goal. Otherwise it goes to the reachable standable cell nearest the goal: a *frontier plan*
   (`frontier.cpp`: a flood in 1.5 m steps over standable cells, nearest the goal first; the node
   tries up to two candidates, `:325–347`). The robot hops in place meanwhile (no path: `idle`).
2. **Flying.** On every new map, `replanStep` (`:709`) re-checks the hops not yet flown
   (`recheckPlan`, `:496`): each landing still standable, its ground moved < 5 cm, each arc still
   clears. If all pass, nothing happens. **No search runs while the robot flies.** The newly seen
   terrain is used only to re-check the plan and to notice that the goal has become standable.
3. **End of a frontier plan.** The robot lands on the last waypoint and hops in place there
   (`goal`). The planner reads `goal` on a frontier plan ("reached the frontier", `:768`), then:
   - waits until 5 s have passed since the previous search *started* (`replan.min_interval`, `:800`;
     `last_plan_time_` is set before the search, `:473`);
   - publishes a lone DELETE (`:815`). hopcopter keeps hopping in place, drops the path 0.5 s later
     (`hopcopter.py:1113`) and reports `idle` at its next apex;
   - waits at least 1 s after the DELETE for that `idle` (`replan.hold_wait`, `:717`);
   - searches from the spot the robot hops on, on the node's thread (`planAndPublish`, `:471`), and
     publishes.
4. The robot takes the new path at its next apex: it syncs to the waypoint nearest its predicted
   landing (`waypoint_tracker.py:95`).
5. Repeat until hopcopter reports `goal` on a plan to the goal itself.

### 1.3 What starts a new plan, and what the robot does meanwhile

| Trigger | Robot | Planner | Robot until the new plan |
|---|---|---|---|
| goal received | anywhere | searches at once, blocking | in place for the whole search |
| reached the frontier | on the frontier, in place | rate limit → DELETE → wait for `idle` → search, blocking | in place for all of it |
| the goal is measured now | mid-plan, moving | rate limit → DELETE → wait → search, blocking | stops after its current hop |
| a re-check fails past the next landing (`replan.cut`) | mid-plan | cuts the plan at the latest waypoint it can stop on; searches in the background | flies the good hops, then in place on the cut waypoint until the search is done |
| a re-check fails at the next landing | mid-plan | DELETE → wait → search, blocking | in place for the search |
| hopcopter holds (missed a waypoint 3 times) | anywhere | DELETE → wait → search | in place |

### 1.4 Why a plan can only start from a stop

The search's start state is a robot hopping in place: it arrives straight down at the speed of a
0.8 m warm-up hop (`hopping_astar.cpp:441`, `:103`, `:538`). Every hop's feasibility depends on the
arrival into its takeoff: the leg returns √η of the arrival speed, along a direction the stance can
reach within the 20° landing tilt. A plan from a waypoint the robot crosses at 1.7 m/s is a
different plan, which the search cannot make today. So the planner stops the robot first (a DELETE,
or the cut's stop) and plans from where it hops in place. This is the "Replanning on the move" item
left for later in `docs/lidar_mapping_plan.md`.

### 1.5 What adds to the wait

- **Blocking searches.** Every search but the cut's runs on the node's only thread, so no map and no
  hopcopter decision is read while it runs, and its plan was made on a map as old as the search is
  long. In 8 of the 11 logged cases of "the goal is measured now", it fired within 1.5 s of a
  frontier plan going out, 6 of them after searches of 5–24 s: the plan was out of date when it
  went out, and the robot was stopped again to replan.
- **The rate limit counts from the start of the previous search.** After a short search it holds a
  robot that already stands on the frontier (maze 2026-11-12: 4 s at the second frontier, after a
  0.16 s search); after a long one it never applies.
- **The DELETE handshake** adds 1.0–1.3 s before the search even starts, also when the robot was
  already hopping in place on the frontier.
- **The search is slow** (§2.3, §2.4).

## 2. Measurements

### 2.1 Goal to arrival (sim, LiDAR map)

From the planner's logs (`~/.ros/log/ballistic_planner_node_*.log`). "In place" adds the first
search and each DELETE-to-new-plan interval; it leaves out rate-limit waits, so it is a lower bound.

| Run | Goal → reached | Replans | In place, at least |
|---|---|---|---|
| maze 11-12 11:12 | 45.3 s | 4 | 30.9 s (68 %); 38 of 50 hops by the apex log (§2.2) |
| maze 11-12 09:31 | 33.6 s | 3 | 18.8 s (56 %) |
| maze 11-11 22:51 | 34.5 s | 3 | 20.3 s (59 %) |
| cliff_gap 11-12 10:29 | 61.5 s | 2 | 49.2 s (80 %) |
| cliff_gap 11-11 19:46 | 109.5 s | 3 | 97.5 s (89 %) |
| slope 11-12 09:32 | 68.9 s | 3 | 59.3 s (86 %) |
| slope 11-11 22:50 | 19.1 s | 1 | 10.5 s (55 %) |
| lilypads 11-07 10:42 | 15.8 s | 2 | 5.6 s (35 %) |

### 2.2 The maze run of 2026-11-12 11:12, hop by hop

The planner's log merged with `hop_tracking/apex_decisions_2026-11-12_11-12-19.csv`; t = 0 when the
goal arrived.

| t [s] | Planner | Robot |
|---|---|---|
| 0 | search 1 (5.6 s) | 6 hops in place |
| 5.6 | frontier plan 1, to (5.65, 1.05) | 4 hops forward, 5.7 m |
| 9.1 | reached the frontier → DELETE → search 2 (0.16 s) | 2 in place |
| 10.5 | frontier plan 2, to (6.55, −0.05): one hop; the 0.5 m wall hides the rest | 1 forward |
| 11.8 | reached the frontier; the rate limit holds the replan until 15.8 | 5 in place |
| 15.8 | DELETE; `idle` at 17.1; search 3 (7.8 s) | 9 in place |
| 25.0 | frontier plan 3, to (13.95, −0.05) | 1 forward |
| 25.3 | the goal is measured now → DELETE → search 4 (7.5 s) | 9 in place |
| 33.9 | plan 4, to the goal | 2 forward |
| 34.8 | waypoint 3 (9.85, −0.15) no longer standable → DELETE → search 5 (5.3 s) | 6 in place |
| 41.4 | plan 5 | 4 forward |
| 45.0 | goal reached | |

12 hops forward, 38 in place. Of the ~34 s in place, 26.4 s were searches, ~4 s the rate limit and
~4 s DELETE handshakes. Plans 3 and 4 were out of date within a second of going out: each was planned
on a map 7–8 s old. (With today's `replan.cut`, the last replan would have been a cut at waypoint 2.)

### 2.3 Search time

| World | In the sim (all logged searches) | Offline: the full plan on the known map (§4 C1, "today") |
|---|---|---|
| low_wall / flat, 4 m | 3.6–7.8 s, up to 21 s | 2.2–2.6 s |
| maze | 0.1–9 s, median 2.3 s | 11.4 s |
| lilypads | 0.03–3.5 s | 3.4 s |
| slope_30deg | up to 28 s, median 8.8 s | 8.1 s |
| cliff_gap | 2.7–31 s; 20–31 s for the long legs | 24.5 s |
| stairs | 3–19 s, median 5 s | 5.2 s |

The sim shares the CPU with Gazebo and FAST-LIO, so a search runs 1.5–3× slower there. The hop
period is 0.88 s (median, apex to apex) and a hop covers ~1.5 m on flat ground, so a moving robot
makes ~1.7 m/s: a 5 s search is 6 hops, ~9 m, of travel.

### 2.4 Where a search spends its time

Stack samples (gdb, 387 samples) of the momentum-aware search in `ballistic_plan_check`:

- 99 % generating neighbours (`momentum_neighbors`): for each expanded state, every landing cell on
  a lattice over a disk of up to ~4 m (300–1,300 cells), and for each up to 5 takeoff angles:
  - 26 % the arc's clearance (`clearance_floor_alpha`), which depends only on the two cells, not on
    the speed;
  - 21 % the feasible-angle interval (friction cones, energy band);
  - ~48 % the rest of the loop: the stance and landing tilt for each angle (14 %; 1–2 M tilt rejects
    per search), the push check and the hop's numbers, mostly trig;
- ~5 % hash maps and allocation; the open list is negligible.

So the cost is the number of edges, 5–33 M per search, and fewer expansions pay directly. There are
many because the edge cost is distance plus `w_energy` × energy (`hopping_astar.cpp:395`), about
half of it energy (the maze's 14 m plan costs 25.5), while the heuristic is the straight-line
distance (`:149`). A\* must expand every state whose cost so far plus the distance left is below the
plan's cost, ~1.8× the distance: most of the reachable map, at several speeds. On empty flat ground
it takes 3,700 expansions to find 3 hops.

## 3. What "not stopping" requires

1. **A deadline.** hopcopter commits hop k → k+1 at the apex of the flight that lands on k (the
   decision and the landing attitude, `hopcopter.py:904`). A new plan that changes the path from
   waypoint k on must reach hopcopter before that apex. If the robot's latest decision aims at
   waypoint L (`ApexDecision.land_index`), that apex is about (k − L + ½) hops away, so
   (k − L + ½) × 0.88 s must exceed the search time plus ~0.1 s.

   | Search | Earliest waypoint to splice at | Ahead of the robot |
   |---|---|---|
   | 0.1 s | L, tight | ~1 hop |
   | 1 s | L + 1 | ~2 hops, 3 m |
   | 5 s | L + 6 | ~10 m, past the end of most plans |

   Plans held 2–10 waypoints and frontiers sat 1–8 m ahead on the logged runs, so planning ahead
   needs searches of ~1 s or less in the sim.
2. **A start that matches the robot.** A plan from waypoint k must start from the arrival there, the
   speed and direction of the planned hop into k (`Hop::v_g`, `heading`, `beta`), not from a hop in
   place.
3. **A seamless handover.** If the new path keeps the old one's waypoints up to k, hopcopter's
   re-sync (the waypoint nearest the predicted landing) lands on the same index and it carries on
   without a hop in place. One detail: at the apex of the flight into k − 1, hopcopter already reads
   waypoint k + 1 (`_lookahead_axis`, `hopcopter.py:836`) to predict how the foot will touch down
   on k. A splice that arrives after that apex and turns the hop out of k sharply moves that
   touchdown by up to ~0.1 m (the 0.4 m leg times the change in landing tilt). A splice before the
   apex into k − 1 avoids even that; the deadline above is the hard one.
4. **Always a place to stop.** The published plan must end on measured ground the robot can stop on
   (the search's stop test: a vertical takeoff within the 20° landing tilt, i.e. an arrival at least
   56° steep). If the next plan is late, the robot stops there, as it does today. Frontier plans
   already end that way, and `cutIndex` (`ballistic_planner_node.cpp:566`) finds such a waypoint
   inside a plan. This keeps the rule that the robot never commits to a landing the planner has not
   checked on the current map.

## 4. Ideas

### A. Start the next search early

- **A1. Search from the end of a frontier plan as soon as it goes out.** The robot will stop there
  anyway, so today's start (a hop in place) is exact. This is the cut's machinery (`startCutSearch`,
  `pollCutSearch`, `releaseCutResult`) with the frontier plan's last waypoint as the cut waypoint:
  search in the background, search again whenever a newer map has arrived and the last search is
  done, and publish the newest result, re-checked on the newest map, as soon as hopcopter reports
  `goal` there. No DELETE, no rate limit, no hold wait. It leaves one hop in place per frontier, if
  the search finishes before the robot arrives.
- **A2. Stop deleting plans that are still good.** "The goal is measured now" does not make the
  current plan wrong; it means a better one exists. Keep flying, search in the background (from the
  plan's end, as in A1, or from a waypoint ahead, as in A4) and swap when the new plan is ready. Only
  a bad *next* landing and a hold need to stop the robot.
- **A3. Search continuously: a receding horizon.** One worker, busy while the robot moves: when a
  search finishes, the next one starts on the newest map, from the hand-over waypoint (A4). A result
  replaces the rest of the plan when it passes the re-check on the newest map and is better by a
  margin (it reaches the goal where the current plan stops at a frontier, or costs a set fraction
  less), so the path does not flicker with each map. The frontier then recedes ahead of the robot, as
  you describe it, and the robot reaches it only where it truly cannot see further (an occluder, a
  dead end).
- **A4. Splice instead of stopping: a search from a moving start.** Let the search start at waypoint
  k with the arrival of the planned hop into k (the speed bin of `hops[k-1].v_g`, its heading and
  descent angle) instead of a hop in place. In `plan_momentum` that is the start state's speed bin and
  its arrival hop (`nullptr` today, `hopping_astar.cpp:499–504`, and `annotate_path`'s first
  `v_in`); the first hop's tilt check then uses the real arrival, as for every other state. The node
  publishes `path[0..k]` plus the new path from k (one marker, one HopPlan, a new stamp). Choose k as
  the earliest stoppable waypoint the deadline (§3.1) allows for the expected search time. A result
  that misses its deadline is discarded, not published: the robot stops at the plan's end and the
  planner falls back to A1.
- **A5. Several hand-over points at once.** Search from k, k + 1 and k + 2 in parallel and use the
  earliest one still before its deadline: cheap insurance against the timing race.

### B. Slow down before the unknown

For a hopper, slowing down means shorter hops, not slower ones: the apex sets the hop period
(~0.9 s), and speed is hop length per period. Shorter hops near the end of the known terrain mean more
hops, each one 0.88 s more for the search and one more scan from closer.

- **B1. The plan already brakes at its end.** Every plan ends in a stop, and the search makes its
  last hop arrive at least 56° steep. The robot does not need to stop for longer than the search
  takes.
- **B2. An approach segment before a frontier or an occluder.** Make the last hops before the plan's
  end short: a cost on hop length that grows near unseen cells, or a cap on hop length by distance
  to the frontier. With the frontier 4 hops away, replacing the last two 1.5 m hops by four 0.75 m
  hops buys ~1.8 s. Short hops also arrive steeper, so the robot can turn harder into the next plan
  (the 20° tilt cap allows less redirection at speed), which helps in the maze's corridor.
- **B3. Make the wait useful: look over the occluder.** If the robot has to wait, hop higher. An
  apex of 1.4 m instead of 0.8 m lifts the sensor from ~1.6 m to ~2.2 m (D6 in
  `docs/lidar_mapping_plan.md`), and the unseen shadow behind a 0.5 m wall shrinks from ~0.45× to
  ~0.3× the robot's distance to the wall. The wait then extends the map, so the next leg is longer.
  In-place hops use `desired_rel_z` today; the planner would need a way to ask for a higher one.
- **Limit.** B2 buys ~1–2 s per approach: enough for a 1–2 s search, not for today's 5–30 s. So it
  comes after C, and its best use is at occluders, where the terrain behind appears only 1–2 hops
  before the robot gets there.

### C. A faster search

- **C1. A better heuristic, and weighted A\*.** Two changes to `heuristic()` (`hopping_astar.cpp:149`),
  for the momentum-aware search only, so the reference search and `ballistic_parity_test` stay
  unchanged:
  - *An energy lower bound, admissible.* Every landing arrives at 3.75 m/s or more (`min_apex`
    0.7 m, binned), and its stance loses 1 − η of that energy: (1 − η)·½m·3.75² ≈ 0.58 J. Less up to
    0.29 J of speed-bin rounding, every hop costs at least 0.30 J. At least ⌈d / 4.1 m⌉ hops remain
    (4.1 m: the longest hop the target lattice generates), and the edge cost charges a climb's
    energy, at least m·g·Δz in total. Hence h = d + w_energy·max(0, ⌈d / 4.1⌉·0.30 J + m·g·Δz).
    Plans of the same cost, 1.1–2.5× fewer expansions.
  - *Weighted A\*:* f = g + ε·h, which bounds the plan's cost by ε times the best one, and in practice
    does much better.

  Offline, full plans from the start to the README goal on each world's known map, on this machine.
  A scratch copy of `hopping_astar.cpp` with the two changes, compiled like the package; the repo is
  unchanged:

  | World | Today | + energy bound | ε = 1.5 | ε = 2 | Plan cost at ε = 2 |
  |---|---|---|---|---|---|
  | flat, 4 m | 2.21 s | 1.31 s | 0.18 s | 0.016 s | +7.9 % |
  | low_wall | 2.59 s | 1.86 s | 0.49 s | 0.087 s | +4.1 % |
  | maze | 11.4 s | 9.0 s | 2.5 s | 0.017 s | +6.1 % |
  | cliff_gap | 24.5 s | 11.8 s | 2.3 s | 0.013 s | +2.2 % |
  | slope_30deg | 8.1 s | 3.3 s | 0.006 s | 0.006 s | +0.1 % |
  | lilypads | 3.4 s | 2.9 s | 0.56 s | 0.17 s | +5.4 % |
  | slanted_pads | 3.5 s | 3.3 s | 1.2 s | 0.26 s | +1.5 % |
  | stairs | 5.2 s | 3.5 s | 1.7 s | 0.57 s | +3.1 % |

  The same scratch build of `ballistic_plan_check` at ε = 2: every flight check passes (leg angle
  ≥ 40° and push within capacity for η 0.71–0.85, landing tilt ≤ 20°, the partial LiDAR maps); only
  the three pinned plans differ, as expected. To watch before adopting it:
  - ε = 3 went wrong once (slanted_pads: 14 hops, +42 %), so stay at 1.5–2;
  - a greedier search may take hops nearer the limits: one case went from 55.9° to 42.7° of leg angle
    (guard 40°) and from 15.9° to 19.8° of landing tilt (cap 20°);
  - the pinned plans need ε = 1 or new pins, and the LiDAR maps (`--lidar-maps`) and frontier plans
    need the same check;
  - with a greedy order, a state's arrival is more often not its cheapest one, so the search's final
    tilt re-check (`recheck_failed`, which returns no plan) may fail more often: fall back to a lower
    ε when it does.

  If more is needed: a 2D cost-to-go from the goal (a Dijkstra over the grid, OBSTACLE blocking, in
  hop-sized steps) as the heuristic sees the maze's walls and poles.
- **C2. Anytime search** (ARA\*): a plan at ε = 3 in milliseconds so the robot can go, then lower ε
  while time remains, each improvement spliced in. It fits the deadline of §3.
- **C3. Cheaper edges** (§2.4); together maybe 1.5–3×:
  - cache the clearance floor and the friction-cone interval per (takeoff cell, landing cell): they
    do not depend on the speed, and each cell is expanded at several speeds;
  - reject over-tilted takeoffs before computing the full stance and push;
  - a coarser landing lattice for long hops (`hop_scan_step` 0.1 m, widening with √radius).
- **C4. Parallel work.** The 300–1,200 edge checks of one expansion are independent; and `search()`
  tries the goal and the frontier candidates one after another (`ballistic_planner_node.cpp:357`),
  which could run side by side.
- **C5. Search less.** Repair a failed re-check locally (a short search from the hop before the bad
  one to the waypoint after it) instead of replanning to the goal; or bound the search to a few hops
  toward a subgoal on a 2D route.

### D. Fewer and better replans

- **D1. Re-check every result on the newest map before publishing it**, as `releaseCutResult`
  already does. Plans 3 and 4 of the maze run were out of date when they went out.
- **D2. Rate-limit path changes, not searches.** `replan.min_interval` exists to stop churn while
  every replan stops the robot. Once replans don't stop it, what needs a limit is how often the path
  changes under it (the margin of A3).
- **D3. Choose frontiers by where they lead.** Today the frontier is the reachable cell nearest the
  goal in a straight line. Scoring the candidates by the cost to reach them plus an optimistic cost
  from there to the goal through bounded unseen cells avoids frontiers in dead ends (risk 5 of
  `docs/lidar_mapping_plan.md`, cliff_gap) and needless short legs.
- **D4. Plan through the unknown optimistically, fly only the known part.** Let the search's tail
  cross bounded unseen cells at an optimistic cost, and cut the published plan at its last stoppable
  measured waypoint (the free-space assumption of D\* Lite; FASTER keeps a safe stop in known space
  the same way). The plan's direction then stays stable as the map grows. A larger change, and D3
  gets most of its benefit.

### E. See earlier

- Look-out hops at occluders (B3).
- The planner reacts to the map at 2 Hz (the mapper's `publish_rate`). With fast searches, 4–5 Hz
  would cut ~0.25 s from each reaction, if the mapper keeps up.
- Yaw does not matter here: the Mid-360 sees all around.

## 5. Recommendation

1. **The faster search (C1).** A heuristic weight parameter (2 in `robot_params()`, 1 in
   `PlannerParams`' reference defaults) and the energy bound. Pass: `ballistic_parity_test`
   byte-identical; `ballistic_plan_check`'s flight checks, with the pinned plans re-pinned or at
   ε = 1; the sim reaching the goal on maze, cliff_gap, slope_30deg and lilypads. Expected: every
   search in §2.2 drops from seconds to under a hop, the first plan included. What remains is the
   DELETE handshakes and the rate limit, ~8 s on that run.
2. **Background scheduling (A1, A2, D1, D2).** Every search on the worker thread; frontier plans
   searched on from their end and released at `goal` there; no DELETE for "the goal is measured
   now" or "reached the frontier"; a re-check before every publish; the rate limit on deletes only.
   Expected: one hop in place per frontier, none when the goal comes into view.
3. **Splice on the move (A4, then A3).** The moving-start search and the deadline, then continuous
   replanning from the hand-over waypoint with the margin. Expected: no hops in place where the next
   terrain is seen at least ~2 hops ahead; one where it appears later (behind the maze's wall). With
   step 1's search times, steps 2 and 3 can be done together.
4. **Only if still needed:** B2 and B3 at occluders, and D3.

Each step stands alone, and step 1 alone removes most of the waiting.

## 6. How to measure

- **Hops in place from goal to arrival**: in `hop_tracking/apex_decisions_*.csv`, apexes with mode
  `goal`, `idle` or `hold` before the first `goal` on the final plan, against `track` and `retry`.
- **Goal → reached**, and each replan's reason and outcome: `replans_*.csv` and the planner's
  `[TIMING]` lines.
- The baselines are §2.1 and §2.2.

## 7. Open questions

- **ε per world.** Is one weight right for all worlds, or should it drop when a search fails or for
  short legs?
- **The splice's arrival.** A splice assumes the robot arrives at waypoint k as planned. hopcopter
  re-solves every takeoff from its real arrival, with the 20° braking guard, so small errors are
  absorbed; a large miss already makes it retry. Does a continuation from a planned arrival hold up
  as well as one from a hop in place? Check in the apex logs (`tilt_needed_deg`, retries) after the
  first spliced runs.
- **CPU.** Continuous background searches compete with Gazebo and FAST-LIO; at ~50 ms a search on
  every map update is ~10 % of a core, but a slow search must not starve the sim (one worker, low
  priority).
- **Where the frontier stops being enough.** Behind the maze's wall, the terrain appears only within
  ~1–2 hops; whether that leaves time for a splice, or needs B2/B3, will show in the logs.
