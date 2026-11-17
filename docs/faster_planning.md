# Making the planner faster

Status: analysis and proposals, 2026-11-16. Nothing here is implemented, and no code in the repo
was changed to write it. The numbers come from a scratch copy of the search (§8), run offline on
this machine (Core Ultra 7 155H) on 49 searches:
- the 8 worlds' known maps, from the start to the README goal;
- the LiDAR map each world has when its first plan is made (`mapper_check --map-out`);
- the 33 maps the node saved during the maze and stairs runs of 2026-11-12/13
  (`vis_planned_path/snapshot_*.json`, each replayed from its start to its target).

`docs/replanning_on_the_move.md` covers *when* the planner searches and why the robot hops in place
meanwhile. This doc covers *how long one search takes*, and it replaces that doc's §4 C with
measurements. `docs/faster_replanning_plan.md` (the FASTER-style scheduler) assumes a fast search
and leaves the speed work separate: this is that work.

## Summary

- **A search costs (expansions) × (landing cells per expansion) × (time per landing cell).** Today a
  search makes up to 56,000 expansions and checks ~530 landing cells per expansion, at ~0.6 µs each:
  2–25 s offline, and 1.5–3× more in the sim.
- **Too many expansions, because the heuristic sees only a fraction of the cost.** The heuristic is the
  straight-line distance to the goal. But energy is 45–83 % of a plan's cost: on the stairs every
  0.3 m hop up a tread costs 2.84, of which 2.54 is energy. From the start, the heuristic sees a
  median 15 % of the remaining cost on the stairs and 38–48 % elsewhere. A\* has to expand every
  state whose cost so far plus that estimate is below the plan's cost.
- **Too much work per expansion.** An expansion checks every landing cell the robot can reach, but
  the search ends up using very few of those checks: on flat ground, 4,100 of 349,000.
- **Exact speed-ups** give the same plans: all 49 searches gave the same path and cost, and
  `ballistic_plan_check` passes unchanged with E1:

  | Change | Effect |
  |---|---|
  | **E1.** An energy lower bound in the heuristic (closed form, free) | 2.4× in total |
  | **E2.** Check a landing cell only once A\* needs it (lazy evaluation) | with E1: 3.3×; batched and with E3: 4.2× |
  | **E3.** Cheaper landing-cell checks | 1.1× |
  | **E4.** Threads | 3.6× on 6 cores |

  **E1 + E3 + E4 together: 7.9× in total (5.1–785× per search).** The worst search drops from
  25 s to 2.6 s, and the stairs to 0.5 s median (0.9 s worst).
- **Weighted A\* on E1** (I1, inexact) adds the rest:
  - With everything above and ε = 1.2: 15× in total, 39× median, worst successful search 0.7 s.
    Plans cost +0.3 % on average (+4.2 % at worst).
  - In `ballistic_plan_check`, ε = 1.2–1.5 changes 1–3 pinned plans. Every flight check passes,
    with margins no worse than today's.
- **A failed search costs the most.** It explores every reachable state, so of the speed-ups above
  only the threads (E4) help it.
  - The logs hold 65 failed searches, median 6 s and up to 22 s each, 289 s in all.
  - Two runs on 2026-11-11 never got a plan: 10 and 20 searches to frontier cells near x = 7 m
    failed, 7 s each.
  - E5 lists ways to fail fast.
- **Recommended order** (§7):
  1. E1, a small exact change;
  2. E4 with E3, exact: ~8× together with E1;
  3. E5, failing fast;
  4. then E2 and/or I1 (ε = 1.2) behind a parameter.

  At 0.1–0.5 s per search, `docs/replanning_on_the_move.md`'s planning ahead becomes practical.

## 1. How planning works now

### 1.1 When the planner searches

- **On a hand-made map:** once per `/goal_pose` (`goalCallback` → `planAndPublish`,
  `ballistic_planner_node.cpp:250`, `:471`).
- **On a LiDAR map, also:**
  - a frontier plan while the goal is unseen;
  - a new search when the robot reaches the frontier, when the goal becomes measured, when the
    robot gives up (hold), or when the re-check of the plan fails.

  `docs/replanning_on_the_move.md` §1 has the triggers.
- **Threads:** every search runs to the end on the node's only thread, except the cut search
  (`replan.cut`), which runs on a `std::async` worker (`startCutSearch`, `:625`). Either way, a
  search runs single-threaded and one at a time.

### 1.2 One search, step by step (`search()`, `:289`)

1. **Map and fields.** Convert the GridMap to `Map2D5` with the LiDAR map contract (`convertMap`,
   `:845`). Derive the standable mask, the surface normals and the inflated clearance field (the
   planner's constructor, `hopping_astar.cpp:112`). This takes 0.2–18 ms, median 3.8 ms, in the
   logs.
2. **Targets.** The goal, if it is standable. On a LiDAR map, also up to 2 frontier cells kept
   0.5 m apart (`frontier.cpp`: a 2D flood in 1.5 m steps, nearest the goal first).
3. **A\* to each target in turn** (`:357`), until one succeeds. A target that fails costs a full
   search before the next one starts.

### 1.3 Inside the A\* (`plan_momentum`, `hopping_astar.cpp:430`)

- **State:** (cell, arrival speed), with 0.1 m cells and 0.25 m/s speed bins. Every landing arrives
  at 3.75–6.25 m/s:
  - the apex must be at least 0.7 m above the landing (`min_apex`);
  - the landing speed is capped (`V_g_max`).

  So a cell can be a state at any of ~11 speeds. A state keeps the arrival direction of its
  cheapest path (in `came_from`).
- **Start:** the robot's cell, hopping in place, a vertical arrival at √(2g·0.8) = 3.96 m/s
  (bin 4.0).
- **Goal:** landing on the goal cell steeply enough to stop: descent ≥ 56°, so a vertical takeoff
  tilts ≤ 20° (`:506`).
- **Expanding a state** (`momentum_neighbors`, `:589`):
  1. **Landing cells.** Every cell on a lattice over the disk the arrival speed can reach
     (`for_each_target`, `:256`). The radius is 1.9–3.9 m and the spacing 0.14–0.20 m, which gives
     600–1,200 cells in open terrain and 470–730 on average on these maps, clipped by the map's edge.
  2. **For each landing cell:**
     - Is it standable? A lookup.
     - The feasible takeoff-angle interval (`feasible_alpha_interval_vertical_push`,
       `ballistic_physics.cpp:169`). It covers the energy band, the apex rule, the landing-speed
       cap, both friction cones and the 42° leg-angle floor: some 20 trig and square-root calls.
     - The clearance floor (`clearance_floor_alpha`, `:336`). The straight line between the two
       cells is walked in 3.3 cm steps over the inflated field: up to ~120 samples for a 4 m hop.
     - On the goal cell, the stop rule.
  3. **Takeoff angles.** Up to 5 per landing cell: the least-injection one and 4 "banking" ones
     that land 1–4 speed bins faster. For each, compute the leg speed and angle, the stance and
     its landing tilt (≤ 20°) and the push capacity. Then compute the hop's numbers and its edge
     cost.
- **Edge cost:** X + w·(injection + speed lost), with w = 1.86 m/J (`edge_cost`, `:395`).
- **Heuristic:** the straight-line distance to the goal (`:149`).
- **Bookkeeping:**
  - The open list is a binary heap with a FIFO tie-break.
  - There is no closed list: stale entries are skipped.
  - `g` and `came_from` live in hash maps.
- **At the end:** the landing tilts are re-checked with the exact arrival directions (`:477`), and a
  failure returns no plan.

### 1.4 Sizes and times (offline, this machine)

| Map | Expansions | Landing cells checked | Time |
|---|---|---|---|
| known flat, 4 m | 3,702 | 2.2 M | 2.2 s |
| known maze | 21,581 | 15.7 M | 11.6 s |
| known cliff_gap | 56,021 | 33.1 M | 24.8 s |
| known slope_30deg | 20,811 | 10.7 M | 8.1 s |
| known stairs | 16,215 | 7.6 M | 5.2 s |
| LiDAR, first plan, 8 worlds (slanted_pads: no path) | 3,354–16,710 | 2.0–9.5 M | 1.4–8.1 s |
| node snapshots, maze (7) | 183–8,630 | 0.1–6.5 M | 0.05–2.7 s, median 1.8 s |
| node snapshots, stairs (26) | 6,969–15,558 | 3.5–7.3 M | 2.0–4.9 s, median 3.2 s |

Over all 49 searches the average is 534 landing cells per expansion, 0.34 ms per expansion and
0.64 µs per landing cell. In the sim the same searches took 1.5–3× longer: the sim shares the CPU
with Gazebo and FAST-LIO (`docs/replanning_on_the_move.md` §2.3).

## 2. Why a search takes seconds

### 2.1 The arithmetic

For the known maze: 21,581 expansions × 725 landing cells × 0.74 µs = 11.6 s. Each factor can be cut:

- **fewer expansions:** a better heuristic (E1, I1);
- **fewer landing cells checked per expansion:** lazy evaluation (E2);
- **less time per check:** cheaper code (E3) and more cores (E4).

### 2.2 The heuristic sees a fraction of the cost

A plan's cost is its distance plus w × its energy, and the energy part is large:

| Plan (known map) | Cost | Distance | Energy share |
|---|---|---|---|
| flat, 3 hops | 7.7 | 4.0 | 48 % |
| maze, 7 hops | 25.5 | 14.1 | 45 % |
| lilypads, 7 hops | 22.3 | 12.0 | 46 % |
| cliff_gap, 13 hops | 34.2 | 9.5 | 72 % |
| slope_30deg, 9 hops | 26.7 | 4.9 | 82 % |
| stairs, 9 hops | 28.0 | 4.9 | 83 % |

Every hop pays for energy, however short:
- **Apex rule.** The apex must be ≥ 0.7 m above the landing, so each landing arrives at ≥ 3.7 m/s.
- **Stance loss.** The stance gives back η = 77 % of that energy, so the push must make up the rest.

On the stairs the plan climbs one tread per hop: 0.30 m forward and 0.18 m up, at 2.84 a hop.

A\* expands every state whose cost so far plus the heuristic is below the plan's cost. So the share
of the remaining cost the heuristic sees sets the size of the search. From the start, it sees:

| Maps | Today (straight line) | E1 (energy bound) | Relaxed hop graph (§5) |
|---|---|---|---|
| known (8) | 16–55 %, median 48 % | 56–87 %, median 77 % | median 79 % |
| LiDAR first plans (7) | 14–52 %, median 38 % | 52–82 %, median 73 % | median 75 % |
| maze snapshots (7) | 34–53 %, median 46 % | 55–84 %, median 72 % | median 75 % |
| stairs snapshots (26) | 10–22 %, median 15 % | 49–74 %, median 55 % | median 58 % |

### 2.3 Most landing-cell checks are wasted

An expansion checks all ~530 landing cells, but A\* only ever needs the successors whose f is below
the plan's cost. With E1 and lazy evaluation (E2), which checks a landing cell only when A\* needs it:

| Map | Checks with E1 | Checks with E1 + E2 |
|---|---|---|
| known flat | 349,000 | 4,100 (85× fewer) |
| LiDAR maze | 747,000 | 15,000 (49×) |
| known maze | 5.4 M | 0.78 M (7×) |
| known slope_30deg | 4.1 M | 1.75 M (2.3×) |
| stairs snapshot 11-13 11:11:07 | 3.6 M | 1.5 M (2.4×) |

### 2.4 Each check is expensive

Stack samples of the search (gdb, `docs/replanning_on_the_move.md` §2.4):

| Part | Share of the time |
|---|---|
| Generating neighbours (`momentum_neighbors`) | 99 % |
| └ clearance | 26 % |
| └ feasible interval | 21 % |
| └ per-angle work: stance, push, the hop's numbers, mostly trig | ~48 % (stance 14 %) |
| Hash maps and allocation | ~5 % |

Two things make this more wasteful:
- **The same work for every speed.** A cell is expanded at 3–10 speeds (flat 3.2 on average; maze
  7.1; slope 9.6; stairs snapshot 10.5). The clearance walk and the friction cones depend only on
  the two cells, but they are redone each time.
- **Wasted clearance walks.** 49–100 % of them find nothing higher than both ends ("inactive") after
  walking every sample.

### 2.5 A failed search is the worst case

A target that cannot be reached is only proven unreachable once every reachable state has been
expanded. No heuristic or weight helps with that: only a faster expansion does.

- **Logs:** 65 failed searches, median 6.1 s and up to 21.6 s each, 289 s in all (the 345 successful
  ones took 2,627 s).
- **Two runs of 2026-11-11 (23:53 and 23:59) never got a plan.** From the start, 10 and 20
  searches to frontier cells near x = 7 m failed, 6–9 s each: 73 s and 144 s of searching while
  the robot hopped in place.
- **Offline:** the LiDAR slanted_pads frontier fails after 3.6 s. With E1, E2 or any weight it
  still takes 3.1–3.9 s; with 6 threads (E4), 0.9–1.0 s.

## 3. Exact speed-ups: the same plans

### E1. An energy lower bound in the heuristic

Each hop's edge cost is X + w·(e_inject + max(0, ke_in − ke_out)). Two facts bound its energy part
from below, for a hop of length X and rise Z from an arrival at v_in:

- (a) the injection plus the speed lost is at least (1 − η)·½m·v_in² + m·g·Z − δ. Here δ ≤ 0.29 J
  is the most that rounding the landing speed to its bin can add;
- (b) the injection alone is at least ½m·(v_s,min² − η·v_in²), where v_s,min is the slowest takeoff
  that reaches (X, Z) with the apex rule.

Every landing arrives at 3.75–6.25 m/s. Taking the least of max(a, b) over that range gives a floor
F(X, Z) for every hop. Numerically, X + w·F(X, Z) ≥ κ·X + c·Z for every hop the robot can fly, with:

| (κ, c) | |
|---|---|
| (1.00, 0) | today's heuristic |
| (1.19, 2.0) | |
| (1.38, 4.0) | |
| (1.58, 6.0) | e.g. on flat ground every metre of hop costs at least 1.58, where today's heuristic counts 1.0 |

Along any path the c·Z terms add up to c·(z_goal − z), so

  h = max over the pairs of κ·d + c·(z_goal − z),  d = straight-line distance.

- **Admissible and consistent:** each pair is, so their max is too. In all 49 searches (140 M
  landing-cell checks), no generated edge cost less than its bound.
- **Free:** one lookup per cell.
- **Fit to the robot:** the pairs depend on the robot's parameters (mass, η, `w_energy`, `min_apex`,
  `V_g_max`, `speed_bin`). So compute them at startup with a small 2D scan over (X, Z), as
  `exp2/kappa.py` does, and do not hard-code them.

**Measured:**
- 2.4× in total, 2.7× median, at least 1.6×;
- the same path and cost in all 49 searches, and `ballistic_plan_check` passes with every pinned
  plan unchanged;
- known flat 2.2 s → 0.33 s, known maze 11.6 s → 3.4 s, stairs snapshots median 3.2 s → 1.6 s.

**Code:** `heuristic()` (`hopping_astar.cpp:149`), only when `momentum_aware`, so the reference search
and `ballistic_parity_test` stay byte-identical. Precompute h per cell instead of calling `hypot`
per push.

### E2. Check a landing cell only when A\* needs it (lazy evaluation)

When a state is expanded:
1. For every landing cell in reach, compute a cheap lower bound of its f:
   g + X + w·LB(X, Z, v_in) + h(landing). LB is (a)/(b) of E1 at the state's own arrival speed.
   This takes ~20 ns per cell, against ~0.6 µs for a full check.
2. Fully check only the cells whose bound is due: no higher than the best f on the open list, plus a
   slack.
3. Keep the rest in a list sorted by bound (8 bytes each) and put the state back on the open list
   at the least of them.
4. When that entry comes up, check the next due cells.
5. If a cheaper path to the state is found, its list is rebuilt.

This is Partial Expansion A\* (Yoshizumi et al. 2000; Felner et al. 2012). Lazy Weighted A\*
(Cohen et al. 2014) is a close relative.

- **Exact.** A deferred cell's real f is at least its bound, and its state waits on the open list
  at that bound, so A\* still expands in order of f. Same path and cost in all 49 searches,
  at every slack.
- **Measured** with E1 and E3, single thread:

  | Slack | Total | Median | Notes |
  |---|---|---|---|
  | 0 | 3.5× | 5.1× | maze snapshots 1.8 s → 0.08 s; LiDAR maze 1.35 s → 0.04 s; known flat 2.2 s → 0.02 s |
  | 0.3 | 4.2× | 6.1× | stairs snapshots 3.2 s → 1.1 s median |

- **The slack is a batch size.** At slack 0 a pop often checks a single cell, and the pops'
  overhead eats much of the gain. A slack of 0.3–1 (in cost units) checks a few cells at once,
  for a few wasted checks.
- **Memory.** Up to ~8 KB of deferred cells per expanded state. On cliff_gap (24,000 expansions)
  the peak is 160 MB against 25 MB without E2. Free a list once it is used up; it could be
  6 bytes an entry, or capped.
- **With threads,** E2 needs batching. At slack 1 and 6 threads it is 7.1× in total and 10.1×
  median: the best median of the exact setups, but a longer tail than E4 without E2 (§6).

### E3. Cheaper landing-cell checks

Measured, all exact:

- **The lattice as a table.** For each speed bin, store the landing lattice once as cell offsets. This
  drops a hash-set insert and the coordinate conversions per landing cell (`for_each_target`,
  `:262`). With h stored per cell, 1.1–1.2×.
- **A clearance early-out.** A 2D range-max table over the inflated field answers "is anything along
  this line higher than both ends?" with 4 lookups per half metre. It skips the walk for 24–100 % of
  checks (flat 100 %, stairs 48–59 %, maze 27–42 %) and saves ~5 %. Finer pieces, or walking only
  the pieces that may be active, would skip more.
- **`-march=native -ffp-contract=off`.** ~1.06×, single-threaded and with 6 threads, with the
  same plans: within the noise of a run. Fused multiply-add would change rounding, so `-ffp-contract=off` keeps it off.
  `ballistic_parity_test` would confirm it before adopting.

Not measured, each straightforward:

- **The feasible interval in tan-space.** `atan` is monotonic, so every max/min in
  `feasible_alpha_interval_vertical_push` can compare tangents. One conversion is left, at the end.
  This targets the 21 %.
- **The tilt cap as `axis.z ≥ cos 20°`,** with no `acos`. Check the push before the stance.
- **Fill in a hop's fields only for a successor that improves a state.** Today the heading, the leg
  angle, β and the apex drop are computed for every one of up to 5 angles per landing cell, and
  most of them are thrown away. This targets the ~48 %.
- **Cache the speed-independent part per cell pair** (clearance, cones, apex rule). Each cell is
  expanded at 3–10 speeds (§2.4). Keep ~4 bytes per landing cell for each expanded cell, ~10–30 MB.
- **Flat arrays for `g`, `came_from` and the lazy state,** indexed by cell × 64 bins: ~5 %, more
  with E2.

### E4. Threads

The landing cells of one expansion are independent. Check them in parallel (OpenMP), then merge the
successors in lattice order, which keeps the same push order and so the same plans.

- **Measured:**

  | Setup | Total | Per search |
  |---|---|---|
  | today's heuristic, 6 threads | 3.6× | 3.1–5.1× |
  | today's heuristic, 20 threads | no better than 6 | |
  | **E1 + E3 + E4, 6 threads** | **7.9×** | 8.1× median, 5.1× at least |

  For E1 + E3 + E4 the worst search is 2.6 s (cliff_gap) instead of 25 s, and the stairs snapshots
  take 0.52 s median (0.88 s worst). More than 6 threads does not help: the rest are hyper-threads and
  efficiency cores. In the sim, Gazebo, FAST-LIO and the mapper also want cores.
- **With lazy checks (E2), threads need batching.** Without a slack a pop checks one or two cells,
  and threads made it about 2× slower (6 threads) and 10× slower (20). Batch with a slack (E2's
  table), or check the due cells of several states at once (ePA\*SE, Mukherjee et al. 2022, made
  for search with expensive edges).
- **Also parallel:** `search()` tries the goal and the frontier cells one after another (`:357`).
  Run them side by side, or as one search (E5).

### E5. Fail fast

- **Budget.** Give each search a budget (expansions or time), and on overrun move on to the next
  target. A budget makes it inexact only for targets that would have succeeded late.
- **Don't retry.** Remember a target that failed (start cell, target cell, map stamp) and do not
  search it again until the map near it changes. This is the case of the two runs in §2.5.
- **One search for all targets.** Search the goal and the frontier cells in one A\*, with h = the
  least over them, and stop at the first reached. Today each failed target is a full search before
  the next one starts.
- **Relaxed reachability, before the search.** Run a 2D Dijkstra over landing cells, with each
  hop's floor F(X, Z) as its cost (§5, 30–130 ms), and drop a target no hop sequence reaches.
  It is cheap, but weak: it did not catch the benchmark's failing slanted_pads frontier, which
  fails on constraints the relaxation ignores.

## 4. Inexact speed-ups: plans may cost more

### I1. Weighted A\* on top of E1

f = g + ε·h bounds the plan's cost by ε times the best one.

| Setup | Total | Median | Median time (known / LiDAR / maze snapshots / stairs snapshots) | Worst | Plan cost, mean / worst |
|---|---|---|---|---|---|
| E1+E2, ε = 1.2 | 6.5× | 29× | 81 / 210 / 4 / 665 ms | 1.8 s | +0.3 % / +4.2 % |
| E1+E2+E3+E4 (slack 3, 6 threads), ε = 1.2 | 15× | 39× | 110 / 147 / 12 / 335 ms | 0.7 s | +0.3 % / +4.2 % |
| E1+E2, ε = 1.5 | 14.5× | 460× | 3 / 2 / 2 / 166 ms | 0.9 s | +3.0 % / +14.6 % |
| E1+E2, ε = 2 | 31× | 910× | 2 / 2 / 2 / 6 ms | 0.3 s | +4.8 % / +16.2 % |
| today's heuristic, ε = 2 | 1.7× | 1.4× | 296 / 908 / 21 / 2,424 ms | 18 s | +0.6 % / +6.9 % |

ε means more on the tighter heuristic: ε = 2 on the straight line is about ε = 1.25 on E1. So use
1.1–1.3 here. With ε = 1.5 on E1, a few plans gain hops (maze: 11 instead of 7, +11 %).

**Flight checks** (a scratch build of `ballistic_plan_check` with E1):

| ε | Pinned plans that differ | Flight checks | Worst leg angle (guard 40°) | Worst landing tilt (cap 20°) | Worst push |
|---|---|---|---|---|---|
| 1 | none | all pass | 48.6° | 18.2° | 98 % of capacity |
| 1.2 | flat (4,0) | all pass | 48.6° | 18.2° | 98 % |
| 1.5 | flat (4,0), low_wall (4,0) and (4,1.5) | all pass | 51.9° | 17.2° | 98 % |

So the margins are no worse than today's. With today's heuristic at ε = 2, one case had gone from
55.9° to 42.7°. The LiDAR maps (`--lidar-maps`) still need the same check.

### I2. Anytime search

Search at ε = 2 first (median 2–6 ms, worst 0.3 s) and publish, so the robot can go. Then keep
searching at a lower ε while time remains. ARA\* (Likhachev et al. 2003) reuses the first search's
work for this. An improved plan replaces the old one only if it differs after the hop the robot is
committed to (`docs/replanning_on_the_move.md` §3). This bounds the wait by the greedy search,
not the exact one.

### I3. A coarser model (measured, poor trades)

| Knob | Total | Plan cost, mean / worst |
|---|---|---|
| `bank_bins` 4 → 2 | 3.9× (E1+E2: 3.3×) | +0.8 % / +12.5 % |
| `bank_bins` 4 → 0 | 3.7× | +30 % / +103 % |
| `hop_scan_step` 0.1 → 0.15 | 5.2× | +16 % / +64 % |
| `hop_scan_step` 0.1 → 0.2 | 12× | +11 % / +67 % |
| `speed_bin` 0.25 → 0.5 | 8.3× | −7 % / −2 %, a different model |

All of these are on top of E1+E2.
- The banking angles and the fine lattice earn their keep. Weighting (I1) buys more speed for less
  cost.
- With coarser speed bins, rounding hands out more free speed, so their costs are not comparable.
  Such plans would need the flight checks before anything else.

### I4. A bounded horizon

Search only up to N hops (or R metres) toward the target, and cost the states at the edge with the
heuristic. This bounds the worst case whatever the map. It can choose the wrong side of an obstacle
beyond the horizon. On a LiDAR map the plan beyond the seen area is a frontier plan anyway, and it
is replanned on the move (`docs/replanning_on_the_move.md` A3).

## 5. Measured and not worth it

- **An obstacle-aware distance in the heuristic.** A 16-connected Dijkstra around OBSTACLE cells,
  scaled to stay a lower bound. On the 5 searches tried, it made almost the same expansions as E1
  (the LiDAR maze: 1,107 against 1,116). The straight line from the start to these goals is rarely
  blocked by an infinitely tall obstacle: the maze's poles and side walls sit off it. The earlier
  doc suggested it, and so does `docs/faster_replanning_plan.md` §4.1. As that plan's horizon
  router it may still earn its place, but not as a heuristic.
- **A relaxed hop graph as the heuristic.** A Dijkstra over landing cells, each hop costing
  X + w·F(X, Z) (the per-hop floor of E1), so it knows which cells can be landed on. It sees only
  1–5 points more of the cost than E1, and costs 30–130 ms per search.

  Most of the stairs' gap (E1 sees 55 %) is in what any per-hop bound ignores:
  - the speed carried from hop to hop;
  - the 4 J/kg and push-capacity caps per hop;
  - the clearance over the risers;
  - the tilt cap when turning;
  - the vertical start and the steep stop.

  A heuristic that knew these would have to search over speeds too, and be as slow as the search.
- **Dominance between speeds.** "Same cell, faster and cheaper" does not dominate, because the cost
  charges for the speed lost later. So no state can be pruned that way.

Not tried, and not recommended now:
- **Incremental search (D\* Lite, LPA\*).** The start moves and carries a speed and an arrival
  direction. The target moves with the frontier. The LiDAR map changes at 2 Hz. Little would be
  reused for a lot of complexity.
- **A GPU.** The work is ~500 small, branchy checks per expansion, in sequence.

## 6. Combined

All on the same 49 searches, as the median / worst time in ms per group, from one back-to-back sweep
(`runs3/`). Its two baselines, at the start and the end, agree within 1.2 % (191.5 s and 193.7 s).
The failed LiDAR slanted_pads search is left out of the groups, but counted in the total. Every row
marked "same" gave the same path and cost as today in all 49.

| Setup | Total (×) | Median (×) | known | LiDAR | maze snapshots | stairs snapshots | Plan cost |
|---|---|---|---|---|---|---|---|
| today | 191 s | | 4,324 / 24,830 | 3,112 / 8,113 | 1,808 / 2,696 | 3,208 / 4,896 | |
| E3 (table) | 168 s (1.1×) | 1.1× | 3,777 / 22,190 | 2,784 / 7,290 | 1,537 / 2,284 | 2,793 / 4,308 | same |
| E1 | 79 s (2.4×) | 2.7× | 2,238 / 8,607 | 1,683 / 3,153 | 502 / 588 | 1,626 / 2,809 | same |
| E1+E2 | 57 s (3.3×) | 5.1× | 1,102 / 6,693 | 528 / 1,751 | 79 / 92 | 1,363 / 2,605 | same |
| E1+E2+E3, slack 0.3 | 46 s (4.2×) | 6.1× | 956 / 4,844 | 481 / 1,436 | 90 / 100 | 1,118 / 2,006 | same |
| E4 alone, 6 threads | 54 s (3.6×) | 3.4× | 1,242 / 6,659 | 888 / 2,348 | 510 / 753 | 959 / 1,434 | same |
| **E1+E3+E4, 6 threads** | **24 s (7.9×)** | 8.1× | 683 / 2,590 | 478 / 976 | 172 / 183 | 515 / 879 | same |
| E1+E2+E3+E4, slack 1, 6 threads | 27 s (7.1×) | 10.1× | 620 / 2,677 | 320 / 837 | 96 / 111 | 669 / 1,131 | same |
| E1+E2+E3+E4, slack 3, 6 threads, ε = 1.2 | 13 s (15×) | 39× | 110 / 639 | 147 / 502 | 12 / 20 | 335 / 711 | +0.3 % / +4.2 % |

Without E2, E1 + E3 + E4 has the best worst case. With E2 the typical search is faster but the tail
is longer. Batching lazy checks across states (ePA\*SE) might get both. In the sim, expect these
times ×1.5–3 for the shared CPU, unless the planner gets cores of its own.

## 7. Recommendation

1. **E1 in `heuristic()`, momentum search only.**
   - Exact, a few dozen lines, 2.4× in total.
   - `ballistic_plan_check` keeps every pinned plan, and `ballistic_parity_test` is untouched.
   - Compute the (κ, c) pairs at startup from the parameters.
   - Add a test that no generated edge costs less than its bound, as the scratch copy did.
2. **E4 + E3: threads, the lattice table and the clearance early-out.**
   - Exact, with a deterministic merge.
   - 8× in total with E1; the stairs drop to ~0.5 s and the worst search to ~2.6 s offline.
   - Cap the threads at what the sim leaves free (try 4–6).
3. **E5: fail fast.** A budget per search, no retry of a failed target until the map near it
   changes, and the goal and frontier cells in one search. This removes the 6–20 s failures and
   the two stuck runs.
4. **Then, measured in the sim:**
   - E2, batched, if the typical search matters more than the tail;
   - and/or I1 at ε = 1.2 behind a parameter (`search.heuristic_weight`), with `ballistic_plan_check`
     (new pins) and `--lidar-maps` passing at that ε;
   - or I2 (greedy first, exact in the background), which fits `docs/replanning_on_the_move.md`'s
     splice.
5. **Measure** with the node's `[TIMING]` lines, before and after each step, on maze, slope,
   cliff_gap and stairs.

E1, E2 and the moving start of `docs/faster_replanning_plan.md` (its step 1) all change
`plan_momentum`, so land E1 first.

## 8. How this was measured

All in the scratchpad of the session that wrote this doc, outside the repo:
`/tmp/claude-1000/-home-zweminhtetaung-hopcopter-bmp/7cc4b22c-bdfe-4f5e-86b4-91f1f6333867/scratchpad/`.

| File (under `exp2/` unless named) | What it is |
|---|---|
| `fast_astar.{hpp,cpp}` | The momentum search, copied from `hopping_astar.cpp`, with knobs: `h0`/`h2`/`h3`/`h4` (heuristics: today, E1, geodesic, relaxed graph), `w` (ε), `t` (lattice table), `l<slack>` (lazy), `c` (clearance early-out), `j<n>` (threads), `v` (count edges below their bound), and `B`/`S`/`P` (`bank_bins`, `speed_bin`, `hop_scan_step`). It links the repo's `ballistic_physics.cpp`, `map2d5.cpp` and `frontier.cpp` unchanged. |
| `bench.cpp` | Runs the repo's `HoppingAStarPlanner` (baseline) or `fast_astar` on the 49 searches, and compares paths and costs with the baseline's. |
| `maps/` | `mapper_check`'s LiDAR maps of 2026-11-11 20:47, copied. |
| `snaps/` | The node's snapshots, converted from JSON. Rebuilt with the node's margins: edge 0.1, unseen 0.3, steep noise 0.02, edge drop 0.3. |
| `kappa.py` | The (κ, c) pairs of E1. |
| `hops.cpp` | The per-hop cost breakdown of §2.2. |
| `sweep3.sh`, `runs3/`, `summarize.py`, `doc_numbers.py` | The runs behind §6 and the timings quoted. `sweep.sh`/`sweep2.sh` (`runs/`) were the first pass. |
| `exp3/` | `ballistic_plan_check` built with E1 and ε (`EXP_H2=1 EXP_WEIGHT=…`), for I1's flight checks. |

Build:
`g++ -O3 -DNDEBUG -std=c++17 -fopenmp` with the package's include paths, like the package's Release
build. With `h0` and no lazy checks, the scratch search reproduces the repo's exactly: the same
expansions, landing-cell checks and paths. Timings are wall-clock on an otherwise idle machine. The
first pass (`runs/`) began while the machine ran ~17 % slower, which inflated some of its ratios, so
every timing here comes from `runs3/`. Counts, costs and paths are deterministic.

## 9. Open questions

- **Sim timings.** The sim runs a search 1.5–3× slower. How much of that is CPU contention, and how
  many cores can the planner have while Gazebo and FAST-LIO run?
- **Pinned plans.** Should the pinned plans in `ballistic_plan_check` follow a weighted search
  (new pins), or should the check keep ε = 1?
- **The stairs' remaining gap.** Is a speed-aware lower bound worth it? For example, a backward
  Dijkstra over (cell, speed) with the tilt ignored, on a coarser grid.
- **Lazy-check memory** in the worst case (160 MB on cliff_gap): cap it, or fall back to full
  expansion past a size.
- **Why the slanted_pads frontier fails.** Its 4 s failure is a real case for E5.
