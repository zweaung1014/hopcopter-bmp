# FASTER-style replanning on the move: plan

Status: plan, 2026-11-16. No code changed. Source: `docs/references/FASTER.pdf` (Tordesillas et al.,
"FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments", T-RO 2021).
This turns ideas A3, A4 and D4 of `docs/replanning_on_the_move.md` into one design, built on the cut
(`docs/replan_cut_plan.md`). The search's speed (C1 there) is separate work. This plan assumes a
fast search and says what happens when one is slow. It applies to LiDAR maps only; the known-map
mode is unchanged.

## Decision (2026-11-16): a 2D cost-to-go through the unknown, no A* there

The hop A* plans only on measured ground; no terrain is guessed for unseen cells. Past the measured
ground, the route to the goal is a **2D cost-to-go** (§3 step 0). FASTER makes the same split: a
cheap grid path through unknown space (JPS), and the expensive planner only near the robot.
Planning on a guessed terrain (an optimistic map) was considered and dropped. It stays a later
option, if the heading at R proves poor.

## 1. What FASTER does

The map holds free-known space F, occupied space O and unknown space U. At every replanning step k,
FASTER computes three trajectories:

- **Whole** (A → E): in F ∪ U (unknown treated as free), ending in a stop at E, near the global
  route. The horizon is long, so the first part is fast.
- **Safe** (R → F): in F only, ending in a stop at any point F the optimizer picks. H is where Whole
  first enters U. R is the last point on Whole before H from which the drone can still stop without
  entering U.
- **Committed** = Whole[A → R] + Safe[R → F]. This is what the drone flies.

What matters:

1. **A is in the future.** A new plan starts from the point of Committed_{k−1} the drone will reach
   δt = α·Δt_{k−1} from now (Δt_{k−1} is the last replanning time, α ≈ 1.25), with the state it will
   have there. The plan is ready before the drone gets there, so the handover is seamless.
2. **Two separate optimizations.** Whole is solved as if the flight continued into U, so A → R is not
   shaped by any braking. The braking lives only on Safe (R → F).
3. **Keep the old plan when the new one fails.** The drone keeps flying Committed_{k−1} if either
   optimization is infeasible, if A → R crosses U, or if replanning takes longer than δt. It always
   holds a plan that ends stopped in F. By induction it is always safe (their Theorem 1, which
   assumes a static, noise-free map).
4. **So it brakes only when it has to.** If each new plan comes in time, the drone switches onto it at
   A, before R, and never flies a Safe trajectory.

The rest of the paper is trajectory machinery: JPS, convex decomposition, the MIQP, interval and time
allocation. None of it carries over, because the hop A* does that job here. One small idea does
carry over. To stop flip-flopping between routes, FASTER weighs the new route against the previous
one, repaired around new obstacles, and keeps the cheaper (§III-E, Fig. 10).

## 2. FASTER on a hopper

| FASTER | Hopcopter |
|---|---|
| continuous trajectory; commits δt ahead | hops. The hop out of waypoint j is committed at the apex of the flight into j (`_decide_and_steer`, `hopcopter.py:904`) |
| L, the drone now | the flight in the air: it lands on `land_index − 1`, and its next hop, into `land_index`, is committed |
| A, δt ahead on Committed_{k−1}, with its state | the hand-over waypoint A ≥ `land_index`: the first whose outgoing hop is decided after the plan is ready. The search starts there with the planned arrival into A (speed, heading, descent angle) |
| U treated as free; JPS through it | a 2D cost-to-go from the goal: unseen cells free, known walls and cliffs blocking |
| Whole, A → E | the momentum A* from A's moving start over measured ground, ending at the goal or at an *exit* next to unseen ground, scored with the cost-to-go from there |
| H, where Whole enters U | the exit: Whole's last waypoint, on measured ground next to unseen cells |
| R, the last point before H that can stop in F | the last Whole waypoint before H that has a Safe branch |
| Safe, R → F | a stop on R (hop in place), or 1–2 braking hops over measured ground that end in a stop |
| Committed | the path hopcopter is given: Committed_{k−1} up to A, Whole from A to R, then Safe from R to F |
| braking | a stop needs an arrival at least 56° steep (a vertical takeoff within the 20° landing tilt). The search makes a plan's last hop steep enough (`beta_stop`, `hopping_astar.cpp:607`) |

```
 prefix of Committed_{k-1}       Whole_k (fast)            Safe_k
 ●───────●───────●───────●═══════●═══════●═══════● R ───▶ ■ F   stop: flown only if the next plan is late
      in the air │       A                        ╲
      lands here └ next hop committed              H (exit) · · · · · · · G (goal)   past H: the 2D
                                                                                cost-to-go route, never flown
```

**What F buys this robot.**
- Today every plan ends in a stop, at the goal, a frontier or a cut waypoint. The robot always flies
  that stop, because the next search starts only once it hops in place there
  (`replanning_on_the_move.md` §1). From goal to arrival it hopped in place 55–90 % of the time on
  the logged runs.
- With FASTER the plan still always ends in a stop (F), so the robot never commits to a landing the
  planner has not checked. But that stop is only a backup: the next plan replaces everything after A
  before the robot reaches R.
- For this robot the braking itself is cheap. A 1.5 m hop with a 0.8 m apex arrives ~65° steep and
  can stop in one landing; the cut's stop test passed on every waypoint of the 18 logged stairs
  plans. Only the fastest hops arrive as shallow as ~49°: the leg alone supplies v_h, at most
  √η·cos 42° ≈ 0.65 of the arrival speed. Those need one braking hop.
- Today the search keeps such hops out of a plan's last hop. Ended at an exit scored by the
  cost-to-go instead of a stop at the frontier, they stay fast up to R.
- So nearly all of the gain is not stopping to wait for a search. Less braking before R adds a
  little.

## 3. The replanning step

Jobs run one at a time on a worker thread: the cut's `std::async` machinery
(`ballistic_planner_node.cpp:625–674`), generalized. A new job starts when the last one has finished
and a newer map has arrived (2 Hz), while a goal is active. The first plan after a goal starts from
the robot hopping in place, as today, but on the worker.

**Inputs, as a snapshot:**
- the map;
- Committed_{k−1} and its stamp;
- the latest ApexDecision: `land_index`, and t_a, when it arrived;
- the hop period T: the median apex-to-apex interval, ~0.88 s. It is measured in wall time, so it
  follows the sim's real-time factor;
- Δt_{k−1} (0.5 s before the first job).

1. **Choose A.** The hop out of waypoint j is decided at the apex into j, around
   t_dec(j) = t_a + (j − land_index + 1)·T. Take the smallest A ≥ land_index + `lead` with
   t_dec(A) − now − margin ≥ α·Δt_{k−1} (α 1.25, margin 0.1 s).
   - `lead` 0 is the hard deadline.
   - `lead` 1 also beats the apex into A − 1. At that apex hopcopter already reads waypoint A + 1 to
     predict the landing axis on A (`_lookahead_axis`, `hopcopter.py:836`). A later splice shifts
     that landing by up to ~0.1 m.
   - A on Committed_{k−1}'s Safe branch (the robot is braking): the moving start works there too.
   - A past F: plan from F with a vertical arrival, as today.

   | search time α·Δt | A, if the job starts just after an apex | just before the next apex |
   |---|---|---|
   | 0.1 s | land_index | land_index + 1 |
   | 0.5 s | land_index | land_index + 1 |
   | 2 s | land_index + 2 | land_index + 3 |

0. **Cost-to-go**, once per map update, shared by every search of that map: a Dijkstra pass outward
   from the goal over the 2D grid, 8-connected.
   - **Free:** unseen cells, bounded or not.
   - **Blocked:** cells off the map, and moves between measured cells that climb more than
     `ctg.max_rise` (0.9 m, the frontier flood's limit: about the most one push climbs). Drops are
     allowed. Walls the robot can hop do not block.
   - **Result:** every cell holds the length of the shortest route to the goal around what is known.
     On open ground it equals the straight-line distance; in front of a seen wall it is the way round
     (through the opening), which is what a straight line gets wrong. A few ms on the worlds' grids.
2. **Whole.** The momentum A* from A (moving start, §4.1) on the real map: measured, standable cells
   only, as today. It may end in either of two ways:
   - **at the goal**, with the stop there, as today;
   - **at an exit**: a standable landing within `exit.band` (0.5 m) of unseen cells, with no stop
     required. It is scored as its cost so far + the cost-to-go from it + `exit.heading_weight` ×
     the angle between the arriving hop's heading and the route's direction there (the cost-to-go's
     downhill direction). A robot arriving at the exit moving the route's way can carry on.

   The cheapest end wins. The cost-to-go is also the search's heuristic: it is a distance that ignores
   energy, so it never overestimates (it replaces the straight-line distance, `hopping_astar.cpp:149`).
   If the search finds no end, keep Committed_{k−1}.
3. **H** is Whole's last waypoint: the exit, or the goal. At the goal, R is the goal and Safe is the
   stop already planned there.
4. **R and Safe.** H lies on measured ground, so it can be R itself. For j from H down to A + 1, the
   first j with a Safe branch is R:
   - Safe0 is a stop on j: `stopStance(hop into j).tilt ≤ 20°` (`ballistic_planner_node.cpp:555`),
     `cutIndex`'s test (:566);
   - otherwise, Safe1–2 is a short search from j, with Whole's arrival there, for the cheapest landing
     on measured ground that can stop, in at most `safe.max_hops` (2) hops (§4.1). The cost is the
     edge cost + the cost-to-go from F, so the branch brakes along the route, not sideways or back.

   If no j qualifies (R ≤ A), Whole gains nothing over the old plan (FASTER's scenario 2). Keep
   Committed_{k−1}; the next job tries again on a newer map.
5. **Committed_k** = Committed_{k−1}[0..A] + Whole[A..R] + Safe[R..F], waypoints and hops.
   - The stance at A turns Committed_{k−1}'s arrival into Whole's first hop. The moving start makes
     the search compute it.
   - Ground heights come from the real map.
   - Indices up to A are unchanged, so `land_index` names the same waypoint on both plans.
6. **Gates**, on the node's thread when the job returns:
   - *stale*: Committed_{k−1} is no longer the active plan (a cut, a new goal or a DELETE). Discard.
     The cut's generation number already covers this.
   - *late*: the latest decision has `land_index > A`, or now + margin > t_dec(A). Discard (FASTER's
     scenario 3). The robot keeps Committed_{k−1}, and the next job picks a later A.
   - *out of date*: re-check Committed_k from A on the newest map. A bad hop after A cuts it there
     (Safe0); a bad hop at A discards it.
   - *churn* (v2, optional): publish only if Committed_{k−1} fails the newest re-check, R_k lies
     further along than R_{k−1}, or Committed_k's cost from A is lower by a margin. This is FASTER's
     choice between JPS_a and JPS_b. v1 always publishes, as FASTER does, and logs how far the first
     new hop turns.
7. **Publish** the HopPlan and the marker pair with one stamp. The HopPlan gets three new fields:
   `base_path_id` (Committed_{k−1}'s stamp), `splice_index` (A) and `safe_index` (R).
   - The planner keeps Committed_{k−1} until a decision carries the new stamp.
   - A decision on the old stamp with `land_index > A` means hopcopter rejected the splice (§4.4):
     revert and plan again.

Δt_k is the job's wall time, map conversion included.

**Between jobs, nothing changes.** Each map still re-checks the active plan, and a bad hop cuts it at
once (the cut). This covers the assumption FASTER relies on and this map breaks: the LiDAR map is not
noise-free, so a plan can turn bad before any new plan is ready.

**When replanning keeps failing**, the robot flies Committed_{k−1} to R, takes the Safe branch and
hops in place on F. The next job starts from F with a vertical arrival, and its plan goes out once the
robot is there (the cut's release, `ballistic_planner_node.cpp:678`). That is today's behaviour, so
FASTER mode is never worse than now.

## 4. Changes

### 4.1 Search core (`ballistic_core`, no ROS)

- **Moving start.** `HoppingAStarPlanner` takes an optional start arrival: speed v_g, heading and
  descent angle β. It sets:
  - the start's speed bin (`hopping_astar.cpp:441`) and hop radius;
  - the start's `arrival_velocity` (:538, vertical today);
  - the first `v_in` of `annotate_path` (:728).

  Unset, nothing changes: `ballistic_parity_test` stays byte-identical, and `ballistic_plan_check`'s
  cases are unchanged.
- **A stop anywhere, for Safe.** A search from a moving start whose goal is any cell of a mask (the
  real map's standable cells) where the landing can stop (the test at :507). It runs with h = 0 and a
  cap on hops. Safe0 is its first expansion, the start cell itself, so one call covers both cases.
  One or two expansions take milliseconds.
- **Cost-to-go.** `cost_to_go(map, goal, max_rise)`: the Dijkstra of §3 step 0, plus its downhill
  direction per cell. Pure and unit-testable (the maze wall: cells in front of it route through the
  opening).
- **Exits and the cost-to-go heuristic** in the momentum search: an optional cost-to-go grid that,
  when given, replaces the straight-line heuristic and lets a landing within `exit.band` of unseen
  cells end the search (a second terminal state beside today's stop at the goal, `kStop`). Unset:
  today's search, byte-identical.
- **`ReplanCore`**: steps 1–6 of §3 as pure C++. The node's `ActivePlan` (plus the splice fields),
  `recheckPlan`, `stopStance` and `cutIndex` move here, so the node and the offline harness (§4.5) run
  the same code.
- **If Whole is still slow after the speed work**, give it FASTER's horizon (its sphere S): also
  allow an exit at any landing ~6 m (4 hops) from A, scored the same way.

### 4.2 Planner node

- **The scheduler of §3.** It replaces the triggers "reached the frontier" and "the goal is measured
  now", with their DELETE, rate limit and hold wait. DELETE remains only for `hold`, and for a bad
  next landing that has no Safe branch.
- **Timing and gates.** Hop period, t_a and Δt bookkeeping; the gates; publishing with the splice
  fields; reverting on a rejection.
- **Plan end.** `active_` gets `terminal = goal | safe`, generalizing `cut`. `goal` mode on the last
  waypoint of a `safe` plan means the robot braked, not that it reached the goal.
- **Parameters:**
  - `replan.mode`: `stop` (today) or `faster`;
  - `faster.alpha` 1.25, `faster.margin` 0.1 s, `faster.lead` 0;
  - `safe.max_hops` 2;
  - `ctg.max_rise` 0.9 m, `exit.band` 0.5 m, `exit.heading_weight`.
- **Logging.** One `replans_*.csv` row per job:
  - k, map stamp, land_index, A, slack at publish [s], search time [s], H, R, Safe hops;
  - outcome: `published`, `late`, `stale`, `no whole`, `R≤A` or `cut on publish`.

  The snapshot PNGs also draw the cost-to-go route past H, dashed, to show where the plan was heading.

### 4.3 Messages

- `HopPlan`: `float64 base_path_id` (0: not a splice), `int32 splice_index`, `int32 safe_index`.
- v2, optional: `ApexDecision.next_arrival`, hopcopter's predicted arrival on `land_index` (`f.v_land`
  from its takeoff solve). With it, a splice at A = land_index starts from the robot's predicted real
  arrival instead of the planned one.

### 4.4 hopcopter

- **Splice adoption.** Pair the marker with its HopPlan by stamp; they come on two topics, so neither
  order is guaranteed.
  - `base_path_id` is the current path and `splice_index ≥ tracker.land` (the hop out of A is not
    decided yet): adopt it and *keep* the tracker's `land` and `retries`. There is no re-sync to the
    nearest waypoint.
  - The splice is late (`tracker.land > splice_index`): reject it and keep flying the current path.
  - Plain plans (`base_path_id` 0): as today.

  New `test_waypoint_tracker.py` cases cover all three.
- **Nothing else.** The Safe branch is an ordinary path end, with `goal` there. Log `on_safe`
  (land_index > safe_index) to count braking.

### 4.5 Offline closed-loop harness: `replan_loop_check`

No ROS, no Gazebo. It combines mapper_check's simulated Mid-360 and the `ElevationMapper` with
`ReplanCore` and an idealized hopper:
- every hop takes T = 0.88 s and lands exactly on the committed path, with a foot contact at each
  touchdown;
- the map is published every 0.5 s;
- each search takes its measured wall time, times a factor for the sim's CPU share.

Per world it reports hops in place from goal to arrival, Safe branches flown, late and stale jobs, and
time to goal. The same harness with `replan.mode:=stop` is the baseline. A Gazebo run takes minutes;
this takes seconds, and it is where the parameters get tuned. It needs `ballistic_core` visible to
`hopcopter_perception`, or a new package that depends on both.

## 5. Order of work

1. **Moving start** in the core, with `ballistic_plan_check` cases: from a waypoint of a pinned plan,
   with its planned arrival, the first hop's landing tilt holds and the plan passes the controller's
   guards. This is pure and independent of the speed work, but it touches `hopping_astar.cpp` next to
   the heuristic, so coordinate with that work.
2. **Cost-to-go, exits, Safe search, `ReplanCore`** (§3 steps 0–6), with unit cases on the LiDAR
   maps that `mapper_check --map-out` writes: the route, H, R and Safe at the cliff_gap rim, the maze
   wall and the stairs.
3. **`replan_loop_check`**, baseline first.
4. **Node scheduler and splice publishing; hopcopter's splice adoption.**
5. **Sim**: maze, cliff_gap, slope_30deg, lilypads, low_wall (there and back), stairs, compared with
   `replanning_on_the_move.md` §2.1–2.2.
6. **Docs**: CLAUDE.md (architecture §1, §3b), README, CHANGELOG, and this file's results.

## 6. Pass criteria

- `ballistic_parity_test` byte-identical; `ballistic_plan_check` all pass; hopping_robot's tests pass,
  plus the splice cases.
- Harness, with searches ≤ 0.3 s:
  - no hops in place between goal and arrival where the terrain ahead is seen at least 2 hops early;
  - at most one per occluder (behind the maze's wall);
  - no landing on a cell the real map had not passed when the plan went out.
- Sim:
  - hops in place from goal to arrival drop from 55–90 % (`replanning_on_the_move.md` §2.1) to under
    15 %;
  - no new falls;
  - no `hold` caused by a splice.
- The known-map mode is unchanged, and `replan.mode:=stop` restores today's behaviour.

## 7. Risks and open questions

- **Search time is the precondition.** At ~1 s per search, A sits 2–3 hops ahead. Behind the maze's
  wall, new terrain appears only 1–2 hops ahead, so R ≤ A and the robot brakes at F as it does today.
  The hopper's ways to widen the margin are B2 and B3 of `replanning_on_the_move.md`: shorter hops
  before an occluder, and higher hops to look over it. Both are FASTER's R-versus-A tradeoff seen from
  the other side.
- **Planned vs. real arrival at A.** A splice assumes the robot arrives at A as planned. hopcopter
  re-solves every takeoff from its real arrival, under the braking tilt guard, so small errors are
  absorbed. Watch `tilt_needed_deg` and retries after splices; if they grow, do v2 of §4.3. During a
  `retry`, splice no earlier than land_index + 1.
- **Heading at R.** Past the exit only the route's direction is modeled, not the robot's momentum.
  The next plan may have to turn the robot at R by more than one landing's 20° allows, and add a hop.
  With a plan every ~0.5 s that should be small; the harness measures it. If not, plan on a guessed
  terrain past the exit (the dropped option).
- **The cost-to-go is optimistic.** Unknown ground counts as free, so the route can lead into a gap
  or a dead end until it is seen (cliff_gap's gap). Only time is lost: the flown part is always on
  measured ground.
- **Churn.** Each job may send the hops past A somewhere new, most likely with a weighted A* on a noisy
  map. If the logs show flip-flopping, add the churn gate (§3 step 6).
- **CPU.** One job runs at a time: at 2 Hz and 0.1 s per job, ~20 % of a core, next to Gazebo and
  FAST-LIO.
- **The apex race.** A splice that reaches hopcopter just after the apex into A is late. The margin
  makes that rare; hopcopter's rejection and the planner's revert make it harmless.
