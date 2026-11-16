// ROS 2 node wrapping the ballistic hopping A* core.
//
// Subscribes:  /elevation_map  (grid_map_msgs/GridMap, transient_local)
//              /cf_1/pose      (geometry_msgs/PoseStamped, live start)
//              /goal_pose      (geometry_msgs/PoseStamped, one-shot trigger)
// Publishes:   /ballistic_hop_plan         (hopcopter_msgs/HopPlan: each hop's
//                takeoff and the stance into it; just before the marker pair,
//                with the same stamp)
//              /ballistic_trajectory       (visualization_msgs/Marker,
//                DELETE then ADD SPHERE_LIST, id=400 — the hopcopter contract;
//                point z = target apex of the hop landing there, i.e. apex height
//                above that hop's takeoff ground; output.hover_offset for the start)
//              /trajectory_start_position  (geometry_msgs/PoseStamped)
// Subscribes too: /hopcopter/apex_decision (hopcopter_msgs/ApexDecision: which
//              waypoint the robot aims at, and when it hops in place).
// On a LiDAR map (/elevation_map with an upper_bound layer, from
// hopcopter_perception) unseen cells carry their bound: the arcs may fly over
// them above it, but no hop lands on them, and the planner refuses to plan from
// an unseen start. A goal the LiDAR has not seen yet gets a frontier plan to the
// reachable cell nearest it. On every map update the remaining hops of the plan
// are re-checked (landing still standable, its ground moved < 5 cm, every arc
// still clears with its planned takeoff). When one fails past the hop the robot
// is committed to, the plan is cut before it, at the latest waypoint the robot
// can stop on; the robot flies the good hops and hops in place there, a search
// from there runs on a worker thread meanwhile, and its plan goes out once the
// robot hops in place on that waypoint (replan.cut, docs/replan_cut_plan.md).
// Otherwise (a bad next landing; a frontier plan can now do better; the robot
// gave up on the path, hold) the planner publishes a DELETE alone, waits for the
// robot to hop in place, and plans again from there, at most once per
// replan.min_interval. On a hand-made map (no upper_bound layer) nothing
// changes: it plans once per /goal_pose.
// CSVs (path + per-hop energy chain) go to data/data_ballistic_planner/.
// A JSON snapshot (map layers + path + hop radii) goes to vis_planned_path/,
// and scripts/visualize_planned_path.py renders it to a PNG in the background.
// Replans are logged, with their reason, to replans_<stamp>.csv there.

#include <ament_index_cpp/get_package_prefix.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <hopcopter_msgs/msg/hop_plan.hpp>
#include <hopcopter_msgs/msg/apex_decision.hpp>

#include "ballistic_motion_planner/ballistic_physics.hpp"
#include "ballistic_motion_planner/frontier.hpp"
#include "ballistic_motion_planner/hopping_astar.hpp"
#include "ballistic_motion_planner/map2d5.hpp"

namespace
{
constexpr int kPathMarkerId = 400;  // hopcopter accepts only this id
constexpr double kDeg = M_PI / 180.0;

geometry_msgs::msg::Vector3 toVector3(const ballistic::Vec3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}
}

class BallisticPlannerNode : public rclcpp::Node
{
public:
  BallisticPlannerNode()
  : Node("ballistic_planner")
  {
    // Physics / search parameters. Defaults are the robot this repo flies
    // (ballistic::robot_params(), stage 3 of docs/follow_planner_plan.md);
    // v_g_max, v_max and e_inject_max are derived from the others.
    const auto d = ballistic::robot_params();
    p_.momentum_aware = declare_parameter("search.momentum_aware", d.momentum_aware);
    p_.g = declare_parameter("physics.g", d.g);
    p_.mass = declare_parameter("physics.mass", d.mass);
    p_.eta = declare_parameter("physics.eta", d.eta);
    p_.h_initial = declare_parameter("physics.h_initial", d.h_initial);
    p_.min_apex = declare_parameter("physics.min_apex", d.min_apex);
    // the same budget per kg at any mass
    p_.e_inject_max = declare_parameter("physics.e_inject_max", d.e_inject_max / d.mass * p_.mass);
    p_.mu = declare_parameter("physics.mu", d.mu);
    p_.V_g_max = declare_parameter("physics.v_g_max", std::sqrt(2.0 * p_.g * 2.0));
    p_.V_max = declare_parameter(
      "physics.v_max",
      std::sqrt(p_.eta * p_.V_g_max * p_.V_g_max + 2.0 * p_.e_inject_max / p_.mass));
    p_.speed_bin = declare_parameter("search.speed_bin", d.speed_bin);
    p_.w_energy = declare_parameter("search.w_energy", d.w_energy);
    p_.hop_scan_step = declare_parameter("search.hop_scan_step", d.hop_scan_step);
    p_.hop_scan_step_ref_radius = declare_parameter(
      "search.hop_scan_step_ref_radius", d.hop_scan_step_ref_radius);
    p_.min_hop_radius = declare_parameter("search.min_hop_radius", d.min_hop_radius);
    p_.bank_bins = static_cast<int>(
      declare_parameter("search.bank_bins", static_cast<int64_t>(d.bank_bins)));
    p_.robot_radius = declare_parameter("robot.radius", d.robot_radius);
    p_.leg_length = declare_parameter("robot.leg_length", d.leg_length);
    p_.bottom_clearance = declare_parameter("robot.bottom_clearance", d.bottom_clearance);
    p_.side_clearance = declare_parameter("robot.side_clearance", d.side_clearance);
    p_.steep_grade = declare_parameter("clearance.steep_grade", d.steep_grade);
    p_.arc_max_step = declare_parameter("clearance.arc_max_step", d.arc_max_step);
    p_.clearance_active_tol = declare_parameter("clearance.active_tolerance", d.clearance_active_tol);
    p_.clearance_start_exempt = declare_parameter("clearance.start_exempt", d.clearance_start_exempt);
    // momentum_aware only: the stance model (hopcopter.py's k2), the landing tilt
    // cap, the leg-angle floor and the push stage 2 measured
    p_.k2 = declare_parameter("stance.k2", d.k2);
    p_.tau_max = declare_parameter("stance.max_tilt_deg", d.tau_max / kDeg) * kDeg;
    p_.alpha_leg_min = declare_parameter("stance.min_leg_angle_deg", d.alpha_leg_min / kDeg) * kDeg;
    p_.push.accel = declare_parameter("push.accel", d.push.accel);
    p_.push.g_coast = declare_parameter("push.g_coast", d.push.g_coast);
    p_.push.t_offset = declare_parameter("push.time_offset", d.push.t_offset);
    p_.push.t_max = declare_parameter("push.max_time", d.push.t_max);
    // Target apex for the start point, which no planned hop lands on.
    hover_offset_ = declare_parameter("output.hover_offset", 0.8);
    // Render each plan's snapshot to a PNG (vis_planned_path/) in the background.
    render_png_ = declare_parameter("output.render_png", true);
    // LiDAR maps: frontier plans and hold-and-replan (no effect on a hand-made map)
    frontier_.hop_radius = declare_parameter("frontier.hop_radius", frontier_.hop_radius);
    frontier_.max_rise = declare_parameter("frontier.max_rise", frontier_.max_rise);
    frontier_attempts_ = static_cast<int>(declare_parameter("frontier.max_attempts", int64_t{2}));
    // On a LiDAR map, land this much further from edges that drop at least
    // standable.lidar_edge_min_drop: a rim seen from above can read a cell wider than it is
    // (no ray sees past it), and plans landed on the maze's 0.3 m wall and 0.15 m from
    // cliff_gap's 1.8 m rim, and the robot fell at both (2026-11-07). Stair risers (0.18 m)
    // keep none: their treads' middle cells stay landable (docs/lidar_map_quality_plan.md).
    lidar_edge_margin_ = declare_parameter("standable.lidar_edge_margin", 0.1);
    lidar_edge_min_drop_ = declare_parameter("standable.lidar_edge_min_drop", 0.3);
    // and this far from unseen cells, which can hide a drop: the body radius plus a typical
    // landing miss (a landing 0.15 m short on a wall's top, next to its unseen shadow, went
    // over the wall's front edge, 2026-11-07)
    unseen_margin_ = declare_parameter("standable.unseen_margin", 0.3);
    // An edge is steeper than steep_grade (60 deg, 0.173 m per cell) less this much: 0.02 makes
    // it a jump over 0.153 m per cell. It was 0.08 while the mapper spread a step over two cells
    // (2026-11-07); the face-point filter ended that (docs/lidar_map_quality_plan.md, Fix 2).
    // At 0 the stairs' 0.18 m risers sat 7 mm over the threshold, and the mapper's per-cell
    // spread (±10 mm on a riser's height, 2026-11-11 stairs run) left 4 in 10 of the near ones
    // no edge. The slope's steepest measured jump is 0.107 m. hopcopter's clearance re-check
    // (lidar_steep_noise) must match.
    lidar_steep_noise_ = declare_parameter("standable.lidar_steep_noise", 0.02);
    replan_enabled_ = declare_parameter("replan.enabled", true);
    replan_interval_ = declare_parameter("replan.min_interval", 5.0);       // [s]
    replan_height_tol_ = declare_parameter("replan.height_tolerance", 0.05);  // [m]
    replan_hold_wait_ = declare_parameter("replan.hold_wait", 1.0);         // [s] after the DELETE
    replan_hold_timeout_ = declare_parameter("replan.hold_timeout", 4.0);   // [s]
    // A re-check that fails past the next landing cuts the plan before the bad hop, and the
    // search from there runs in the background while the robot flies the good hops; false:
    // always DELETE and hold (docs/replan_cut_plan.md).
    replan_cut_ = declare_parameter("replan.cut", true);
    const auto map_topic = declare_parameter("topics.map", std::string("/elevation_map"));
    const auto traj_topic = declare_parameter("topics.trajectory", std::string("/ballistic_trajectory"));
    const auto plan_topic = declare_parameter("topics.hop_plan", std::string("/ballistic_hop_plan"));

    // Each check guards a silent plan-returns-nothing failure.
    ballistic::validate_params(p_);

    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(traj_topic, 10);
    plan_pub_ = create_publisher<hopcopter_msgs::msg::HopPlan>(plan_topic, 10);
    start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/trajectory_start_position", 10);

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic, rclcpp::QoS(1).transient_local(),
      [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
        latest_map_msg_ = msg;
        map_changed_ = true;
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf_1/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        robot_x_ = msg->pose.position.x;
        robot_y_ = msg->pose.position.y;
        have_pose_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goalCallback(*msg);
      });
    decision_sub_ = create_subscription<hopcopter_msgs::msg::ApexDecision>(
      "/hopcopter/apex_decision", 20,
      [this](const hopcopter_msgs::msg::ApexDecision::SharedPtr msg) {
        decision_ = *msg;
        decision_time_ = std::chrono::steady_clock::now();
        have_decision_ = true;
      });
    replan_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {replanStep();});

    csv_dir_ = "/home/zweminhtetaung/hopcopter-bmp/data/data_ballistic_planner";
    std::error_code ec;
    std::filesystem::create_directories(csv_dir_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Could not create CSV dir %s: %s",
        csv_dir_.c_str(), ec.message().c_str());
    }

    planning_csv_dir_ = csv_dir_ + "/planning_tracking";
    std::filesystem::create_directories(planning_csv_dir_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Could not create CSV dir %s: %s",
        planning_csv_dir_.c_str(), ec.message().c_str());
    }

    vis_dir_ = csv_dir_ + "/vis_planned_path";
    std::filesystem::create_directories(vis_dir_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Could not create vis dir %s: %s",
        vis_dir_.c_str(), ec.message().c_str());
    }

    RCLCPP_INFO(get_logger(),
      "Ballistic planner ready (%s search). Waiting for %s and /cf_1/pose; "
      "plans once per /goal_pose.",
      p_.momentum_aware ? "momentum-aware" : "reference", map_topic.c_str());
  }

private:
  enum class ReplanState {kIdle, kWaitHold};
  // The plan hopcopter flies, as the replan check compares it with each new map.
  struct ActivePlan
  {
    bool valid = false;
    bool frontier = false;
    bool cut = false;    // cut before a bad hop: its last waypoint is a stop, not the goal
    double stamp = 0.0;  // the marker's stamp [s]: hopcopter's path_id
    double sx = 0.0, sy = 0.0, tx = 0.0, ty = 0.0;  // where it was planned from, and to
    std::vector<std::pair<double, double>> path;
    std::vector<ballistic::Hop> hops;
    // [0] the start into hops[0], [i] the landing on path[i] into hops[i], last: the stop
    std::vector<ballistic::Stance> stances;
    std::vector<double> ground;  // the map's height at each waypoint when planned
  };

  void goalCallback(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!latest_map_msg_) {
      RCLCPP_WARN(get_logger(), "Goal received but no elevation map yet — ignoring. "
        "Re-publish the goal once the map is up.");
      return;
    }
    if (!have_pose_) {
      RCLCPP_WARN(get_logger(), "Goal received but no /cf_1/pose yet — ignoring. "
        "Re-publish the goal once the pose is up.");
      return;
    }
    goal_x_ = msg.pose.position.x;
    goal_y_ = msg.pose.position.y;
    have_goal_ = true;
    goal_reached_ = false;
    failed_plans_ = 0;
    replan_state_ = ReplanState::kIdle;
    dropCutSearch();
    planAndPublish(robot_x_, robot_y_, "goal");
  }

  // One search and what it found. map is declared before planner, which refers to it.
  struct SearchResult
  {
    bool searched = false;  // false: refused before searching (off the map, unseen start, ...)
    bool found = false;
    std::string why;
    double sx = 0.0, sy = 0.0, tx = 0.0, ty = 0.0;
    bool frontier = false;
    uint64_t gen = 0;  // the cut search's generation (cut_gen_ when it started)
    std::unique_ptr<ballistic::Map2D5> map;
    std::unique_ptr<ballistic::HoppingAStarPlanner> planner;
    std::vector<std::pair<double, double>> path;
  };

  // Plan from (sx, sy) to (gx, gy) on msg. On a LiDAR map whose goal is not standable yet,
  // plan to the reachable cell nearest it instead (a frontier plan). Publishes nothing and
  // touches no member but the logger, so it can run on a worker thread (the cut search).
  std::unique_ptr<SearchResult> search(
    const grid_map_msgs::msg::GridMap & msg, double sx, double sy, double gx, double gy,
    const std::string & why)
  {
    auto r = std::make_unique<SearchResult>();
    r->why = why;
    r->sx = sx;
    r->sy = sy;
    try {
      const auto t0 = std::chrono::steady_clock::now();
      auto map = convertMap(msg);
      if (!map->is_within_bounds(sx, sy)) {
        RCLCPP_ERROR(get_logger(),
          "Robot position (%.2f, %.2f) is outside the map "
          "[%.2f, %.2f] x [%.2f, %.2f] — refusing to plan.",
          sx, sy, map->origin_x(), map->origin_x() + map->size_x(),
          map->origin_y(), map->origin_y() + map->size_y());
        return r;
      }
      if (!map->is_within_bounds(gx, gy)) {
        RCLCPP_ERROR(get_logger(), "Goal (%.2f, %.2f) is outside the map — refusing to plan.", gx, gy);
        return r;
      }
      // Start guard: an unseen start would carry its upper bound (or OBSTACLE)
      // as its height, and every hop would be planned from it.
      if (!ballistic::start_measured(*map, sx, sy)) {
        RCLCPP_ERROR(get_logger(),
          "The start cell (%.2f, %.2f) has no measured height yet — refusing to plan. "
          "(The mapper writes the start disk at the first touchdown.)", sx, sy);
        return r;
      }
      const auto standable = map->standable_mask(p_.robot_radius, p_.side_clearance, p_.steep_grade);
      const auto [rg, cg] = map->world_to_grid(gx, gy);
      const bool goal_standable = standable[static_cast<size_t>(rg) * map->cols() + cg];
      const bool lidar_map = !map->landable_mask().empty();

      std::vector<std::pair<double, double>> targets;
      if (goal_standable || !lidar_map) {
        targets.emplace_back(gx, gy);
      }
      if (lidar_map) {
        // Frontier candidates, nearest the goal first, kept apart by 0.5 m so a
        // failed search is not repeated next door.
        for (const auto & [r_, c_] : ballistic::frontier_candidates(
            *map, standable, {sx, sy}, {gx, gy}, frontier_))
        {
          if (static_cast<int>(targets.size()) >= (goal_standable ? 1 : 0) + frontier_attempts_) {
            break;
          }
          const auto xy = map->grid_to_world(r_, c_);
          bool apart = true;
          for (const auto & t : targets) {
            apart = apart && std::hypot(t.first - xy.first, t.second - xy.second) >= 0.5;
          }
          if (apart && !(r_ == rg && c_ == cg)) {
            targets.push_back(xy);
          }
        }
      }
      if (targets.empty()) {
        RCLCPP_ERROR(get_logger(),
          "(%s) The goal (%.2f, %.2f) is not on measured, standable ground, and no measured cell "
          "the robot can reach is nearer it: waiting for the map.", why.c_str(), gx, gy);
        return r;
      }

      r->searched = true;
      const auto t1 = std::chrono::steady_clock::now();
      for (size_t k = 0; k < targets.size(); ++k) {
        const auto [tx, ty] = targets[k];
        const bool frontier = !(tx == gx && ty == gy);
        auto planner = std::make_unique<ballistic::HoppingAStarPlanner>(
          *map, std::make_pair(sx, sy), std::make_pair(tx, ty), p_);
        const auto ts = std::chrono::steady_clock::now();
        auto path = planner->plan();
        const auto te = std::chrono::steady_clock::now();
        const double prep_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double plan_ms = std::chrono::duration<double, std::milli>(te - ts).count();
        if (!path) {
          RCLCPP_ERROR(get_logger(),
            "(%s) No path found from (%.2f, %.2f) to %s (%.2f, %.2f)%s "
            "[prep %.1f ms, search %.1f ms, %ld expansions].",
            why.c_str(), sx, sy, frontier ? "the frontier cell" : "the goal", tx, ty,
            planner->recheck_failed() ?
            ": the search's path exceeds the landing tilt cap with its exact arrival directions" : "",
            prep_ms, plan_ms, planner->n_expansions());
          continue;
        }
        RCLCPP_INFO(get_logger(),
          "[TIMING] (%s) %s with %zu waypoints: map+fields %.1f ms, A* %.1f ms "
          "(%ld expansions, %ld edge checks, %ld accepted; %ld hops over the tilt cap, "
          "%ld over one push)", why.c_str(), frontier ? "Frontier path" : "Path", path->size(),
          prep_ms, plan_ms, planner->n_expansions(), planner->n_edge_checks(),
          planner->n_edges_accepted(), planner->n_tilt_rejects(), planner->n_push_rejects());
        if (frontier) {
          RCLCPP_WARN(get_logger(),
            "Goal (%.2f, %.2f) not on measured ground yet: frontier plan to (%.2f, %.2f), %.2f m "
            "from it; replanning when the goal is measured or the robot gets there.",
            gx, gy, tx, ty, std::hypot(tx - gx, ty - gy));
        }
        logHopTable(*planner);
        r->found = true;
        r->tx = tx;
        r->ty = ty;
        r->frontier = frontier;
        r->path = std::move(*path);
        r->map = std::move(map);
        r->planner = std::move(planner);
        return r;
      }
      r->tx = gx;
      r->ty = gy;
      return r;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error in search: %s", e.what());
      r->searched = false;
      return r;
    }
  }

  // The plan a search found, as the replan check and the publishers see it (no stamp yet).
  static ActivePlan planOf(const SearchResult & r)
  {
    ActivePlan plan;
    plan.valid = r.path.size() >= 2;
    plan.frontier = r.frontier;
    plan.sx = r.sx;
    plan.sy = r.sy;
    plan.tx = r.tx;
    plan.ty = r.ty;
    plan.path = r.path;
    plan.hops = r.planner->path_hops();
    plan.stances = r.planner->path_stances();
    for (const auto & [x, y] : r.path) {
      const auto [row, col] = r.map->world_to_grid(x, y);
      plan.ground.push_back(r.map->at(row, col));
    }
    return plan;
  }

  // HopPlan first, then the marker pair, all with one stamp; plan becomes the active plan.
  void publishPlan(ActivePlan plan)
  {
    if (plan.path.size() < 2) {
      RCLCPP_WARN(get_logger(),
        "Path has %zu point(s); hopcopter ignores <2-point trajectories — not publishing.",
        plan.path.size());
      return;
    }
    const auto now_stamp = now();
    plan.stamp = now_stamp.seconds();
    publishHopPlan(plan, now_stamp);
    publishTrajectory(plan.path, plan.hops, now_stamp);
    active_ = std::move(plan);
  }

  // Publish a search's plan, write its CSVs and snapshot, and make it the active plan.
  void publishResult(const SearchResult & r)
  {
    publishPlan(planOf(r));
    publishStartPosition(r.sx, r.sy, active_.ground.front());
    recordResult(r);
    logReplan(r.why, r.frontier ? "frontier plan" : "plan", r.sx, r.sy, r.tx, r.ty, r.path.size());
    failed_plans_ = 0;
  }

  // The CSVs and the snapshot (+ PNG) of a search's plan.
  void recordResult(const SearchResult & r)
  {
    const auto & planner = *r.planner;
    const auto stamp = makeStamp();
    writeCsvs(*r.map, r.path, planner.path_hops(), planner.path_stances(), stamp);
    const auto snapshot = writeVisSnapshot(
      *r.map, r.path, planner.path_hops(), planner.path_stances(), planner.v_g_initial(),
      r.sx, r.sy, r.tx, r.ty, stamp, r.frontier);
    if (render_png_ && !snapshot.empty()) {
      renderSnapshot(snapshot);
    }
  }

  // Plan from (sx, sy) to the goal on the latest map and publish it, here and now. Returns
  // whether a plan went out.
  bool planAndPublish(double sx, double sy, const std::string & why)
  {
    last_plan_time_ = std::chrono::steady_clock::now();
    const auto r = search(*latest_map_msg_, sx, sy, goal_x_, goal_y_, why);
    if (r->found) {
      publishResult(*r);
      return true;
    }
    if (r->searched) {
      logReplan(why, "no path", sx, sy, goal_x_, goal_y_, 0);
      ++failed_plans_;
    }
    return false;
  }

  // A failed re-check: the first bad waypoint (see recheckPlan) and why.
  struct Recheck
  {
    int bad = -1;  // -1: the plan holds
    std::string reason;
  };

  // The hops of plan from waypoint `first` on, against map. The first bad waypoint is the
  // landing that fails, the takeoff whose ground moved, or the landing of an arc that no
  // longer clears (that arc must not be flown).
  Recheck recheckPlan(const ballistic::Map2D5 & map, const ActivePlan & plan, int first) const
  {
    const auto standable = map.standable_mask(p_.robot_radius, p_.side_clearance, p_.steep_grade);
    const auto inflated = map.inflated_field(p_.robot_radius + p_.side_clearance, p_.steep_grade);
    const auto & path = plan.path;
    char buf[200];
    for (size_t j = std::max(1, first); j < path.size(); ++j) {
      const auto [xs, ys] = path[j - 1];
      const auto [xg, yg] = path[j];
      const auto [rg, cg] = map.world_to_grid(xg, yg);
      const auto [rs, cs] = map.world_to_grid(xs, ys);
      const int jj = static_cast<int>(j);
      if (!standable[static_cast<size_t>(rg) * map.cols() + cg]) {
        std::snprintf(buf, sizeof(buf), "waypoint %zu (%.2f, %.2f) is no longer standable", j, xg, yg);
        return {jj, buf};
      }
      const double tg = map.at(rg, cg), ts = map.at(rs, cs);
      if (std::abs(tg - plan.ground[j]) > replan_height_tol_) {
        std::snprintf(buf, sizeof(buf), "waypoint %zu's ground moved %+.3f m", j, tg - plan.ground[j]);
        return {jj, buf};
      }
      if (j >= 2 && std::abs(ts - plan.ground[j - 1]) > replan_height_tol_) {
        std::snprintf(buf, sizeof(buf), "waypoint %zu's ground moved %+.3f m", j - 1,
          ts - plan.ground[j - 1]);
        return {jj - 1, buf};
      }
      // The planned arc against the new terrain: a hop planned at its clearance
      // floor fails by a hair on any change, so only a real deficit counts
      // (hopcopter's own re-check raises the apex for the small ones).
      const double exempt = inflated[static_cast<size_t>(rs) * map.cols() + cs] > ts + 1e-9 ?
        p_.clearance_start_exempt : 0.0;
      const auto sampled = ballistic::arc_samples(
        xs, ys, ts, xg, yg, tg, map, inflated, p_.arc_max_step, p_.clearance_active_tol, exempt);
      if (!sampled) {
        std::snprintf(buf, sizeof(buf), "the arc into waypoint %zu leaves the map", j);
        return {jj, buf};
      }
      const double tan_a = std::tan(plan.hops[j - 1].alpha_s);
      const double X = sampled->X;
      const double k = (X * tan_a - (tg - ts)) / (X * X);
      double worst = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < sampled->u.size(); ++i) {
        if (sampled->active[i]) {
          const double u = sampled->u[i];
          worst = std::min(worst, ts + u * tan_a - k * u * u - sampled->field[i]);
        }
      }
      if (worst < p_.bottom_clearance - replan_height_tol_) {
        std::snprintf(buf, sizeof(buf),
          "the arc into waypoint %zu clears the terrain by %.2f m now (needs %.2f)",
          j, worst, p_.bottom_clearance);
        return {jj, buf};
      }
    }
    return {};
  }

  // The stance that stops a landing from hop h: a vertical takeoff, as the search's stop at
  // the goal (hopping_astar.cpp, annotate_path).
  ballistic::Stance stopStance(const ballistic::Hop & h) const
  {
    const double v_z = std::sqrt(std::max(0.0, h.v_g * h.v_g - h.v_h * h.v_h));
    return ballistic::stance(
      {h.v_h * std::cos(h.heading), h.v_h * std::sin(h.heading), -v_z}, {0.0, 0.0, 1.0}, p_.k2);
  }

  // Where to cut a plan whose waypoint `bad` failed: the latest waypoint before it that the
  // robot can stop on (landing tilt within tau_max for a vertical takeoff) and is not
  // committed past (its takeoff toward waypoint `committed` was fixed at the apex that chose
  // it). -1: none.
  int cutIndex(const ActivePlan & plan, int bad, int committed) const
  {
    for (int k = bad - 1; k >= std::max(1, committed); --k) {
      if (stopStance(plan.hops[static_cast<size_t>(k) - 1]).tilt <= p_.tau_max + 1e-9) {
        return k;
      }
    }
    return -1;
  }

  // plan up to waypoint k, ending in a stop there.
  ActivePlan cutPlan(const ActivePlan & plan, int k) const
  {
    const auto n = static_cast<size_t>(k);
    ActivePlan cut;
    cut.valid = true;
    cut.frontier = plan.frontier;
    cut.cut = true;
    cut.sx = plan.sx;
    cut.sy = plan.sy;
    cut.tx = plan.path[n].first;
    cut.ty = plan.path[n].second;
    cut.path.assign(plan.path.begin(), plan.path.begin() + n + 1);
    cut.hops.assign(plan.hops.begin(), plan.hops.begin() + n);
    cut.stances.assign(plan.stances.begin(), plan.stances.begin() + n);
    cut.stances.push_back(stopStance(plan.hops[n - 1]));
    cut.ground.assign(plan.ground.begin(), plan.ground.begin() + n + 1);
    return cut;
  }

  // Publish plan cut at waypoint k, and search on from there in the background.
  void publishCut(const ActivePlan & plan, int k, const std::string & reason)
  {
    const auto cut = cutPlan(plan, k);
    const auto [cx, cy] = cut.path.back();
    publishPlan(cut);  // plan may be active_, which this replaces
    RCLCPP_WARN(get_logger(),
      "Replanning (%s): path cut at waypoint %d (%.2f, %.2f), landing tilt %.1f deg for the stop; "
      "searching from there", reason.c_str(), k, cx, cy, cut.stances.back().tilt / kDeg);
    logReplan(reason, "cut at " + std::to_string(k), cut.sx, cut.sy, cx, cy, cut.path.size());
    // A search still running started elsewhere: its result is stale.
    ++cut_gen_;
    cut_ready_.reset();
    cut_reason_ = reason;
    cut_from_ = {cx, cy};
    cut_needed_ = true;
    cut_next_start_ = std::chrono::steady_clock::now();
    startCutSearch();
  }

  // Forget the cut search (a new goal, or a DELETE): its result will be discarded.
  void dropCutSearch()
  {
    ++cut_gen_;
    cut_ready_.reset();
    cut_needed_ = false;
  }

  // Start the search from the cut waypoint on a worker thread, one at a time.
  void startCutSearch()
  {
    if (!cut_needed_ || cut_future_.valid() || std::chrono::steady_clock::now() < cut_next_start_) {
      return;
    }
    cut_needed_ = false;
    last_plan_time_ = std::chrono::steady_clock::now();
    const auto msg = latest_map_msg_;
    const double sx = cut_from_.first, sy = cut_from_.second;
    const double gx = goal_x_, gy = goal_y_;
    const auto why = "replan: " + cut_reason_;
    const auto gen = cut_gen_;
    cut_future_ = std::async(std::launch::async, [this, msg, sx, sy, gx, gy, why, gen]() {
        auto r = search(*msg, sx, sy, gx, gy, why);
        r->gen = gen;
        return r;
      });
  }

  // Collect a finished cut search. A stale one is discarded; a failed one is retried later.
  void pollCutSearch()
  {
    if (!cut_future_.valid() ||
      cut_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      return;
    }
    auto r = cut_future_.get();
    if (r->gen != cut_gen_) {
      RCLCPP_INFO(get_logger(), "(%s) Discarding a stale search from (%.2f, %.2f)",
        r->why.c_str(), r->sx, r->sy);
      logReplan(r->why, "discarded (stale)", r->sx, r->sy, r->tx, r->ty, r->path.size());
      return;
    }
    if (!r->found) {
      if (r->searched) {
        logReplan(r->why, "no path", r->sx, r->sy, goal_x_, goal_y_, 0);
      }
      ++failed_plans_;
      // Try again on a newer map; the robot hops in place on the cut waypoint meanwhile.
      const double wait = replan_interval_ * std::min(6, 1 << std::min(failed_plans_, 3));
      cut_next_start_ = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(wait * 1000.0));
      cut_needed_ = true;
      RCLCPP_WARN(get_logger(), "(%s) No plan from the cut waypoint; searching again in %.0f s",
        r->why.c_str(), wait);
      return;
    }
    cut_ready_ = std::move(r);
  }

  // The robot hops in place on the cut waypoint and the search from there is done: re-check
  // its plan on the latest map, then publish it, or its own cut, or search again.
  void releaseCutResult()
  {
    const auto r = std::move(cut_ready_);
    auto plan = planOf(*r);
    const auto map = convertMap(*latest_map_msg_);
    const auto rc = recheckPlan(*map, plan, 1);
    if (rc.bad < 0) {
      RCLCPP_INFO(get_logger(), "(%s) The robot is on the cut waypoint: publishing the rest",
        r->why.c_str());
      publishResult(*r);
      return;
    }
    // The robot is on the plan's start, so its first hop is not committed yet.
    const int k = replan_cut_ ? cutIndex(plan, rc.bad, 1) : -1;
    recordResult(*r);
    if (k >= 1) {
      publishStartPosition(r->sx, r->sy, plan.ground.front());
      logReplan(r->why, "plan", r->sx, r->sy, r->tx, r->ty, r->path.size());
      failed_plans_ = 0;
      publishCut(plan, k, rc.reason);
      return;
    }
    RCLCPP_WARN(get_logger(), "(%s) The new plan failed its re-check (%s): searching again",
      r->why.c_str(), rc.reason.c_str());
    logReplan(r->why, "discarded: " + rc.reason, r->sx, r->sy, r->tx, r->ty, r->path.size());
    cut_reason_ = rc.reason;
    cut_needed_ = true;
    startCutSearch();
  }

  // Replanning on LiDAR maps. Runs at 10 Hz on the wall clock.
  void replanStep()
  {
    if (!replan_enabled_ || !have_goal_ || goal_reached_ || !latest_map_msg_) {
      return;
    }
    pollCutSearch();
    const auto now_t = std::chrono::steady_clock::now();
    const double since_plan = std::chrono::duration<double>(now_t - last_plan_time_).count();
    if (replan_state_ == ReplanState::kWaitHold) {
      const double since_delete = std::chrono::duration<double>(now_t - delete_time_).count();
      const bool holding = have_decision_ && decision_time_ > delete_time_ &&
        (decision_.mode == "idle" || decision_.mode == "hold");
      if ((holding && since_delete >= replan_hold_wait_) || since_delete >= replan_hold_timeout_) {
        replan_state_ = ReplanState::kIdle;
        double sx = robot_x_, sy = robot_y_;
        if (holding) {
          sx = decision_.aim.x;
          sy = decision_.aim.y;
        } else {
          RCLCPP_WARN(get_logger(), "No in-place hop reported %.1f s after the DELETE: replanning "
            "from /cf_1/pose", since_delete);
        }
        if (!planAndPublish(sx, sy, "replan: " + replan_reason_) && failed_plans_ == 0) {
          ++failed_plans_;  // refused before searching (off the map, unseen start, ...)
        }
      }
      return;
    }
    const bool on_active = active_.valid && have_decision_ &&
      std::abs(decision_.path_id - active_.stamp) < 1e-4;
    // A cut plan: once the robot hops in place on its last waypoint, the rest goes out.
    if (on_active && active_.cut && decision_.mode == "goal") {
      if (cut_ready_) {
        releaseCutResult();
        return;
      }
      if (!cut_needed_ && !cut_future_.valid()) {
        cut_needed_ = true;  // nothing on the way (should not happen): search again
      }
    }
    startCutSearch();
    if (!map_changed_ && active_.valid) {
      return;
    }
    // Only a LiDAR map grows; a hand-made map's plan is final.
    if (std::find(latest_map_msg_->layers.begin(), latest_map_msg_->layers.end(), "upper_bound") ==
      latest_map_msg_->layers.end())
    {
      map_changed_ = false;
      return;
    }
    std::string reason;
    if (on_active) {
      const auto map = convertMap(*latest_map_msg_);
      if (decision_.mode == "goal") {
        if (active_.cut) {
          map_changed_ = false;
          return;  // waiting on the cut waypoint for the rest (above)
        }
        if (active_.frontier) {
          reason = "reached the frontier";
        } else {
          goal_reached_ = true;
          RCLCPP_INFO(get_logger(), "Goal (%.2f, %.2f) reached", goal_x_, goal_y_);
          return;
        }
      } else if (decision_.mode == "hold") {
        reason = "the robot gave up on the path (hold)";
      } else if (decision_.mode == "track" || decision_.mode == "retry") {
        const auto rc = recheckPlan(*map, active_, decision_.land_index);
        reason = rc.reason;
        // Keep the good hops: cut before the bad one, at once, and search on from there.
        const int k = rc.bad < 0 || !replan_cut_ ? -1 : cutIndex(active_, rc.bad, decision_.land_index);
        if (k >= 1) {
          map_changed_ = false;
          pending_reason_.clear();
          publishCut(active_, k, reason);
          return;
        }
      }
      if (reason.empty() && active_.frontier && !active_.cut) {
        const auto standable = map->standable_mask(p_.robot_radius, p_.side_clearance, p_.steep_grade);
        const auto [rg, cg] = map->world_to_grid(goal_x_, goal_y_);
        if (standable[static_cast<size_t>(rg) * map->cols() + cg]) {
          reason = "the goal is measured now";
        }
      }
    } else if (!active_.valid) {
      reason = "no plan yet";  // the last attempt failed: try again on the newer map
    }
    map_changed_ = false;
    // After failed attempts, wait longer each time (up to 6x), so a robot off the
    // map or a goal nothing reaches does not replan every few seconds.
    const double interval = replan_interval_ * std::min(6, 1 << std::min(failed_plans_, 3));
    if (reason.empty() || since_plan < interval) {
      if (!reason.empty() && reason != pending_reason_) {
        RCLCPP_INFO(get_logger(), "Replan pending (%s), %.1f s after the last plan",
          reason.c_str(), since_plan);
        pending_reason_ = reason;
      }
      return;
    }
    pending_reason_.clear();
    replan_reason_ = reason;
    dropCutSearch();
    // DELETE alone: hopcopter drops the path 0.5 s later and hops in place.
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.header.stamp = now();
    del.id = kPathMarkerId;
    del.action = visualization_msgs::msg::Marker::DELETE;
    traj_pub_->publish(del);
    active_.valid = false;
    delete_time_ = now_t;
    replan_state_ = ReplanState::kWaitHold;
    RCLCPP_WARN(get_logger(), "Replanning (%s): path deleted, waiting for the robot to hop in place",
      reason.c_str());
  }

  void logReplan(const std::string & why, const std::string & outcome, double sx, double sy,
    double tx, double ty, size_t n)
  {
    if (!replan_csv_.is_open()) {
      replan_csv_.open(vis_dir_ + "/../replans_" + makeStamp() + ".csv");
      replan_csv_ << "wall_time,reason,outcome,start_x,start_y,target_x,target_y,goal_x,goal_y,waypoints\n";
    }
    replan_csv_ << std::fixed << std::setprecision(3)
                << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
                << ",\"" << why << "\"," << outcome << "," << sx << "," << sy << "," << tx << ","
                << ty << "," << goal_x_ << "," << goal_y_ << "," << n << "\n";
    replan_csv_.flush();
  }

  // GridMap -> Map2D5. Iterate OUR indices and query grid_map by position:
  // the two cell-center lattices coincide, so the lookup is exact, and
  // grid_map's inverted internal index ordering never enters the picture.
  std::unique_ptr<ballistic::Map2D5> convertMap(const grid_map_msgs::msg::GridMap & msg)
  {
    grid_map::GridMap gm;
    grid_map::GridMapRosConverter::fromMessage(msg, gm);

    const double res = gm.getResolution();
    const double size_x = gm.getLength().x();
    const double size_y = gm.getLength().y();
    const double origin_x = gm.getPosition().x() - size_x / 2.0;
    const double origin_y = gm.getPosition().y() - size_y / 2.0;

    auto map = std::make_unique<ballistic::Map2D5>(size_x, size_y, res, origin_x, origin_y);
    // The map contract (docs/lidar_mapping_plan.md): a measured cell is its
    // height; an unseen cell with an upper bound is terrain at the bound for the
    // clearance test, never a landing; anything else (unseen and unbounded, or a
    // hand-made map's NaN) is OBSTACLE, an infinitely tall column. A map without
    // an upper_bound layer is hand-made: every finite cell is landable.
    const bool lidar = gm.exists("upper_bound");
    std::vector<uint8_t> landable;
    if (lidar) {
      landable.assign(static_cast<size_t>(map->rows()) * map->cols(), 0);
    }
    for (int r = 0; r < map->rows(); ++r) {
      for (int c = 0; c < map->cols(); ++c) {
        const auto [x, y] = map->grid_to_world(r, c);
        const grid_map::Position pos(x, y);
        const float v = gm.atPosition("elevation", pos);
        if (!std::isnan(v)) {
          map->at(r, c) = static_cast<double>(v);
          if (lidar) {
            landable[static_cast<size_t>(r) * map->cols() + c] = 1;
          }
          continue;
        }
        const float b = lidar ? gm.atPosition("upper_bound", pos) : NAN;
        map->at(r, c) = std::isnan(b) ? ballistic::Map2D5::OBSTACLE : static_cast<double>(b);
      }
    }
    if (lidar) {
      map->set_landable(std::move(landable), lidar_edge_margin_, unseen_margin_, lidar_steep_noise_,
        lidar_edge_min_drop_);
    }
    return map;
  }

  // tilt is the landing that launches each hop; the last line is the stop.
  void logHopTable(const ballistic::HoppingAStarPlanner & planner)
  {
    std::ostringstream oss;
    oss << "\nhop      X      Z   alpha  a_leg    v_s   v_g   drop    J/kg   tilt\n";
    const auto & hops = planner.path_hops();
    const auto & stances = planner.path_stances();
    for (size_t i = 0; i < hops.size(); ++i) {
      const auto & h = hops[i];
      char line[160];
      std::snprintf(line, sizeof(line),
        "%3zu  %5.2f %+6.2f  %5.1f  %5.1f  %5.2f %5.2f  %5.2f  %6.2f  %5.1f\n",
        i + 1, h.X, h.Z, h.alpha_s / kDeg, h.alpha_leg / kDeg, h.v_s, h.v_g,
        h.apex_drop, h.e_inject / p_.mass, stances[i].tilt / kDeg);
      oss << line;
    }
    char line[80];
    std::snprintf(line, sizeof(line), "stop at the goal: tilt %.1f", stances.back().tilt / kDeg);
    oss << line;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  // Each hop's takeoff, and the stance the planner computed for it.
  void publishHopPlan(const ActivePlan & plan, const rclcpp::Time & stamp)
  {
    const auto & path = plan.path;
    if (path.size() < 2) {
      return;  // no hop, and no marker either (publishTrajectory)
    }
    const auto & hops = plan.hops;
    const auto & stances = plan.stances;
    hopcopter_msgs::msg::HopPlan msg;
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;
    msg.start.x = path[0].first;
    msg.start.y = path[0].second;
    msg.start.z = plan.ground[0];
    msg.start_speed = hops[0].v_g_in;
    msg.start_axis = toVector3(stances[0].axis);
    msg.start_tilt = stances[0].tilt;
    for (size_t i = 0; i < hops.size(); ++i) {
      const auto & h = hops[i];
      hopcopter_msgs::msg::PlannedHop hop;
      hop.landing.x = path[i + 1].first;
      hop.landing.y = path[i + 1].second;
      hop.landing.z = plan.ground[i + 1];
      hop.heading = h.heading;
      hop.alpha_leg = h.alpha_leg;
      hop.v_leg = h.v_leg;
      hop.v_h = h.v_h;
      hop.target_apex = h.apex_drop + h.Z;
      hop.e_inject_per_kg = h.e_inject / p_.mass;
      const double v_z = std::sqrt(std::max(0.0, h.v_g * h.v_g - h.v_h * h.v_h));
      hop.arrival_velocity.x = h.v_h * std::cos(h.heading);
      hop.arrival_velocity.y = h.v_h * std::sin(h.heading);
      hop.arrival_velocity.z = -v_z;
      hop.landing_axis = toVector3(stances[i + 1].axis);
      hop.landing_tilt = stances[i + 1].tilt;
      msg.hops.push_back(hop);
    }
    msg.momentum_aware = p_.momentum_aware;
    msg.g = p_.g;
    msg.eta = p_.eta;
    msg.k2 = p_.k2;
    plan_pub_->publish(msg);
  }

  // Apex height above takeoff ground of the hop that lands on path[i].
  // hops[i] is the edge path[i]->path[i+1]; path[0] has no incoming hop.
  double waypointApex(size_t i, const std::vector<ballistic::Hop> & hops) const
  {
    if (i == 0 || i > hops.size()) {
      return hover_offset_;
    }
    const auto & h = hops[i - 1];
    return h.apex_drop + h.Z;
  }

  void publishTrajectory(
    const std::vector<std::pair<double, double>> & path,
    const std::vector<ballistic::Hop> & hops,
    const rclcpp::Time & stamp)
  {
    if (path.size() < 2) {
      RCLCPP_WARN(get_logger(),
        "Path has %zu point(s); hopcopter ignores <2-point trajectories — not publishing.",
        path.size());
      return;
    }

    // DELETE first: puts hopcopter into "awaiting new path" so it does not
    // fall back to the previous goal. The pair is load-bearing.
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.header.stamp = stamp;
    del.id = kPathMarkerId;
    del.action = visualization_msgs::msg::Marker::DELETE;
    traj_pub_->publish(del);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = stamp;
    m.ns = "ballistic_path";
    m.id = kPathMarkerId;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.color.b = 1.0;  // blue, to distinguish from RRT*'s red
    m.color.a = 1.0;
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = path[i].first;
      pt.y = path[i].second;
      // Control target: hopcopter uses z as its desired hop height.
      pt.z = waypointApex(i, hops);
      m.points.push_back(pt);
    }
    traj_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "Published trajectory marker (id=%d, %zu points)",
      kPathMarkerId, m.points.size());
  }

  // ground: the start cell's height
  void publishStartPosition(double sx, double sy, double ground)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.header.stamp = now();
    ps.pose.position.x = sx;
    ps.pose.position.y = sy;
    ps.pose.position.z = ground + hover_offset_;
    ps.pose.orientation.w = 1.0;
    start_pub_->publish(ps);
  }

  static std::string makeStamp()
  {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream stamp;
    stamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return stamp.str();
  }

  // JSON number; non-finite (inflated OBSTACLE = +inf) -> null.
  static void jsonNum(std::ostream & os, double v)
  {
    if (std::isfinite(v)) {
      os << v;
    } else {
      os << "null";
    }
  }

  template<typename T>
  static void jsonArray(std::ostream & os, const char * key, const std::vector<T> & v)
  {
    os << "\"" << key << "\": [";
    for (size_t i = 0; i < v.size(); ++i) {
      if (i) {os << ",";}
      jsonNum(os, static_cast<double>(v[i]));
    }
    os << "],\n";
  }

  // Everything visualize_planned_path.py needs to redraw the plan, including
  // the planner's own map layers (same calls as HoppingAStarPlanner's ctor).
  // Returns the file path, or "" on failure.
  std::string writeVisSnapshot(
    const ballistic::Map2D5 & map,
    const std::vector<std::pair<double, double>> & path,
    const std::vector<ballistic::Hop> & hops,
    const std::vector<ballistic::Stance> & stances,
    double v_g_initial, double sx, double sy, double gx, double gy,
    const std::string & stamp, bool frontier)
  {
    const std::string file = vis_dir_ + "/snapshot_" + stamp + ".json";
    std::ofstream f(file);
    if (!f) {
      RCLCPP_ERROR(get_logger(), "Failed to open snapshot %s", file.c_str());
      return "";
    }

    std::vector<double> grid(static_cast<size_t>(map.rows()) * map.cols());
    for (int r = 0; r < map.rows(); ++r) {
      for (int c = 0; c < map.cols(); ++c) {
        grid[static_cast<size_t>(r) * map.cols() + c] = map.at(r, c);
      }
    }
    const auto steep = map.steep_mask(p_.steep_grade, false);
    const auto standable = map.standable_mask(
      p_.robot_radius, p_.side_clearance, p_.steep_grade);
    const auto inflated = map.inflated_field(
      p_.robot_radius + p_.side_clearance, p_.steep_grade);

    // hop_radii[i] is the ring available when departing path[i]: the first
    // from the seeded start speed, later ones from each hop's recorded radius.
    std::vector<double> hop_radii{ballistic::max_hop_radius(
        v_g_initial, p_.eta, p_.e_inject_max, p_.mass, p_.g, p_.V_max)};
    for (const auto & h : hops) {
      hop_radii.push_back(h.hop_radius);
    }
    std::vector<double> apex;
    for (size_t i = 0; i < path.size(); ++i) {
      apex.push_back(waypointApex(i, hops));
    }

    f << std::setprecision(9) << "{\n"
      << "\"stamp\": \"" << stamp << "\",\n"
      << "\"origin_x\": " << map.origin_x() << ",\n"
      << "\"origin_y\": " << map.origin_y() << ",\n"
      << "\"resolution\": " << map.resolution() << ",\n"
      << "\"rows\": " << map.rows() << ",\n"
      << "\"cols\": " << map.cols() << ",\n"
      << "\"robot_radius\": " << p_.robot_radius << ",\n"
      << "\"bottom_clearance\": " << p_.bottom_clearance << ",\n"
      << "\"side_clearance\": " << p_.side_clearance << ",\n"
      << "\"steep_grade\": " << p_.steep_grade << ",\n"
      << "\"start\": [" << sx << "," << sy << "],\n"
      << "\"goal\": [" << gx << "," << gy << "],\n"
      << "\"final_goal\": [" << goal_x_ << "," << goal_y_ << "],\n"
      << "\"frontier\": " << (frontier ? "true" : "false") << ",\n";
    f << "\"path\": [";
    for (size_t i = 0; i < path.size(); ++i) {
      if (i) {f << ",";}
      f << "[" << path[i].first << "," << path[i].second << "]";
    }
    f << "],\n";
    jsonArray(f, "target_apex", apex);
    jsonArray(f, "hop_radii", hop_radii);
    jsonArray(f, "grid", grid);
    jsonArray(f, "steep", steep);
    jsonArray(f, "standable", standable);
    jsonArray(f, "inflated", inflated);
    if (!map.landable_mask().empty()) {
      // LiDAR map: 1 = measured; 0 = unseen (its grid value is the upper bound,
      // or OBSTACLE when unbounded)
      jsonArray(f, "measured", map.landable_mask());
    }
    f << "\"leg_length\": " << p_.leg_length << ",\n"
      << "\"arc_max_step\": " << p_.arc_max_step << ",\n"
      << "\"momentum_aware\": " << (p_.momentum_aware ? "true" : "false") << ",\n"
      << "\"stop_tilt\": " << stances.back().tilt << ",\n";
    writeHopSideViews(f, map, path, hops, stances, inflated);
    f << "\"format\": 2\n}\n";
    return file;
  }

  // Per-hop side-view data. Samples come from the planner's own arc_samples
  // (same centreline, spacing and active mask clearance_floor_alpha inverts),
  // and min_clearance is Python's arc_clearance: min over active samples of
  // foot(u) - inflated(u), the number compared against bottom_clearance.
  void writeHopSideViews(
    std::ostream & f,
    const ballistic::Map2D5 & map,
    const std::vector<std::pair<double, double>> & path,
    const std::vector<ballistic::Hop> & hops,
    const std::vector<ballistic::Stance> & stances,
    const std::vector<double> & inflated)
  {
    f << "\"hops\": [\n";
    for (size_t i = 0; i < hops.size() && i + 1 < path.size(); ++i) {
      const auto & h = hops[i];
      const auto [x_s, y_s] = path[i];
      const auto [x_g, y_g] = path[i + 1];
      const auto [rs, cs] = map.world_to_grid(x_s, y_s);
      const auto [rg, cg] = map.world_to_grid(x_g, y_g);
      const double t_s = map.at(rs, cs);
      const double t_g = map.at(rg, cg);

      const double exempt = inflated[static_cast<size_t>(rs) * map.cols() + cs] > t_s + 1e-9 ?
        p_.clearance_start_exempt : 0.0;
      const auto sampled = ballistic::arc_samples(
        x_s, y_s, t_s, x_g, y_g, t_g, map, inflated, p_.arc_max_step, p_.clearance_active_tol, exempt);
      std::vector<double> u, field, ground;
      std::vector<uint8_t> active;
      double min_c = std::numeric_limits<double>::infinity();
      if (sampled) {
        u = sampled->u;
        field = sampled->field;
        active = sampled->active;
        const double X = sampled->X;
        const double tan_a = std::tan(h.alpha_s);
        const double k = (X * tan_a - (t_g - t_s)) / (X * X);
        for (size_t j = 0; j < u.size(); ++j) {
          const double cx = x_s + u[j] * (x_g - x_s) / X;
          const double cy = y_s + u[j] * (y_g - y_s) / X;
          const auto [r, c] = map.world_to_grid(cx, cy);
          ground.push_back(map.at(r, c));
          if (active[j]) {
            const double foot = t_s + u[j] * tan_a - k * u[j] * u[j];
            min_c = std::min(min_c, foot - field[j]);
          }
        }
      }

      f << (i ? ",\n" : "") << "{";
      jsonArray(f, "u", u);
      jsonArray(f, "field", field);
      jsonArray(f, "ground", ground);
      jsonArray(f, "active", active);
      f << "\"min_clearance\": ";
      jsonNum(f, min_c);
      f << ", \"t_s\": " << t_s << ", \"t_g\": " << t_g
        << ", \"alpha_s\": " << h.alpha_s << ", \"X\": " << h.X << ", \"Z\": " << h.Z
        << ", \"apex_drop\": " << h.apex_drop
        << ", \"target_apex\": " << h.apex_drop + h.Z
        << ", \"heading\": " << h.heading << ", \"alpha_leg\": " << h.alpha_leg
        << ", \"v_leg\": " << h.v_leg << ", \"v_h\": " << h.v_h
        << ", \"launch_tilt\": " << stances[i].tilt
        << ", \"land_tilt\": " << stances[i + 1].tilt << "}";
    }
    f << "\n],\n";
  }

  // Fire-and-forget: the trailing '&' keeps matplotlib off the planning thread.
  void renderSnapshot(const std::string & snapshot)
  {
    std::string script;
    try {
      script = ament_index_cpp::get_package_prefix("ballistic_motion_planner") +
        "/lib/ballistic_motion_planner/visualize_planned_path.py";
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Cannot locate visualize_planned_path.py: %s", e.what());
      return;
    }
    const std::string cmd = "python3 '" + script + "' '" + snapshot + "' &";
    if (std::system(cmd.c_str()) != 0) {
      RCLCPP_WARN(get_logger(), "Failed to launch path renderer: %s", cmd.c_str());
    }
  }

  void writeCsvs(
    const ballistic::Map2D5 & map,
    const std::vector<std::pair<double, double>> & path,
    const std::vector<ballistic::Hop> & hops,
    const std::vector<ballistic::Stance> & stances,
    const std::string & stamp)
  {
    {
      std::ofstream f(csv_dir_ + "/ballistic_path_" + stamp + ".csv");
      if (f) {
        f << "x,y,z\n" << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < path.size(); ++i) {
          f << path[i].first << "," << path[i].second << "," << waypointApex(i, hops) << "\n";
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open path CSV in %s", csv_dir_.c_str());
      }
    }
    {
      std::ofstream f(csv_dir_ + "/ballistic_hops_" + stamp + ".csv");
      if (f) {
        // launch_tilt: the landing that launches this hop; land_tilt: this hop's
        // landing (the next launch, or the stop at the goal)
        f << "hop,X,Z,alpha_deg,v_s,v_g,v_g_in,e_inject,apex_drop,target_apex,hop_radius,"
             "heading_deg,alpha_leg_deg,v_leg,v_h,e_inject_per_kg,launch_tilt_deg,land_tilt_deg\n"
          << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < hops.size(); ++i) {
          const auto & h = hops[i];
          f << i + 1 << "," << h.X << "," << h.Z << ","
            << h.alpha_s * 180.0 / M_PI << "," << h.v_s << "," << h.v_g << ","
            << h.v_g_in << "," << h.e_inject << "," << h.apex_drop << ","
            << h.apex_drop + h.Z << "," << h.hop_radius << ","
            << h.heading / kDeg << "," << h.alpha_leg / kDeg << "," << h.v_leg << ","
            << h.v_h << "," << h.e_inject / p_.mass << ","
            << stances[i].tilt / kDeg << "," << stances[i + 1].tilt / kDeg << "\n";
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open hops CSV in %s", csv_dir_.c_str());
      }
    }
    // Planned per-hop tracking, mirroring hopcopter's executed-hop CSV so the
    // two diff column-for-column. hops[i] is the edge path[i]->path[i+1], so
    // its landing waypoint is path[i+1]. Energies use the planner's own mass/g.
    {
      std::ofstream f(planning_csv_dir_ + "/planned_hops_" + stamp + ".csv");
      if (f) {
        f << "timestamp,hop_number,land_x,land_y,land_z,roll_deg,pitch_deg,yaw_deg,"
             "previous_apex_h,target_apex,potential_energy_J,landing_velocity_mps,landing_ke_J,"
             "takeoff_velocity_mps,takeoff_ke_J,energy_injection_J\n"
          << std::fixed << std::setprecision(6);
        const double ts = now().seconds();
        for (size_t i = 0; i < hops.size(); ++i) {
          const auto & h = hops[i];
          const auto & [lx, ly] = path[i + 1];
          const auto [r, c] = map.world_to_grid(lx, ly);
          const double land_z = map.at(r, c);  // ground, matches executed foot_z
          const double pe = p_.mass * p_.g * h.apex_drop;
          const double landing_ke = 0.5 * p_.mass * h.v_g * h.v_g;
          const double takeoff_ke = 0.5 * p_.mass * h.v_s * h.v_s;
          f << ts << "," << i + 1 << "," << lx << "," << ly << "," << land_z
            << ",,,,"  // roll_deg,pitch_deg,yaw_deg blank (not modeled in planning)
            << h.apex_drop << "," << h.apex_drop + h.Z << ","
            << pe << "," << h.v_g << "," << landing_ke << ","
            << h.v_s << "," << takeoff_ke << "," << h.e_inject << "\n";
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open planned-hops CSV in %s",
          planning_csv_dir_.c_str());
      }
    }
  }

  ballistic::PlannerParams p_;
  ballistic::FrontierParams frontier_;
  int frontier_attempts_ = 2;
  bool replan_enabled_ = true, replan_cut_ = true;
  double replan_interval_, replan_height_tol_, replan_hold_wait_, replan_hold_timeout_;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  bool have_goal_ = false, goal_reached_ = false, map_changed_ = false;
  int failed_plans_ = 0;
  double lidar_edge_margin_ = 0.1, unseen_margin_ = 0.3, lidar_steep_noise_ = 0.02;
  double lidar_edge_min_drop_ = 0.3;
  ActivePlan active_;
  ReplanState replan_state_ = ReplanState::kIdle;
  std::string replan_reason_, pending_reason_;
  std::chrono::steady_clock::time_point last_plan_time_, delete_time_, decision_time_;
  hopcopter_msgs::msg::ApexDecision decision_;
  bool have_decision_ = false;
  std::ofstream replan_csv_;
  double hover_offset_;
  bool render_png_;
  std::string csv_dir_;
  std::string planning_csv_dir_;
  std::string vis_dir_;

  grid_map_msgs::msg::GridMap::SharedPtr latest_map_msg_;
  double robot_x_ = 0.0, robot_y_ = 0.0;
  bool have_pose_ = false;

  // The search from a cut waypoint (docs/replan_cut_plan.md). cut_gen_ goes up whenever a
  // running search goes stale; cut_future_ is last, so it is destroyed (waits for the worker)
  // before anything the worker uses.
  std::string cut_reason_;
  std::pair<double, double> cut_from_{0.0, 0.0};
  bool cut_needed_ = false;
  uint64_t cut_gen_ = 0;
  std::chrono::steady_clock::time_point cut_next_start_;
  std::unique_ptr<SearchResult> cut_ready_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<hopcopter_msgs::msg::HopPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<hopcopter_msgs::msg::ApexDecision>::SharedPtr decision_sub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  std::future<std::unique_ptr<SearchResult>> cut_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallisticPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
