#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/trajectory.h"

#include "PlanningScene.h"
#include "TrajectoryUtils.h"

namespace motion_planner {

struct PlanningRequest {
  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  /// Wall-clock budget for the search/optimization [s].
  double time_limit{10.0};
  /// RNG seed for randomized planners (0 → nondeterministic).
  unsigned int seed{0};
};

struct PlanningResult {
  /// True if the planner produced a trajectory from start to goal. Planners
  /// that ignore obstacles can succeed with `stats.collision_free == false`.
  bool success{false};
  std::string planner_name;
  std::string message;
  std::shared_ptr<const Trajectory<double>> trajectory;
  /// Optional geometric path (e.g. RRT waypoints, Bézier control points).
  std::vector<Eigen::VectorXd> waypoints;
  double planning_time{0};  // [s]
  TrajectoryStats stats;    // filled in by MotionPlanner::Plan()

  /// One-line human readable summary.
  std::string Summary() const;
};

/// Base class for every planning method.
///
/// Subclasses implement DoPlan(); Plan() wraps it with input validation,
/// timing and a uniform trajectory evaluation so all methods are comparable.
/// To add a method: subclass, implement name()/description()/DoPlan(), and
/// register it in MakeAllPlanners() (Planners.cpp).
class MotionPlanner {
 public:
  explicit MotionPlanner(const PlanningScene& scene) : scene_(scene) {}
  virtual ~MotionPlanner() = default;
  MotionPlanner(const MotionPlanner&) = delete;
  MotionPlanner& operator=(const MotionPlanner&) = delete;

  virtual std::string name() const = 0;
  virtual std::string description() const = 0;
  /// Whether the method avoids obstacles by construction.
  virtual bool avoids_collisions() const = 0;

  PlanningResult Plan(const PlanningRequest& request) const;

 protected:
  virtual PlanningResult DoPlan(const PlanningRequest& request) const = 0;
  const PlanningScene& scene() const { return scene_; }

 private:
  const PlanningScene& scene_;
};

/// Builds one instance of every available planner, in UI display order.
std::vector<std::unique_ptr<MotionPlanner>> MakeAllPlanners(
    const PlanningScene& scene);

}  // namespace motion_planner
