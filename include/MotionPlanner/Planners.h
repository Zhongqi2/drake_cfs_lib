#pragma once

#include <random>

#include "MotionPlanner.h"

namespace motion_planner {

/// Straight line in joint space at constant speed (velocity jumps at the
/// ends). Ignores obstacles: the simplest baseline.
class LinearPlanner final : public MotionPlanner {
 public:
  using MotionPlanner::MotionPlanner;
  std::string name() const override { return "Linear"; }
  std::string description() const override {
    return "Joint-space straight line, constant velocity (ignores obstacles)";
  }
  bool avoids_collisions() const override { return false; }

 protected:
  PlanningResult DoPlan(const PlanningRequest& request) const override;
};

/// Quintic (5th-order) polynomial per joint: same straight path as Linear but
/// with zero velocity and acceleration at both ends (minimum jerk).
class QuinticPolynomialPlanner final : public MotionPlanner {
 public:
  using MotionPlanner::MotionPlanner;
  std::string name() const override { return "Quintic"; }
  std::string description() const override {
    return "Quintic polynomial, smooth min-jerk timing (ignores obstacles)";
  }
  bool avoids_collisions() const override { return false; }

 protected:
  PlanningResult DoPlan(const PlanningRequest& request) const override;
};

/// Degree-4 Bézier curve with control points [s, s, c, g, g]. The repeated
/// end points give zero end velocities; the middle point c bends the curve.
/// If the straight curve collides, random via-configurations v are sampled and
/// c is chosen so the curve passes through v (B(½) = v); the shortest
/// collision-free candidate wins.
class BezierPlanner final : public MotionPlanner {
 public:
  struct Params {
    int max_attempts{3000};
    int candidates_to_compare{5};
    double initial_sigma{0.3};  // [rad] spread of via-point samples
    double max_sigma{2.0};
  };
  explicit BezierPlanner(const PlanningScene& scene) : BezierPlanner(scene, Params{}) {}
  BezierPlanner(const PlanningScene& scene, Params params)
      : MotionPlanner(scene), params_(params) {}
  std::string name() const override { return "Bezier"; }
  std::string description() const override {
    return "Bezier curve, random via-point search to bend around obstacles";
  }
  bool avoids_collisions() const override { return true; }

 protected:
  PlanningResult DoPlan(const PlanningRequest& request) const override;

 private:
  Params params_;
};

/// RRT-Connect (Kuffner & LaValle 2000): two trees grown from start and goal
/// toward random samples until they meet, followed by random shortcutting and
/// time parameterization (smooth spline if it stays collision-free, otherwise
/// stop-and-go quintics along the verified polyline).
class RRTConnectPlanner final : public MotionPlanner {
 public:
  struct Params {
    double step_size{0.2};   // [rad] max extension per step
    int max_iterations{20000};
    int shortcut_iterations{200};
  };
  explicit RRTConnectPlanner(const PlanningScene& scene) : RRTConnectPlanner(scene, Params{}) {}
  RRTConnectPlanner(const PlanningScene& scene, Params params)
      : MotionPlanner(scene), params_(params) {}
  std::string name() const override { return "RRT-Connect"; }
  std::string description() const override {
    return "Sampling-based RRT-Connect + shortcut smoothing";
  }
  bool avoids_collisions() const override { return true; }

  /// Exposed so other planners (e.g. TrajOpt) can use it as a seed.
  std::vector<Eigen::VectorXd> FindPath(const Eigen::VectorXd& start,
                                        const Eigen::VectorXd& goal,
                                        double time_limit, std::mt19937* rng,
                                        std::string* message) const;
  std::vector<Eigen::VectorXd> Shortcut(std::vector<Eigen::VectorXd> path,
                                        std::mt19937* rng) const;

 protected:
  PlanningResult DoPlan(const PlanningRequest& request) const override;

 private:
  Params params_;
};

/// Drake KinematicTrajectoryOptimization: a B-spline whose control points are
/// optimized for short duration and path length, subject to joint position /
/// velocity limits and minimum-distance (collision) constraints. If the
/// straight line is free it is solved in two stages from a straight seed
/// (without, then with collision constraints, as in the original
/// KinematicTrajOpt); otherwise, or if that fails, it is seeded with
/// RRT-Connect paths and tried with SNOPT, then IPOPT.
class TrajOptPlanner final : public MotionPlanner {
 public:
  struct Params {
    int num_control_points{12};
    int spline_order{4};
    int num_collision_samples{40};
    // Extra distance [m] on top of the checker's env_collision_padding (the
    // checker already subtracts its padding from every distance).
    double min_distance{0.01};
    double influence_distance_offset{0.05};
    double min_duration{0.5};
    double max_duration{15.0};
    bool use_rrt_seed_fallback{true};
    int max_rrt_seeds{2};  // fresh RRT seeds to try (each with SNOPT, IPOPT)
    int max_major_iterations{300};
  };
  explicit TrajOptPlanner(const PlanningScene& scene) : TrajOptPlanner(scene, Params{}) {}
  TrajOptPlanner(const PlanningScene& scene, Params params)
      : MotionPlanner(scene), params_(params) {}
  std::string name() const override { return "TrajOpt"; }
  std::string description() const override {
    return "Kinematic trajectory optimization (B-spline, collision constraints)";
  }
  bool avoids_collisions() const override { return true; }

 protected:
  PlanningResult DoPlan(const PlanningRequest& request) const override;

 private:
  Params params_;
};

}  // namespace motion_planner
