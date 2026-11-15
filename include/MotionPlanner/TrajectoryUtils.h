#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

#include "PlanningScene.h"

namespace motion_planner {

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

/// Shortest duration for a rest-to-rest quintic (min-jerk) move q0 → q1 that
/// respects the velocity limits (peak quintic speed is 1.875·|Δq|/T).
double QuinticDuration(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1,
                       const Eigen::VectorXd& vmax, double min_duration = 0.3);

/// Rest-to-rest quintic polynomial q(t) = q0 + Δq·(10τ³ − 15τ⁴ + 6τ⁵),
/// τ = (t − t0)/T. Zero velocity and acceleration at both ends.
PiecewisePolynomial<double> QuinticSegment(const Eigen::VectorXd& q0,
                                           const Eigen::VectorXd& q1,
                                           double duration,
                                           double start_time = 0.0);

/// Joint-space polyline turned into a trajectory that stops at every waypoint
/// (one quintic per segment). It follows the polyline exactly, so it is
/// collision-free whenever every polyline edge is.
PiecewisePolynomial<double> StopAndGoTrajectory(
    const std::vector<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& vmax);

/// C2 cubic spline through the waypoints with zero end velocities. Smoother
/// than StopAndGoTrajectory but may deviate (overshoot) between waypoints.
PiecewisePolynomial<double> SmoothSplineTrajectory(
    const std::vector<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& vmax);

/// Samples the trajectory at fixed time steps (end time always included).
/// Returns one configuration per column.
Eigen::MatrixXd SampleTrajectory(const Trajectory<double>& traj, double dt);

/// Quality measures of a trajectory, computed the same way for every planner
/// so the methods can be compared fairly.
struct TrajectoryStats {
  double duration{0};
  double joint_path_length{0};       // Σ‖Δq‖ [rad]
  double ee_path_length{0};          // end-effector path length [m]
  double min_clearance{0};           // smallest robot distance [m]
  bool within_limits{true};
  bool collision_free{true};
  double first_collision_time{-1};   // -1 if collision-free
  bool reaches_goal{true};
  double max_velocity_ratio{0};      // max |q̇ᵢ| / vmaxᵢ (≤1 is feasible)
};

/// Checks the trajectory densely: samples are at most `max_joint_step` apart
/// in joint space and every sample is collision-checked.
TrajectoryStats EvaluateTrajectory(const PlanningScene& scene,
                                   const Trajectory<double>& traj,
                                   const Eigen::VectorXd& goal,
                                   double max_joint_step = 0.02);

/// True if every sample of traj (dense in joint space) is collision-free and
/// within the joint limits.
bool IsTrajectoryValid(const PlanningScene& scene,
                       const Trajectory<double>& traj,
                       double max_joint_step = 0.02);

}  // namespace motion_planner
