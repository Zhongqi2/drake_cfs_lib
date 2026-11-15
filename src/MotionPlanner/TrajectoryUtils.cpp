#include "TrajectoryUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "drake/common/polynomial.h"

namespace motion_planner {

double QuinticDuration(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1,
                       const Eigen::VectorXd& vmax, double min_duration) {
  const double t = (1.875 * (q1 - q0).cwiseAbs().array() / vmax.array()).maxCoeff();
  return std::max(t, min_duration);
}

PiecewisePolynomial<double> QuinticSegment(const Eigen::VectorXd& q0,
                                           const Eigen::VectorXd& q1,
                                           double duration, double start_time) {
  const int n = q0.size();
  const Eigen::VectorXd dq = q1 - q0;
  const double T = duration;
  drake::MatrixX<drake::Polynomial<double>> poly(n, 1);
  for (int i = 0; i < n; ++i) {
    Eigen::VectorXd c(6);
    // Coefficients in ascending powers of (t - start_time).
    c << q0[i], 0.0, 0.0, 10.0 * dq[i] / std::pow(T, 3),
        -15.0 * dq[i] / std::pow(T, 4), 6.0 * dq[i] / std::pow(T, 5);
    poly(i, 0) = drake::Polynomial<double>(c);
  }
  return PiecewisePolynomial<double>({poly}, {start_time, start_time + T});
}

PiecewisePolynomial<double> StopAndGoTrajectory(
    const std::vector<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& vmax) {
  if (waypoints.size() < 2) {
    throw std::invalid_argument("StopAndGoTrajectory needs ≥ 2 waypoints");
  }
  PiecewisePolynomial<double> traj;
  double t = 0.0;
  for (size_t k = 0; k + 1 < waypoints.size(); ++k) {
    const double T = QuinticDuration(waypoints[k], waypoints[k + 1], vmax, 0.2);
    auto seg = QuinticSegment(waypoints[k], waypoints[k + 1], T, t);
    if (k == 0) {
      traj = seg;
    } else {
      traj.ConcatenateInTime(seg);
    }
    t += T;
  }
  return traj;
}

PiecewisePolynomial<double> SmoothSplineTrajectory(
    const std::vector<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& vmax) {
  const int m = waypoints.size();
  if (m < 2) throw std::invalid_argument("SmoothSplineTrajectory needs ≥ 2 waypoints");
  if (m == 2) {
    return QuinticSegment(waypoints[0], waypoints[1],
                          QuinticDuration(waypoints[0], waypoints[1], vmax));
  }
  // Break times proportional to the per-segment limit-respecting duration.
  // The 1.5 factor leaves room for the spline's speed-up between knots.
  std::vector<double> breaks{0.0};
  std::vector<Eigen::MatrixXd> samples;
  for (int k = 0; k < m; ++k) samples.push_back(waypoints[k]);
  for (int k = 0; k + 1 < m; ++k) {
    const double seg = 1.5 * ((waypoints[k + 1] - waypoints[k]).cwiseAbs().array() /
                              vmax.array()).maxCoeff();
    breaks.push_back(breaks.back() + std::max(seg, 0.1));
  }
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(waypoints[0].size());
  auto spline = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      breaks, samples, zero, zero);

  // Slow the whole spline down uniformly if it still exceeds a velocity limit.
  const auto qdot = spline.derivative();
  double ratio = 0;
  for (double t = 0; t <= spline.end_time(); t += 0.01) {
    ratio = std::max(ratio, (qdot.value(t).col(0).cwiseAbs().array() /
                             vmax.array()).maxCoeff());
  }
  if (ratio > 1.0) spline.ScaleTime(ratio * 1.02);
  return spline;
}

Eigen::MatrixXd SampleTrajectory(const Trajectory<double>& traj, double dt) {
  const double t0 = traj.start_time();
  const double tf = traj.end_time();
  const int n = std::max(2, static_cast<int>(std::ceil((tf - t0) / dt)) + 1);
  Eigen::MatrixXd q(traj.rows(), n);
  for (int i = 0; i < n; ++i) {
    const double t = std::min(t0 + i * dt, tf);
    q.col(i) = traj.value(t).col(0);
  }
  q.col(n - 1) = traj.value(tf).col(0);
  return q;
}

namespace {

// Samples dense enough that consecutive configurations are ≤ max_joint_step
// apart, estimated from a coarse pre-pass of the trajectory.
Eigen::MatrixXd DenseSamples(const Trajectory<double>& traj,
                             double max_joint_step) {
  const Eigen::MatrixXd coarse = SampleTrajectory(traj, 0.01);
  double max_step = 0;
  for (int i = 1; i < coarse.cols(); ++i) {
    max_step = std::max(max_step, (coarse.col(i) - coarse.col(i - 1)).norm());
  }
  if (max_step <= max_joint_step) return coarse;
  const double dt = 0.01 * max_joint_step / max_step;
  return SampleTrajectory(traj, dt);
}

}  // namespace

TrajectoryStats EvaluateTrajectory(const PlanningScene& scene,
                                   const Trajectory<double>& traj,
                                   const Eigen::VectorXd& goal,
                                   double max_joint_step) {
  TrajectoryStats s;
  s.duration = traj.end_time() - traj.start_time();
  const Eigen::MatrixXd q = DenseSamples(traj, max_joint_step);
  const double dt = s.duration / std::max<int>(1, q.cols() - 1);

  s.min_clearance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d prev_ee = scene.EndEffectorPosition(q.col(0));
  for (int i = 0; i < q.cols(); ++i) {
    const Eigen::VectorXd qi = q.col(i);
    if (i > 0) {
      s.joint_path_length += (qi - q.col(i - 1)).norm();
      const Eigen::Vector3d ee = scene.EndEffectorPosition(qi);
      s.ee_path_length += (ee - prev_ee).norm();
      prev_ee = ee;
      const Eigen::ArrayXd v = (qi - q.col(i - 1)).cwiseAbs().array() / dt;
      s.max_velocity_ratio = std::max(
          s.max_velocity_ratio, (v / scene.velocity_limits().array()).maxCoeff());
    }
    if (!scene.WithinLimits(qi, 1e-3)) s.within_limits = false;
    if (s.collision_free && !scene.IsCollisionFree(qi)) {
      s.collision_free = false;
      s.first_collision_time = traj.start_time() + i * dt;
    }
  }
  // Clearance is more expensive: evaluate on a coarser subset.
  const int stride = std::max<int>(1, q.cols() / 60);
  for (int i = 0; i < q.cols(); i += stride) {
    s.min_clearance = std::min(s.min_clearance, scene.MinDistance(q.col(i)));
  }
  s.reaches_goal = (traj.value(traj.end_time()).col(0) - goal).norm() < 1e-3;
  return s;
}

bool IsTrajectoryValid(const PlanningScene& scene,
                       const Trajectory<double>& traj, double max_joint_step) {
  const Eigen::MatrixXd q = DenseSamples(traj, max_joint_step);
  for (int i = 0; i < q.cols(); ++i) {
    if (!scene.WithinLimits(q.col(i), 1e-3) ||
        !scene.IsCollisionFree(q.col(i))) {
      return false;
    }
  }
  return true;
}

}  // namespace motion_planner
