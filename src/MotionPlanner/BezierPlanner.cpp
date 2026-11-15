// Bézier-curve planner with a randomized via-point search.
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

#include "drake/common/trajectories/bezier_curve.h"

#include "Planners.h"

namespace motion_planner {

using drake::trajectories::BezierCurve;

namespace {

Eigen::MatrixXd ControlPoints(const Eigen::VectorXd& s, const Eigen::VectorXd& c,
                              const Eigen::VectorXd& g) {
  Eigen::MatrixXd P(s.size(), 5);
  P << s, s, c, g, g;
  return P;
}

// Shortest duration such that |dq_i/dt| ≤ vmax_i along the whole curve.
double BezierDuration(const Eigen::MatrixXd& P, const Eigen::VectorXd& vmax) {
  const BezierCurve<double> unit(0.0, 1.0, P);
  const auto d = unit.MakeDerivative();
  double ratio = 0;
  for (int k = 0; k <= 100; ++k) {
    ratio = std::max(ratio, (d->value(k / 100.0).col(0).cwiseAbs().array() /
                             vmax.array()).maxCoeff());
  }
  return std::max(ratio, 0.3);
}

double CurveLength(const BezierCurve<double>& curve) {
  double len = 0;
  Eigen::VectorXd prev = curve.value(curve.start_time()).col(0);
  for (int k = 1; k <= 50; ++k) {
    const double t = curve.start_time() +
                     k / 50.0 * (curve.end_time() - curve.start_time());
    const Eigen::VectorXd q = curve.value(t).col(0);
    len += (q - prev).norm();
    prev = q;
  }
  return len;
}

}  // namespace

PlanningResult BezierPlanner::DoPlan(const PlanningRequest& req) const {
  PlanningResult result;
  const Eigen::VectorXd& s = req.start;
  const Eigen::VectorXd& g = req.goal;
  const Eigen::VectorXd mid = 0.5 * (s + g);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(req.time_limit);

  std::mt19937 rng(req.seed ? req.seed : std::random_device{}());
  std::normal_distribution<double> normal(0.0, 1.0);

  // A candidate middle control point c is accepted if the whole curve is
  // inside the joint limits and collision-free.
  double best_len = std::numeric_limits<double>::infinity();
  Eigen::VectorXd best_c;
  int num_valid = 0;
  auto try_candidate = [&](const Eigen::VectorXd& c) {
    const BezierCurve<double> curve(0.0, 1.0, ControlPoints(s, c, g));
    if (!IsTrajectoryValid(scene(), curve)) return false;
    ++num_valid;
    const double len = CurveLength(curve);
    if (len < best_len) {
      best_len = len;
      best_c = c;
    }
    return true;
  };

  int attempts = 0;
  // c = midpoint gives the straight joint-space line: try it first.
  if (!try_candidate(mid)) {
    for (; attempts < params_.max_attempts &&
           num_valid < params_.candidates_to_compare &&
           std::chrono::steady_clock::now() < deadline;
         ++attempts) {
      // Widen the search gradually if nearby via points keep failing.
      const double sigma =
          params_.initial_sigma + (params_.max_sigma - params_.initial_sigma) *
                                      attempts / params_.max_attempts;
      Eigen::VectorXd v(s.size());
      for (int i = 0; i < v.size(); ++i) v[i] = mid[i] + sigma * normal(rng);
      v = scene().Clamp(v);
      if (!scene().IsCollisionFree(v)) continue;
      // Choose c so that B(½) = v:  B(½) = (5s + 6c + 5g) / 16.
      const Eigen::VectorXd c = (16.0 * v - 5.0 * s - 5.0 * g) / 6.0;
      try_candidate(c);
    }
  }

  if (num_valid == 0) {
    result.message = "no collision-free Bezier curve found after " +
                     std::to_string(attempts) + " via-point samples";
    return result;
  }
  const Eigen::MatrixXd P = ControlPoints(s, best_c, g);
  const double T = BezierDuration(P, scene().velocity_limits());
  result.trajectory = std::make_shared<BezierCurve<double>>(0.0, T, P);
  for (int k = 0; k < P.cols(); ++k) result.waypoints.push_back(P.col(k));
  result.success = true;
  result.message = attempts == 0 ? "direct curve"
                                 : "bent via-point curve (" +
                                       std::to_string(attempts) + " samples)";
  return result;
}

}  // namespace motion_planner
