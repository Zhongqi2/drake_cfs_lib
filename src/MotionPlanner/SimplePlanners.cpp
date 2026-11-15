// Linear and quintic-polynomial planners: obstacle-unaware baselines.
#include <algorithm>
#include <memory>

#include "Planners.h"

namespace motion_planner {

PlanningResult LinearPlanner::DoPlan(const PlanningRequest& req) const {
  PlanningResult result;
  // Constant speed: the slowest joint (relative to its limit) sets the time.
  const double T = std::max(
      0.3, ((req.goal - req.start).cwiseAbs().array() /
            scene().velocity_limits().array()).maxCoeff());
  Eigen::MatrixXd samples(req.start.size(), 2);
  samples << req.start, req.goal;
  result.trajectory = std::make_shared<PiecewisePolynomial<double>>(
      PiecewisePolynomial<double>::FirstOrderHold(Eigen::Vector2d(0.0, T),
                                                  samples));
  result.waypoints = {req.start, req.goal};
  result.success = true;
  return result;
}

PlanningResult QuinticPolynomialPlanner::DoPlan(const PlanningRequest& req) const {
  PlanningResult result;
  const double T = QuinticDuration(req.start, req.goal, scene().velocity_limits());
  result.trajectory = std::make_shared<PiecewisePolynomial<double>>(
      QuinticSegment(req.start, req.goal, T));
  result.waypoints = {req.start, req.goal};
  result.success = true;
  return result;
}

}  // namespace motion_planner
