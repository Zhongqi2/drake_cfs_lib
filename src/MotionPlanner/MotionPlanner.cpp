#include "MotionPlanner.h"

#include <chrono>
#include <cstdio>
#include <exception>

#include "Planners.h"

namespace motion_planner {

std::string PlanningResult::Summary() const {
  char buf[512];
  if (!trajectory) {
    std::snprintf(buf, sizeof(buf), "[%s] FAILED after %.2fs: %s",
                  planner_name.c_str(), planning_time, message.c_str());
    return buf;
  }
  std::snprintf(
      buf, sizeof(buf),
      "[%s] %s  plan %.2fs | motion %.2fs | joint path %.2f rad | EE path "
      "%.2f m | min clearance %.3f m | %s%s%s",
      planner_name.c_str(), success ? "OK" : "FAILED", planning_time,
      stats.duration, stats.joint_path_length, stats.ee_path_length,
      stats.min_clearance,
      stats.collision_free ? "collision-free" : "IN COLLISION",
      stats.within_limits ? "" : " | VIOLATES JOINT LIMITS",
      message.empty() ? "" : (" | " + message).c_str());
  return buf;
}

PlanningResult MotionPlanner::Plan(const PlanningRequest& request) const {
  PlanningResult result;
  result.planner_name = name();
  const int n = scene_.num_positions();
  if (request.start.size() != n || request.goal.size() != n) {
    result.message = "start/goal must have " + std::to_string(n) + " entries";
    return result;
  }
  if (!scene_.WithinLimits(request.start) || !scene_.WithinLimits(request.goal)) {
    result.message = "start or goal violates joint limits";
    return result;
  }
  if (avoids_collisions()) {
    if (!scene_.IsCollisionFree(request.start)) {
      result.message = "start configuration is in collision";
      return result;
    }
    if (!scene_.IsCollisionFree(request.goal)) {
      result.message = "goal configuration is in collision";
      return result;
    }
  }

  const auto t0 = std::chrono::steady_clock::now();
  try {
    result = DoPlan(request);
  } catch (const std::exception& e) {
    result = PlanningResult{};
    result.message = std::string("exception: ") + e.what();
  }
  result.planner_name = name();
  result.planning_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  if (!result.trajectory) {
    result.success = false;
    return result;
  }
  result.stats = EvaluateTrajectory(scene_, *result.trajectory, request.goal);
  if (!result.stats.reaches_goal) {
    result.success = false;
    result.message = "trajectory does not end at the goal";
  } else if (!result.stats.collision_free) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "first collision at t=%.2fs",
                  result.stats.first_collision_time);
    // A method that promises collision avoidance has failed; a method that
    // ignores obstacles still "succeeds" but the warning is reported.
    if (avoids_collisions()) result.success = false;
    result.message = buf;
  }
  return result;
}

std::vector<std::unique_ptr<MotionPlanner>> MakeAllPlanners(
    const PlanningScene& scene) {
  std::vector<std::unique_ptr<MotionPlanner>> planners;
  planners.push_back(std::make_unique<LinearPlanner>(scene));
  planners.push_back(std::make_unique<QuinticPolynomialPlanner>(scene));
  planners.push_back(std::make_unique<BezierPlanner>(scene));
  planners.push_back(std::make_unique<RRTConnectPlanner>(scene));
  planners.push_back(std::make_unique<TrajOptPlanner>(scene));
  return planners;
}

}  // namespace motion_planner
