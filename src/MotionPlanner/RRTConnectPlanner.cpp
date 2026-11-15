// RRT-Connect with shortcut smoothing.
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

#include "Planners.h"

namespace motion_planner {

namespace {

struct Tree {
  std::vector<Eigen::VectorXd> q;
  std::vector<int> parent;

  int Add(const Eigen::VectorXd& x, int p) {
    q.push_back(x);
    parent.push_back(p);
    return static_cast<int>(q.size()) - 1;
  }
  int Nearest(const Eigen::VectorXd& x) const {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < q.size(); ++i) {
      const double d = (q[i] - x).squaredNorm();
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  }
  // Root → node.
  std::vector<Eigen::VectorXd> PathTo(int node) const {
    std::vector<Eigen::VectorXd> path;
    for (int i = node; i >= 0; i = parent[i]) path.push_back(q[i]);
    std::reverse(path.begin(), path.end());
    return path;
  }
};

enum class ExtendStatus { kTrapped, kAdvanced, kReached };

}  // namespace

std::vector<Eigen::VectorXd> RRTConnectPlanner::FindPath(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    double time_limit, std::mt19937* rng, std::string* message) const {
  if (scene().IsEdgeCollisionFree(start, goal)) {
    *message = "direct edge is free";
    return {start, goal};
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(time_limit);
  Tree tree_a, tree_b;  // a: grown from start, b: from goal (swapped each step)
  tree_a.Add(start, -1);
  tree_b.Add(goal, -1);
  Tree* ta = &tree_a;
  Tree* tb = &tree_b;

  // Moves one step from the nearest node of `tree` toward `target`.
  auto extend = [&](Tree* tree, const Eigen::VectorXd& target, int* new_node) {
    const int near = tree->Nearest(target);
    const Eigen::VectorXd& q_near = tree->q[near];
    const Eigen::VectorXd delta = target - q_near;
    const double dist = delta.norm();
    const bool reached = dist <= params_.step_size;
    const Eigen::VectorXd q_new =
        reached ? target : Eigen::VectorXd(q_near + delta * (params_.step_size / dist));
    if (!scene().IsEdgeCollisionFree(q_near, q_new)) return ExtendStatus::kTrapped;
    *new_node = tree->Add(q_new, near);
    return reached ? ExtendStatus::kReached : ExtendStatus::kAdvanced;
  };

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    if (std::chrono::steady_clock::now() > deadline) {
      *message = "time limit reached";
      return {};
    }
    const Eigen::VectorXd q_rand = scene().SampleUniform(rng);
    int a_new = -1;
    if (extend(ta, q_rand, &a_new) != ExtendStatus::kTrapped) {
      // Greedily connect the other tree to the newly added node.
      const Eigen::VectorXd target = ta->q[a_new];
      int b_new = -1;
      ExtendStatus status;
      do {
        status = extend(tb, target, &b_new);
      } while (status == ExtendStatus::kAdvanced);

      if (status == ExtendStatus::kReached) {
        auto path_a = ta->PathTo(a_new);
        auto path_b = tb->PathTo(b_new);  // root(b) → target
        std::reverse(path_b.begin(), path_b.end());
        path_a.insert(path_a.end(), path_b.begin() + 1, path_b.end());
        if (ta != &tree_a) std::reverse(path_a.begin(), path_a.end());
        *message = "trees connected after " + std::to_string(iter + 1) +
                   " iterations, " +
                   std::to_string(tree_a.q.size() + tree_b.q.size()) + " nodes";
        return path_a;
      }
    }
    std::swap(ta, tb);
  }
  *message = "iteration limit reached";
  return {};
}

std::vector<Eigen::VectorXd> RRTConnectPlanner::Shortcut(
    std::vector<Eigen::VectorXd> path, std::mt19937* rng) const {
  for (int k = 0; k < params_.shortcut_iterations && path.size() > 2; ++k) {
    std::uniform_int_distribution<int> pick(0, static_cast<int>(path.size()) - 1);
    int i = pick(*rng), j = pick(*rng);
    if (i > j) std::swap(i, j);
    if (j - i < 2) continue;
    if (scene().IsEdgeCollisionFree(path[i], path[j])) {
      path.erase(path.begin() + i + 1, path.begin() + j);
    }
  }
  return path;
}

PlanningResult RRTConnectPlanner::DoPlan(const PlanningRequest& req) const {
  PlanningResult result;
  std::mt19937 rng(req.seed ? req.seed : std::random_device{}());
  std::string info;
  auto path = FindPath(req.start, req.goal, req.time_limit, &rng, &info);
  if (path.empty()) {
    result.message = info;
    return result;
  }
  const size_t raw = path.size();
  path = Shortcut(std::move(path), &rng);
  result.waypoints = path;

  // Prefer a smooth spline; fall back to stopping at each verified waypoint.
  auto smooth = std::make_shared<PiecewisePolynomial<double>>(
      SmoothSplineTrajectory(path, scene().velocity_limits()));
  std::string timing = "smooth spline";
  if (IsTrajectoryValid(scene(), *smooth)) {
    result.trajectory = smooth;
  } else {
    result.trajectory = std::make_shared<PiecewisePolynomial<double>>(
        StopAndGoTrajectory(path, scene().velocity_limits()));
    timing = "stop-and-go (spline clipped an obstacle)";
  }
  result.success = true;
  result.message = info + "; " + std::to_string(raw) + " → " +
                   std::to_string(path.size()) + " waypoints; " + timing;
  return result;
}

}  // namespace motion_planner
