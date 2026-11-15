// Headless comparison of the planners on random collision-free queries.
//
//   ./planner_benchmark --trials 20 --seed 1
//   ./planner_benchmark --planner RRT-Connect --trials 50
#include <cstdio>
#include <iostream>
#include <map>
#include <random>

#include "AppOptions.h"
#include "MotionPlanner.h"
#include "PlanningScene.h"

int main(int argc, char** argv) {
  using namespace motion_planner;
  const AppOptions options = ParseAppOptions(argc, argv);
  PlanningScene scene(options.scene);
  auto planners = MakeAllPlanners(scene);
  std::mt19937 rng(options.seed ? options.seed : std::random_device{}());

  struct Totals {
    int ok{0}, produced{0};
    double plan_time{0}, duration{0}, joint_len{0};
  };
  std::map<std::string, Totals> totals;

  for (int trial = 0; trial < options.trials; ++trial) {
    PlanningRequest req;
    req.start = scene.SampleCollisionFree(&rng).value();
    req.goal = scene.SampleCollisionFree(&rng).value();
    req.time_limit = options.time_limit;
    req.seed = rng();
    const bool direct = scene.IsEdgeCollisionFree(req.start, req.goal);
    std::printf("--- query %d (straight line %s) ---\n", trial + 1,
                direct ? "free" : "blocked");
    for (const auto& planner : planners) {
      if (options.planner != "all" && options.planner != planner->name()) continue;
      const PlanningResult r = planner->Plan(req);
      std::cout << "  " << r.Summary() << "\n";
      Totals& t = totals[planner->name()];
      if (r.trajectory) {
        ++t.produced;
        t.plan_time += r.planning_time;
        t.duration += r.stats.duration;
        t.joint_len += r.stats.joint_path_length;
      }
      if (r.success && r.stats.collision_free) ++t.ok;
    }
  }

  std::printf("\n%-12s %14s %12s %12s %14s\n", "planner", "collision-free",
              "avg plan[s]", "avg move[s]", "avg path[rad]");
  for (const auto& planner : planners) {
    auto it = totals.find(planner->name());
    if (it == totals.end()) continue;
    const Totals& t = it->second;
    const double n = std::max(1, t.produced);
    std::printf("%-12s %9d/%-4d %12.2f %12.2f %14.2f\n", planner->name().c_str(),
                t.ok, options.trials, t.plan_time / n, t.duration / n,
                t.joint_len / n);
  }
  return 0;
}
