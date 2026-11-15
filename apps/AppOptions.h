#pragma once
// Tiny command-line parser shared by the planner apps.

#include <cstdlib>
#include <iostream>
#include <string>

#include "PlanningScene.h"

#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif

namespace motion_planner {

struct AppOptions {
  PlanningScene::Options scene;
  unsigned int seed{0};
  std::string host_ip;  // empty → auto-detect
  int trials{10};
  std::string planner{"all"};
  double time_limit{10.0};
};

inline AppOptions ParseAppOptions(int argc, char** argv) {
  AppOptions o;
  o.scene.package_dir = PROJECT_ROOT;
  o.scene.model_url =
      "package://project/script/environment_description/planner_demo.dmd.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << arg << "\n";
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--model") o.scene.model_url = next();
    else if (arg == "--package-dir") o.scene.package_dir = next();
    else if (arg == "--robot") o.scene.robot_model_instance = next();
    else if (arg == "--ee-frame") o.scene.end_effector_frame = next();
    else if (arg == "--seed") o.seed = std::stoul(next());
    else if (arg == "--host-ip") o.host_ip = next();
    else if (arg == "--trials") o.trials = std::stoi(next());
    else if (arg == "--planner") o.planner = next();
    else if (arg == "--time-limit") o.time_limit = std::stod(next());
    else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Options:\n"
             "  --model URL         package:// model or directives file\n"
             "                      (default: planner_demo.dmd.yaml)\n"
             "  --package-dir DIR   directory of the 'project' package\n"
             "  --robot NAME        robot model instance (default gp7)\n"
             "  --ee-frame NAME     end-effector frame (default EndEffector_Link)\n"
             "  --seed N            RNG seed (0 = random)\n"
             "  --host-ip IP        [ui] IP shown in the URL (default: auto)\n"
             "  --trials N          [benchmark] number of random queries\n"
             "  --planner NAME      [benchmark] planner name or 'all'\n"
             "  --time-limit S      [benchmark] per-query time limit\n";
      std::exit(0);
    } else {
      std::cerr << "unknown option " << arg << " (see --help)\n";
      std::exit(1);
    }
  }
  return o;
}

}  // namespace motion_planner
