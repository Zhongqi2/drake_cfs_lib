// Interactive motion planner: pick start/goal and a method in the browser,
// watch the robot move in the Drake (Meshcat) visualizer.
//
//   ./planner_ui                       # demo scene with obstacles
//   ./planner_ui --model package://project/script/environment_description/robot_environment.dmd.yaml
#include <iostream>

#include "AppOptions.h"
#include "MotionPlanner.h"
#include "PlannerUI.h"
#include "PlanningScene.h"

int main(int argc, char** argv) {
  using namespace motion_planner;
  const AppOptions options = ParseAppOptions(argc, argv);
  std::cout << "Loading " << options.scene.model_url << " ..." << std::endl;
  PlanningScene scene(options.scene);
  PlannerUI ui(scene, MakeAllPlanners(scene), options.seed, options.host_ip);
  ui.Run();
  return 0;
}
