#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

#include "MotionPlanner.h"
#include "PlanningScene.h"

namespace motion_planner {

/// Browser UI built from Meshcat controls (sliders + buttons in the panel on
/// the right of the Drake visualizer).
///
///  * Joint sliders pose the robot; "Set START/GOAL" stores that pose.
///  * "Random START/GOAL" samples collision-free poses.
///  * One "Plan: <method>" button per planner plans and animates the motion.
///  * "Reset: same START & GOAL" undoes the goal→start chaining so another
///    planner can be tried on the same query.
///  * "Compare all" runs every planner and draws all end-effector paths.
/// Best guess at this machine's reachable IPv4 address: the server side of
/// the current SSH connection if any, otherwise the first non-loopback,
/// non-container interface; "localhost" if nothing is found.
std::string DetectHostIp();

class PlannerUI {
 public:
  PlannerUI(const PlanningScene& scene,
            std::vector<std::unique_ptr<MotionPlanner>> planners,
            unsigned int seed = 0, const std::string& host_ip = "");
  ~PlannerUI();

  std::string url() const { return meshcat_->web_url(); }
  /// Blocks forever, polling the browser controls.
  void Run();

 private:
  void AddControls();
  bool Clicked(const std::string& button);
  void SetStatus(const std::string& text);

  Eigen::VectorXd ReadSliders() const;
  void WriteSliders(const Eigen::VectorXd& q);
  void ShowConfiguration(const Eigen::VectorXd& q);
  void DrawStartGoalMarkers();
  void DrawEndEffectorPath(const std::string& path,
                           const Trajectory<double>& traj,
                           const drake::geometry::Rgba& color);

  void SetStart(const Eigen::VectorXd& q);
  void SetGoal(const Eigen::VectorXd& q);
  bool StartEqualsGoal();
  void PlanAndExecute(size_t planner_index);
  void Animate(const Trajectory<double>& traj);
  void CompareAll();
  void ResetToLastQuery();

  const PlanningScene& scene_;
  std::vector<std::unique_ptr<MotionPlanner>> planners_;
  std::mt19937 rng_;

  // Visualization-only diagram (separate from the collision checker's model).
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  drake::multibody::MultibodyPlant<double>* plant_{};
  std::unique_ptr<drake::systems::Context<double>> context_;

  std::vector<std::string> joint_sliders_;
  std::map<std::string, int> clicks_seen_;
  std::string status_button_;

  Eigen::VectorXd start_, goal_, shown_;
  // Start/goal of the last executed motion, for "Reset: same START & GOAL".
  Eigen::VectorXd last_start_, last_goal_;
  bool shown_in_collision_{false};
  std::shared_ptr<const Trajectory<double>> last_trajectory_;
};

}  // namespace motion_planner
