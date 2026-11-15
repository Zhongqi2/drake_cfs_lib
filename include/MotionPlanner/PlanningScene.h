#pragma once

#include <memory>
#include <optional>
#include <random>
#include <string>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/collision_checker.h"

namespace motion_planner {

/// Robot + environment model shared by every planner.
///
/// Wraps a Drake SceneGraphCollisionChecker and exposes the handful of queries
/// planners need: joint limits, collision checks, clearance, random sampling
/// and end-effector forward kinematics.
class PlanningScene {
 public:
  struct Options {
    /// Model file (URDF/SDF/.dmd.yaml) given as a package:// URL.
    std::string model_url;
    /// Directory registered as the "project" package.
    std::string package_dir;
    /// Model instance that is the robot (everything else is environment).
    std::string robot_model_instance{"gp7"};
    /// Frame whose origin is drawn as the end-effector path.
    std::string end_effector_frame{"EndEffector_Link"};
    /// Joint-space step [rad] used when checking edges for collision.
    double edge_step_size{0.02};
    /// Safety margin [m] between robot and environment.
    double env_collision_padding{0.01};
    /// Safety margin [m] between robot links.
    double self_collision_padding{0.0};
    /// Fallback limits for joints that have infinite limits in the model.
    double default_position_limit{M_PI};
    double default_velocity_limit{1.0};
  };

  explicit PlanningScene(const Options& options);
  ~PlanningScene();

  const Options& options() const { return options_; }
  const drake::multibody::MultibodyPlant<double>& plant() const;
  const drake::planning::CollisionChecker& checker() const { return *checker_; }

  int num_positions() const { return lower_.size(); }
  const Eigen::VectorXd& lower_limits() const { return lower_; }
  const Eigen::VectorXd& upper_limits() const { return upper_; }
  /// Symmetric velocity limit (always finite and positive).
  const Eigen::VectorXd& velocity_limits() const { return vmax_; }
  std::vector<std::string> joint_names() const;

  bool WithinLimits(const Eigen::VectorXd& q, double tol = 1e-6) const;
  bool IsCollisionFree(const Eigen::VectorXd& q) const;
  bool IsEdgeCollisionFree(const Eigen::VectorXd& q1,
                           const Eigen::VectorXd& q2) const;
  /// Smallest signed distance between the robot and anything else, capped at
  /// `influence_distance` (returned when nothing is closer than that).
  double MinDistance(const Eigen::VectorXd& q,
                     double influence_distance = 0.5) const;

  Eigen::VectorXd SampleUniform(std::mt19937* rng) const;
  std::optional<Eigen::VectorXd> SampleCollisionFree(
      std::mt19937* rng, int max_tries = 2000) const;
  Eigen::VectorXd Clamp(const Eigen::VectorXd& q) const;

  Eigen::Vector3d EndEffectorPosition(const Eigen::VectorXd& q) const;

 private:
  Options options_;
  std::unique_ptr<drake::planning::CollisionChecker> checker_;
  Eigen::VectorXd lower_, upper_, vmax_;
};

}  // namespace motion_planner
