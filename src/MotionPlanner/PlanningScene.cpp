#include "PlanningScene.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace motion_planner {

using drake::planning::CollisionCheckerParams;
using drake::planning::RobotDiagramBuilder;
using drake::planning::SceneGraphCollisionChecker;

PlanningScene::PlanningScene(const Options& options) : options_(options) {
  auto builder = std::make_unique<RobotDiagramBuilder<double>>(0.0);
  builder->parser().package_map().Add("project", options_.package_dir);
  builder->parser().AddModelsFromUrl(options_.model_url);
  builder->plant().Finalize();

  const auto& plant = builder->plant();
  if (!plant.HasModelInstanceNamed(options_.robot_model_instance)) {
    throw std::runtime_error("PlanningScene: no model instance named '" +
                             options_.robot_model_instance + "'");
  }
  const auto robot = plant.GetModelInstanceByName(options_.robot_model_instance);

  // Joint limits; replace infinite ones (e.g. continuous joints) with defaults.
  lower_ = plant.GetPositionLowerLimits();
  upper_ = plant.GetPositionUpperLimits();
  vmax_ = plant.GetVelocityUpperLimits().cwiseAbs();
  for (int i = 0; i < lower_.size(); ++i) {
    if (!std::isfinite(lower_[i])) lower_[i] = -options_.default_position_limit;
    if (!std::isfinite(upper_[i])) upper_[i] = options_.default_position_limit;
    if (!std::isfinite(vmax_[i]) || vmax_[i] <= 0) {
      vmax_[i] = options_.default_velocity_limit;
    }
  }
  if (plant.num_positions() != plant.num_velocities()) {
    throw std::runtime_error(
        "PlanningScene: only robots with q̇ = v (no floating bases) are "
        "supported");
  }

  CollisionCheckerParams params;
  params.model = builder->Build();
  params.robot_model_instances = {robot};
  params.edge_step_size = options_.edge_step_size;
  params.env_collision_padding = options_.env_collision_padding;
  params.self_collision_padding = options_.self_collision_padding;
  params.configuration_distance_function = [](const Eigen::VectorXd& q1,
                                              const Eigen::VectorXd& q2) {
    return (q1 - q2).norm();
  };
  checker_ = std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  if (!plant.HasFrameNamed(options_.end_effector_frame)) {
    throw std::runtime_error("PlanningScene: no frame named '" +
                             options_.end_effector_frame + "'");
  }
}

PlanningScene::~PlanningScene() = default;

const drake::multibody::MultibodyPlant<double>& PlanningScene::plant() const {
  return checker_->plant();
}

std::vector<std::string> PlanningScene::joint_names() const {
  std::vector<std::string> names(num_positions());
  for (drake::multibody::JointIndex j : plant().GetJointIndices()) {
    const auto& joint = plant().get_joint(j);
    for (int k = 0; k < joint.num_positions(); ++k) {
      names[joint.position_start() + k] =
          joint.num_positions() == 1 ? joint.name()
                                     : joint.name() + "_" + std::to_string(k);
    }
  }
  return names;
}

bool PlanningScene::WithinLimits(const Eigen::VectorXd& q, double tol) const {
  return q.size() == num_positions() &&
         (q.array() >= lower_.array() - tol).all() &&
         (q.array() <= upper_.array() + tol).all();
}

bool PlanningScene::IsCollisionFree(const Eigen::VectorXd& q) const {
  return checker_->CheckConfigCollisionFree(q);
}

bool PlanningScene::IsEdgeCollisionFree(const Eigen::VectorXd& q1,
                                        const Eigen::VectorXd& q2) const {
  return checker_->CheckEdgeCollisionFree(q1, q2);
}

double PlanningScene::MinDistance(const Eigen::VectorXd& q,
                                  double influence_distance) const {
  const auto clearance = checker_->CalcRobotClearance(q, influence_distance);
  if (clearance.size() == 0) return influence_distance;
  return clearance.distances().minCoeff();
}

Eigen::VectorXd PlanningScene::SampleUniform(std::mt19937* rng) const {
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  Eigen::VectorXd q(num_positions());
  for (int i = 0; i < q.size(); ++i) {
    q[i] = lower_[i] + unit(*rng) * (upper_[i] - lower_[i]);
  }
  return q;
}

std::optional<Eigen::VectorXd> PlanningScene::SampleCollisionFree(
    std::mt19937* rng, int max_tries) const {
  for (int i = 0; i < max_tries; ++i) {
    Eigen::VectorXd q = SampleUniform(rng);
    if (IsCollisionFree(q)) return q;
  }
  return std::nullopt;
}

Eigen::VectorXd PlanningScene::Clamp(const Eigen::VectorXd& q) const {
  return q.cwiseMax(lower_).cwiseMin(upper_);
}

Eigen::Vector3d PlanningScene::EndEffectorPosition(
    const Eigen::VectorXd& q) const {
  const auto& plant_context = checker_->UpdatePositions(q);
  return plant()
      .GetFrameByName(options_.end_effector_frame)
      .CalcPoseInWorld(plant_context)
      .translation();
}

}  // namespace motion_planner
