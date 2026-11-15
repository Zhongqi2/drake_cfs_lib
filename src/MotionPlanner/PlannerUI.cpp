#include "PlannerUI.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <chrono>
#include <cstdlib>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <thread>

#include "drake/geometry/meshcat_params.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace motion_planner {

using drake::geometry::Rgba;

namespace {

constexpr char kSetStart[] = "Set START = sliders";
constexpr char kSetGoal[] = "Set GOAL = sliders";
constexpr char kRandomStart[] = "Random START";
constexpr char kRandomGoal[] = "Random GOAL";
constexpr char kShowStart[] = "Show START";
constexpr char kShowGoal[] = "Show GOAL";
constexpr char kReset[] = "Reset: same START & GOAL";
constexpr char kReplay[] = "Replay last motion";
constexpr char kCompare[] = "Compare all planners";
constexpr char kSpeed[] = "Playback speed";
constexpr char kPlanPrefix[] = "Plan: ";

const std::vector<std::pair<std::string, Rgba>>& Palette() {
  static const std::vector<std::pair<std::string, Rgba>> palette{
      {"orange", Rgba(1.0, 0.55, 0.0, 1.0)},  {"blue", Rgba(0.2, 0.45, 1.0, 1.0)},
      {"purple", Rgba(0.6, 0.3, 0.9, 1.0)},  {"cyan", Rgba(0.0, 0.8, 0.8, 1.0)},
      {"magenta", Rgba(0.95, 0.2, 0.6, 1.0)}, {"yellow", Rgba(0.9, 0.85, 0.1, 1.0)},
  };
  return palette;
}

std::string FormatQ(const Eigen::VectorXd& q) {
  std::string s = "[";
  char buf[32];
  for (int i = 0; i < q.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%s%.2f", i ? ", " : "", q[i]);
    s += buf;
  }
  return s + "]";
}

}  // namespace

std::string DetectHostIp() {
  // 1) Over SSH, SSH_CONNECTION = "<client ip> <client port> <server ip> <port>".
  if (const char* ssh = std::getenv("SSH_CONNECTION")) {
    std::istringstream in(ssh);
    std::string client_ip, client_port, server_ip;
    if (in >> client_ip >> client_port >> server_ip && !server_ip.empty()) {
      return server_ip;
    }
  }
  // 2) First IPv4 interface that is not loopback or a docker/bridge device.
  std::string found = "localhost";
  ifaddrs* list = nullptr;
  if (getifaddrs(&list) == 0) {
    for (ifaddrs* it = list; it != nullptr; it = it->ifa_next) {
      if (!it->ifa_addr || it->ifa_addr->sa_family != AF_INET) continue;
      const std::string name = it->ifa_name;
      if (name == "lo" || name.rfind("docker", 0) == 0 ||
          name.rfind("br-", 0) == 0 || name.rfind("veth", 0) == 0) {
        continue;
      }
      char buf[INET_ADDRSTRLEN];
      const auto* addr = reinterpret_cast<sockaddr_in*>(it->ifa_addr);
      if (inet_ntop(AF_INET, &addr->sin_addr, buf, sizeof(buf))) {
        found = buf;
        break;
      }
    }
    freeifaddrs(list);
  }
  return found;
}

PlannerUI::PlannerUI(const PlanningScene& scene,
                     std::vector<std::unique_ptr<MotionPlanner>> planners,
                     unsigned int seed, const std::string& host_ip)
    : scene_(scene),
      planners_(std::move(planners)),
      rng_(seed ? seed : std::random_device{}()) {
  // Visualization diagram: same model as the planning scene + Meshcat.
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant_ = &plant;
  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("project", scene_.options().package_dir);
  parser.AddModelsFromUrl(scene_.options().model_url);
  plant.Finalize();

  // Listen on all interfaces, and report the machine's real IP in the URL so
  // it can be opened from another computer (e.g. when working over SSH).
  drake::geometry::MeshcatParams meshcat_params;
  meshcat_params.host = "*";
  meshcat_params.web_url_pattern =
      "http://" + (host_ip.empty() ? DetectHostIp() : host_ip) + ":{port}";
  meshcat_ = std::make_shared<drake::geometry::Meshcat>(meshcat_params);
  drake::geometry::MeshcatVisualizerd::AddToBuilder(&builder, scene_graph,
                                                     meshcat_);
  diagram_ = builder.Build();
  context_ = diagram_->CreateDefaultContext();

  // Initial start (home pose if it is free) and a random goal.
  const Eigen::VectorXd home = scene_.Clamp(Eigen::VectorXd::Zero(scene_.num_positions()));
  start_ = scene_.IsCollisionFree(home)
               ? home
               : scene_.SampleCollisionFree(&rng_).value_or(home);
  goal_ = scene_.SampleCollisionFree(&rng_).value_or(start_);

  AddControls();
  WriteSliders(start_);
  ShowConfiguration(start_);
  DrawStartGoalMarkers();
  SetStatus("Ready: pick a planner");
}

PlannerUI::~PlannerUI() = default;

void PlannerUI::AddControls() {
  const auto names = scene_.joint_names();
  const auto& lo = scene_.lower_limits();
  const auto& hi = scene_.upper_limits();
  for (int i = 0; i < scene_.num_positions(); ++i) {
    joint_sliders_.push_back(names[i]);
    meshcat_->AddSlider(names[i], lo[i], hi[i], 0.01, 0.0);
  }
  meshcat_->AddSlider(kSpeed, 0.1, 3.0, 0.1, 1.0);
  for (const char* b : {kSetStart, kSetGoal, kRandomStart, kRandomGoal,
                        kShowStart, kShowGoal}) {
    meshcat_->AddButton(b);
  }
  for (const auto& planner : planners_) {
    meshcat_->AddButton(kPlanPrefix + planner->name());
  }
  meshcat_->AddButton(kReset);
  meshcat_->AddButton(kReplay);
  meshcat_->AddButton(kCompare);
}

bool PlannerUI::Clicked(const std::string& button) {
  const int clicks = meshcat_->GetButtonClicks(button);
  int& seen = clicks_seen_[button];
  if (clicks > seen) {
    seen = clicks;
    return true;
  }
  return false;
}

// Meshcat has no text widget, so the status line is shown as an (inert)
// button at the bottom of the control panel. It is also printed.
void PlannerUI::SetStatus(const std::string& text) {
  std::cout << "[UI] " << text << std::endl;
  const std::string label = "ⓘ " + text;
  if (label == status_button_) return;
  if (!status_button_.empty()) meshcat_->DeleteButton(status_button_);
  meshcat_->AddButton(label);
  status_button_ = label;
}

Eigen::VectorXd PlannerUI::ReadSliders() const {
  Eigen::VectorXd q(joint_sliders_.size());
  for (size_t i = 0; i < joint_sliders_.size(); ++i) {
    q[i] = meshcat_->GetSliderValue(joint_sliders_[i]);
  }
  return q;
}

void PlannerUI::WriteSliders(const Eigen::VectorXd& q) {
  for (size_t i = 0; i < joint_sliders_.size(); ++i) {
    meshcat_->SetSliderValue(joint_sliders_[i], q[i]);
  }
}

void PlannerUI::ShowConfiguration(const Eigen::VectorXd& q) {
  auto& plant_context = plant_->GetMyMutableContextFromRoot(context_.get());
  plant_->SetPositions(&plant_context, q);
  diagram_->ForcedPublish(*context_);
  shown_ = q;
}

void PlannerUI::DrawStartGoalMarkers() {
  using drake::math::RigidTransformd;
  meshcat_->SetObject("/planner/start", drake::geometry::Sphere(0.03),
                      Rgba(0.1, 0.85, 0.2, 0.9));
  meshcat_->SetTransform("/planner/start",
                         RigidTransformd(scene_.EndEffectorPosition(start_)));
  meshcat_->SetObject("/planner/goal", drake::geometry::Sphere(0.03),
                      Rgba(0.9, 0.1, 0.1, 0.9));
  meshcat_->SetTransform("/planner/goal",
                         RigidTransformd(scene_.EndEffectorPosition(goal_)));
}

void PlannerUI::DrawEndEffectorPath(const std::string& path,
                                    const Trajectory<double>& traj,
                                    const Rgba& color) {
  const Eigen::MatrixXd q = SampleTrajectory(traj, 0.02);
  Eigen::Matrix3Xd points(3, q.cols());
  for (int i = 0; i < q.cols(); ++i) {
    points.col(i) = scene_.EndEffectorPosition(q.col(i));
  }
  meshcat_->SetLine(path, points, 4.0, color);
}

void PlannerUI::SetStart(const Eigen::VectorXd& q) {
  start_ = q;
  DrawStartGoalMarkers();
  SetStatus(std::string("START set") +
            (scene_.IsCollisionFree(q) ? "" : " (IN COLLISION)") + " " + FormatQ(q));
}

void PlannerUI::SetGoal(const Eigen::VectorXd& q) {
  goal_ = q;
  DrawStartGoalMarkers();
  SetStatus(std::string("GOAL set") +
            (scene_.IsCollisionFree(q) ? "" : " (IN COLLISION)") + " " + FormatQ(q));
}

void PlannerUI::Animate(const Trajectory<double>& traj) {
  const double speed = std::max(0.1, meshcat_->GetSliderValue(kSpeed));
  const auto wall_start = std::chrono::steady_clock::now();
  for (;;) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start)
            .count();
    const double t = std::min(traj.start_time() + speed * elapsed, traj.end_time());
    ShowConfiguration(traj.value(t).col(0));
    if (t >= traj.end_time()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  WriteSliders(shown_);
}

bool PlannerUI::StartEqualsGoal() {
  if ((start_ - goal_).norm() > 1e-3) return false;
  SetStatus("START equals GOAL: pick a new GOAL first");
  return true;
}

void PlannerUI::PlanAndExecute(size_t index) {
  if (StartEqualsGoal()) return;
  const MotionPlanner& planner = *planners_[index];
  SetStatus("Planning with " + planner.name() + " ...");
  meshcat_->Delete("/planner/compare");
  meshcat_->Delete("/planner/ee_path");

  PlanningRequest request;
  request.start = start_;
  request.goal = goal_;
  request.seed = rng_();
  const PlanningResult result = planner.Plan(request);
  std::cout << result.Summary() << std::endl;

  if (!result.trajectory) {
    SetStatus(planner.name() + " FAILED: " + result.message);
    return;
  }
  last_trajectory_ = result.trajectory;
  DrawEndEffectorPath("/planner/ee_path", *result.trajectory,
                      Palette()[index % Palette().size()].second);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%s: %s, plan %.2fs, motion %.2fs, %s",
                planner.name().c_str(), result.success ? "OK" : "FAILED",
                result.planning_time, result.stats.duration,
                result.stats.collision_free ? "collision-free"
                                            : "COLLIDES (shown anyway)");
  SetStatus(buf);
  Animate(*result.trajectory);

  // The robot now rests at the goal: chain it as the next start.
  last_start_ = request.start;
  last_goal_ = request.goal;
  start_ = result.trajectory->value(result.trajectory->end_time()).col(0);
  DrawStartGoalMarkers();
  std::cout << "[UI] robot at goal; it is now the START for the next plan "
               "(click \"" << kReset << "\" to retry the same query)\n";
}

void PlannerUI::ResetToLastQuery() {
  if (last_start_.size() == 0) {
    SetStatus("Nothing to reset yet: run a planner first");
    return;
  }
  start_ = last_start_;
  goal_ = last_goal_;
  WriteSliders(start_);
  ShowConfiguration(start_);
  DrawStartGoalMarkers();
  SetStatus("Reset: robot back at START, same GOAL. Try another planner");
}

void PlannerUI::CompareAll() {
  if (StartEqualsGoal()) return;
  SetStatus("Comparing all planners ...");
  meshcat_->Delete("/planner/ee_path");
  meshcat_->Delete("/planner/compare");
  PlanningRequest request;
  request.start = start_;
  request.goal = goal_;
  request.seed = rng_();

  std::printf("\n%-12s %-7s %8s %8s %10s %9s %10s %s\n", "planner", "result",
              "plan[s]", "move[s]", "joint[rad]", "EE[m]", "clear[m]", "path color");
  int num_ok = 0;
  for (size_t i = 0; i < planners_.size(); ++i) {
    const auto& [color_name, color] = Palette()[i % Palette().size()];
    const PlanningResult r = planners_[i]->Plan(request);
    const bool ok = r.success && r.stats.collision_free;
    num_ok += ok;
    if (r.trajectory) {
      DrawEndEffectorPath("/planner/compare/" + planners_[i]->name(),
                          *r.trajectory, color);
      std::printf("%-12s %-7s %8.2f %8.2f %10.2f %9.2f %10.3f %s\n",
                  r.planner_name.c_str(), ok ? "OK" : "COLLIDE", r.planning_time,
                  r.stats.duration, r.stats.joint_path_length,
                  r.stats.ee_path_length, r.stats.min_clearance, color_name.c_str());
    } else {
      std::printf("%-12s %-7s %8.2f   (%s)\n", r.planner_name.c_str(), "FAIL",
                  r.planning_time, r.message.c_str());
    }
  }
  std::printf("\n");
  SetStatus(std::to_string(num_ok) + "/" + std::to_string(planners_.size()) +
            " planners collision-free; paths drawn, table in console");
}

void PlannerUI::Run() {
  std::cout << "\n=== Motion planner UI: open " << meshcat_->web_url()
            << " in your browser ===\n"
            << "Planners:\n";
  for (const auto& p : planners_) {
    std::cout << "  - " << p->name() << ": " << p->description() << "\n";
  }
  std::cout << std::endl;

  for (;;) {
    if (Clicked(kSetStart)) SetStart(ReadSliders());
    if (Clicked(kSetGoal)) SetGoal(ReadSliders());
    if (Clicked(kRandomStart)) {
      if (auto q = scene_.SampleCollisionFree(&rng_)) {
        SetStart(*q);
        WriteSliders(*q);
        ShowConfiguration(*q);
      } else {
        SetStatus("Could not sample a collision-free START");
      }
    }
    if (Clicked(kRandomGoal)) {
      if (auto q = scene_.SampleCollisionFree(&rng_)) {
        SetGoal(*q);
      } else {
        SetStatus("Could not sample a collision-free GOAL");
      }
    }
    if (Clicked(kShowStart)) {
      WriteSliders(start_);
      ShowConfiguration(start_);
    }
    if (Clicked(kShowGoal)) {
      WriteSliders(goal_);
      ShowConfiguration(goal_);
    }
    for (size_t i = 0; i < planners_.size(); ++i) {
      if (Clicked(kPlanPrefix + planners_[i]->name())) PlanAndExecute(i);
    }
    if (Clicked(kReset)) ResetToLastQuery();
    if (Clicked(kReplay)) {
      if (last_trajectory_) {
        Animate(*last_trajectory_);
      } else {
        SetStatus("Nothing to replay yet");
      }
    }
    if (Clicked(kCompare)) CompareAll();

    // Follow the joint sliders when the user drags them.
    const Eigen::VectorXd q = ReadSliders();
    // Threshold > half a slider step, so slider rounding does not re-trigger.
    if ((q - shown_).cwiseAbs().maxCoeff() > 6e-3) {
      ShowConfiguration(q);
      const bool in_collision = !scene_.IsCollisionFree(q);
      if (in_collision != shown_in_collision_) {
        SetStatus(in_collision ? "Slider pose is IN COLLISION"
                               : "Slider pose is collision-free");
        shown_in_collision_ = in_collision;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

}  // namespace motion_planner
