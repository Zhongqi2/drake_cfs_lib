#include "RobotVisualizer.h"
#include "KinematicTrajOpt.h"
#include <thread>
Eigen::VectorXd generateRandomJointTarget(const Eigen::VectorXd& lower_limits, const Eigen::VectorXd& upper_limits) {
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::VectorXd goal_position(lower_limits.size());

    for (int i = 0; i < lower_limits.size(); ++i) {
        std::uniform_real_distribution<double> dis(lower_limits[i], upper_limits[i]);
        goal_position[i] = dis(gen);
    }
    return goal_position;
}

int main() 
{
  // construct the robot model
  const std::string robot_model_url = "package://project/script/environment_description/robot_environment.dmd.yaml";
  RobotVisualizer gp7_robot(robot_model_url);
  KinematicTrajOpt kine_traj_opt(robot_model_url);
  
  // show robot
  // gp7_robot.show_robot();

  Eigen::VectorXd start_position(7),goal_position(7);
  start_position << 0,0,0,0,0,0,0;
  goal_position << 0,0,0,0,0,0,0;
  TrajOptResult traj_opt_result;

  Eigen::VectorXd PositionLowerLimits = gp7_robot.plant->GetPositionLowerLimits();
  Eigen::VectorXd PositionUpperLimits = gp7_robot.plant->GetPositionUpperLimits();
  
  int count = 0;
  int max_times = 100;
  while(count <=max_times)
  {
    // Generate a random goal position
    goal_position = generateRandomJointTarget(PositionLowerLimits, PositionUpperLimits);
    traj_opt_result = kine_traj_opt.solve_trajectory(start_position,goal_position);
    if(traj_opt_result.is_success == true)
    {
      start_position = goal_position;
      gp7_robot.show_robot_trajectory(traj_opt_result.trajectory);
      count++;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  while (1)
  {

  }
  return 0;
}
  // Eigen::VectorXd joint_position(7);
  // joint_position << 1,1,1,1,1,1,1;
  // SDF sign_distance_function = kine_traj_opt.cal_minSDF_and_gradient(joint_position);
  // cout << "min_distance:" << sign_distance_function.min_distance << "\n" << endl;