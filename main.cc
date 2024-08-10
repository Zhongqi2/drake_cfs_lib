#include "src/RobotModel.h"

int main() 
{
  bool show_robot = false;
  RobotModel gp7_robot(show_robot);
  Eigen::VectorXd joint_position(6);
  joint_position << 0,0,0,0,0,0;
  SDF sign_distance_function = gp7_robot.cal_minSDF_and_gradient(joint_position);
  cout << "min_distance:" << sign_distance_function.min_distance << "\n" << endl;
  cout << "ddistance_dq:" << sign_distance_function.ddistance_dq << "\n" << endl;

  return 0;
}
