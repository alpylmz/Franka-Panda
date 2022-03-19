// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka_msgs/SetPositionCommand.h>
#include <std_msgs/Float32.h>

// the position goals
float ___x = 0.0;
float ___y = 0.0;
float ___z = 0.5;
double prev_pos_z = 0.0;
double position_step = 0.0000001;
double incremental_z = 0.0;

double move_step = 0.001;

bool positionServiceCallback(franka_msgs::SetPositionCommand::Request &req,
                             franka_msgs::SetPositionCommand::Response &res) {
  std::cout << "Position Service Callback" << std::endl;
  //std::cout << "Request: " << req.position.x << " " << req.position.y << " " << req.position.z << std::endl;
  std::cout << "Response: " << res.success << std::endl;
  ___x += req.x;
  ___y += req.y;
  ___z += req.z;
  res.success = true;
  return true;
}

ros::Publisher publisher_x;
ros::ServiceServer service_position_service;

namespace franka_example_controllers {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  // start the position service
  service_position_service = node_handle.advertiseService("/franka_position_service", positionServiceCallback);

  // start a publisher that gives ___x
  publisher_x = node_handle.advertise<std_msgs::Float32>("/franka_position_x", 1);


  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}


/*void CartesianPoseExampleController::update(const ros::Time&,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  
  std::array<double, 16> new_pose = initial_pose_;
  if(abs(incremental_z - ___z) > position_step){
    if(incremental_z < ___z){
      incremental_z += position_step;
    }
    else{
      incremental_z -= position_step;
    }
  }
  
  new_pose[12] -= ___x;
  new_pose[13] -= ___y;
  new_pose[14] -= incremental_z;
  //std::cout << "z: " << incremental_z << std::endl;
  //std::cout << "New Pose: " << new_pose[0] << " " << new_pose[1] << " " << new_pose[2] << " " << new_pose[3] << " " << new_pose[4] << " " << new_pose[5] << " " << new_pose[6] << " " << new_pose[7] << " " << new_pose[8] << " " << new_pose[9] << " " << new_pose[10] << " " << new_pose[11] << " " << new_pose[12] << " " << new_pose[13] << " " << new_pose[14] << " " << new_pose[15] << std::endl;
  //std_msgs::Float32 msg;
  //msg.data = ___x;
  //publisher_x.publish(msg);
  cartesian_pose_handle_->setCommand(new_pose);

}*/


double CartesianPoseExampleController::calculateMovementTowards(double current, double goal, ros::Duration elapsed_time)
{
  if(abs(current - goal) < 0.001)
  {
    return current;
  }

  if( abs(goal - current) >  move_step)
  {
    if(current < goal)
      return current + move_step;
    if(current > goal)
      return current - move_step;
  }
  else
  {
    return goal;
  }

  return current;
}

void CartesianPoseExampleController::update(const ros::Time&,
                                            const ros::Duration& period) {

  elapsed_time_ += period;

  double task_x = 0.2;
  double task_y = 0.0;
  double task_z = 0.0;

  std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  std::array<double, 16> new_pose = initial_pose_;

  
  new_pose[12] += calculateMovementTowards(current_pose[12], task_x+initial_pose_[12], elapsed_time_);
  new_pose[13] += calculateMovementTowards(current_pose[13], task_y+initial_pose_[13], elapsed_time_);
  new_pose[14] += calculateMovementTowards(current_pose[14], task_z+initial_pose_[14], elapsed_time_);
  
  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPoseExampleController::update2(const ros::Time&,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] -= delta_x;
  new_pose[14] -= delta_z;
  std::cout << "New Pose: " << new_pose[0] << " " << new_pose[1] << " " << new_pose[2] << " " << new_pose[3] << " " << new_pose[4] << " " << new_pose[5] << " " << new_pose[6] << " " << new_pose[7] << " " << new_pose[8] << " " << new_pose[9] << " " << new_pose[10] << " " << new_pose[11] << " " << new_pose[12] << " " << new_pose[13] << " " << new_pose[14] << " " << new_pose[15] << std::endl;
  cartesian_pose_handle_->setCommand(new_pose);
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
