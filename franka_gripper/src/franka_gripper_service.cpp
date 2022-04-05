#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include "ros/ros.h"
#include "franka_gripper/GripperCommand.h"

franka::Gripper* gripper;


bool handle_service_request(franka_gripper::GripperCommand::Request  &req,
                            franka_gripper::GripperCommand::Response &res)
{
    try {

        bool success;
        if(req.homing)
        { 
            gripper.homing();
        }

        if(req.want_to_pick)
        {
            franka::GripperState gripper_state = gripper.readOnce();
            if (gripper_state.max_width < req.width) {
                std::cout << "The stated width is out of limits." << std::endl;
                success = false;
            }

            if (!gripper.grasp(req.width, req.speed, req.force)) {
                std::cout << "Failed to grasp object." << std::endl;
                success = false;
            }
        }
        }
        else
        {
            std::cout << "Releasing Object" << std::endl;
            gripper.stop();
        }
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        success = false;
    }

    return success;
}


int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: ./grasp_object <franka-robot-ip> <do_need_to_home_first>" << std::endl;
    return -1;
  }

  try {
    gripper = new franka::Gripper(argv[1]);
    
    ros::init(argc, argv, "franka_custom_gripper_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("franka_custom_gripper_service", add);
    ROS_INFO("Ready to control the gripper.");
    ros::spin();

    std::stringstream ss(argv[2]);
    bool homing;
    if (!(ss >> homing)) {
      std::cerr << "<do_need_to_home_first> can be 0 or 1." << std::endl;
      return -1;
    }

    if (homing) {
      gripper.homing(); // home to reset the gripper in the start
    }


    } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
