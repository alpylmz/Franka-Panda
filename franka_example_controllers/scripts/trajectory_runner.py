import roslib
import rospy
import actionlib
import moveit_commander
import time
import numpy as np
import rospkg
import json

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from franka_msgs.srv import HRI,HRIResponse

def connect_service(name):
    client = actionlib.SimpleActionClient(name,
                                            FollowJointTrajectoryAction)
    print(client.wait_for_server(timeout=rospy.Duration(1.0)))
    print("Connected to trajectory manager")
    return client

def switch_controller_mode(switch_controller, client, mode):
    # Mode 0: velocity, Mode 1: trajectory
    if mode == 0:
        ret = switch_controller(['joint_group_vel_controller'], 
                           ['vel_based_pos_traj_controller'], 2, False, 1)
    elif mode == 1:
        ret = switch_controller(['vel_based_pos_traj_controller'],
                            ['joint_group_vel_controller'], 2, False, 1)

    return ret

def handle_hri_service(req):
    
    goal = load_traj_from_json("traj1.json")
    resp = HRIResponse()
    resp.trajGoal = goal
    resp.success = True
    return resp

def load_traj_from_json(file_name):
    goal = FollowJointTrajectoryGoal()
    a = open(file_name, "r").read()
    f = json.loads(a)
    count = f["point_count"]
    for z in range(count):
        goal.trajectory.points.append(JointTrajectoryPoint())
        goal.trajectory.points[z].positions = f["points"][z]
        goal.trajectory.points[z].time_from_start = rospy.Duration(f["times_from_start"][z])
    
    goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)
    goal.trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6","panda_joint7"]
    return goal

def main():
    rospy.init_node("trajectory_controll")

    s = rospy.Service('hri_traj', HRI, handle_hri_service)


    global group
    group  = moveit_commander.MoveGroupCommander("panda_arm")
    
    current = group.get_current_joint_values()
    #print(group.get_joint_names())
    #exit()
    #client = connect_service("effort_joint_trajectory_controller/follow_joint_trajectory")
    client = connect_service("position_joint_trajectory_controller/follow_joint_trajectory")
    #pub = rospy.Publisher("effort_joint_trajectory_controller/command", Float64MultiArray, queue_size=10) ",
    #rospy.spin()

    while True:
        
        a = input("trajectory to run : ")

        goal = load_traj_from_json("traj1.json")
        
        client.send_goal(goal)
        print("the goal is sent")
        # Wait for up to 5 seconds for the motion to complete
        result = client.wait_for_result(rospy.Duration(100.0))
        print(result)
        print("-----------------------")


if __name__ == "__main__": main()
