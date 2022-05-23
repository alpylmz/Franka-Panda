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
    '''pose = group.get_current_pose()
    pose_init = group.get_current_pose()
    pose_init.pose.position.x = 0.20710896497751272 + 0.1
    pose_init.pose.position.y = -0.10011557364036852 + 0.1
    pose_init.pose.position.z = 0.4912372491432301 + 0.1

    pose_init.pose.orientation.x = -0.9238165756924602
    pose_init.pose.orientation.y = 0.382825554904605
    pose_init.pose.orientation.z = -0.002594822112012797
    pose_init.pose.orientation.w = 0.0008921244561330177
    

    #print(pose.pose)
    list = []
    list.append(pose_init.pose)
    pose = group.get_current_pose()

    for x in range(1):
        pose = group.get_current_pose()
        pose.pose.position.x += 0.1
        #pose.pose.position.y += 0.1
        pose.pose.position.z += 0.1
        list.append(pose.pose)
    pose = group.get_current_pose()
    list.append(pose.pose)
    for x in range(1):
        pose = group.get_current_pose()
        pose.pose.position.x -= 0.1
        #pose.pose.position.y -= 0.1
        pose.pose.position.z -= 0.1
        list.append(pose.pose)
    
    
    list.append(pose_init.pose)
    
    print(list)
    path, fraction = group.compute_cartesian_path(waypoints=list, eef_step=0.01, jump_threshold=0.0)
    path = group.retime_trajectory(group.get_current_state(), path, 0.1)
    #print("path is", path)
    group.execute(path, wait=True)
    return'''

    kol = load_traj_from_json("standup.json")
    init_goal = FollowJointTrajectoryGoal()
    init_goal.trajectory.points.append(JointTrajectoryPoint())
    init_goal.trajectory.points[0].positions = kol.trajectory.points[0].positions
    init_goal.trajectory.points[0].time_from_start = rospy.Duration(5.0)
    init_goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)
    init_goal.trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6","panda_joint7"]
    client.send_goal(init_goal)
    result = client.wait_for_result(rospy.Duration(100.0))
    print(result)
    print("-----------------------")

    while True:
        
        a = input("trajectory to run : ")

        goal = load_traj_from_json("standup.json")
        
        client.send_goal(goal)
        print("the goal is sent")
        # Wait for up to 5 seconds for the motion to complete
        result = client.wait_for_result(rospy.Duration(100.0))
        print(result)
        print("-----------------------")


if __name__ == "__main__": main()
