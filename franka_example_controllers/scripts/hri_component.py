#!/usr/bin/env python

import rospy
import json
import actionlib
from moveit_commander import MoveGroupCommander

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from franka_msgs.srv import HRI, HRIResponse, HRIRequest
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
import rospkg


aggresive_trajectory = FollowJointTrajectoryGoal()
breath_trajectory = FollowJointTrajectoryGoal()
hesitation_trajectory = FollowJointTrajectoryGoal()
lose_trajectory = FollowJointTrajectoryGoal()
mock_trajectory = FollowJointTrajectoryGoal()
nod_trajectory = FollowJointTrajectoryGoal()
positive_trajectory = FollowJointTrajectoryGoal()
salut_trajectory = FollowJointTrajectoryGoal()
think_trajectory = FollowJointTrajectoryGoal()
up_trajectory = FollowJointTrajectoryGoal()
win_trajectory = FollowJointTrajectoryGoal()



init_joints = [0.10167984932684593, -1.18724311909567, -0.09535411201373874, -2.180816145388399, -0.025952730392610403, 1.4625436514020722, 0.7860640899928077]

def connect_service(name):
    client = actionlib.SimpleActionClient(name,
                                            FollowJointTrajectoryAction)
    
    #client = actionlib.ActionClient(name, FollowJointTrajectoryAction)
    
    print(client.wait_for_server(timeout=rospy.Duration(2.0)))
    print("Connected to trajectory manager")
    return client

commander = MoveGroupCommander('panda_arm')

def handle_hri_service(req):


    move = req.move

    print(move)

    
    commander.set_joint_value_target(init_joints)
    plan1 = commander.plan()
    commander.go(wait=True)

    rospy.wait_for_service('controller_manager/load_controller')

    try:
        load = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        resp1 = load('effort_joint_trajectory_controller')
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.wait_for_service('controller_manager/switch_controller')

    try:
        switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        resp1 = switch(['effort_joint_trajectory_controller'], ['position_joint_trajectory_controller'], 1, True, 1)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return

    

    client = connect_service("effort_joint_trajectory_controller/follow_joint_trajectory")

    
    goal = breath_trajectory
    goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
    
    if move == "breath":
        goal = breath_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "win":
        goal = win_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True
    elif move == "lose":
        goal = lose_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True      
    elif move == "nod":
        goal = nod_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "salut":
        goal = salut_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "aggresive":
        goal = aggresive_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "hesitation":
        goal = hesitation_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True
    elif move == "think":
        goal = think_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True      
    elif move == "mock":
        goal = mock_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "up":
        goal = up_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    elif move == "positive":
        goal = positive_trajectory
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True 
    
    #client.cancel_all_goals()
    #client.stop_tracking_goal()
    #client.send_goal_and_wait(goal)
    #print(client.feedback_cb)
    print(goal.trajectory.header.stamp)
    client.send_goal(goal)
    #client.
    result = client.wait_for_result(rospy.Duration(30.0))
    print("------")
    #print(client.get_state())
    #client.cancel_goal()
    #client.cancel_all_goals()
    #print(result)
    stat = client.get_state()
    client.stop_tracking_goal()

    print(stat)

    while stat == 2:
        print("state is twoo")
        print(stat)
        goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(1)
        client.send_goal(goal)
        result = client.wait_for_result(rospy.Duration(30.0))
        stat = client.get_state()

    rospy.wait_for_service('controller_manager/switch_controller')

    try:
        switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        resp1 = switch(['position_joint_trajectory_controller'], ['effort_joint_trajectory_controller'], 2, True, 1)
        print(resp1)
    except rospy.ServiceException as e:
        print("SHUT DOWN THE SYSTEMS IMMIDIEATLY Service call failed: %s"%e)
    print("FINISHEDDDDD")
    return resp
    ## how to decide which traj to send?=?=?=?=?
    

def load_traj_from_json(file_name):
    goal = FollowJointTrajectoryGoal()
    a = open(file_name, "r").read()
    f = json.loads(a)
    count = f["point_count"]
    for z in range(count):
        goal.trajectory.points.append(JointTrajectoryPoint())
        goal.trajectory.points[z].positions = f["points"][z]
        goal.trajectory.points[z].time_from_start = rospy.Duration(f["times_from_start"][z])
    
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1)
    goal.trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6","panda_joint7"]
    return goal

def main():
    rospy.init_node("trajectory_controll")

    s = rospy.Service('hri_traj', HRI, handle_hri_service)

    
    global aggresive_trajectory
    global breath_trajectory
    global hesitation_trajectory
    global lose_trajectory
    global mock_trajectory
    global nod_trajectory
    global positive_trajectory
    global salut_trajectory
    global think_trajectory
    global up_trajectory
    global win_trajectory
    
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('franka_example_controllers') + "/scripts/"
    
    aggresive_trajectory = load_traj_from_json(base_path + "json_files/aggresive.json")
    breath_trajectory = load_traj_from_json(base_path + "json_files/breath2.json")
    hesitation_trajectory = load_traj_from_json(base_path + "json_files/hesitation.json")
    lose_trajectory = load_traj_from_json(base_path + "json_files/lose.json")
    mock_trajectory = load_traj_from_json(base_path + "json_files/mock.json")
    nod_trajectory = load_traj_from_json(base_path + "json_files/nod.json")
    positive_trajectory = load_traj_from_json(base_path + "json_files/positive.json")
    salut_trajectory = load_traj_from_json(base_path + "json_files/salut.json")
    think_trajectory = load_traj_from_json(base_path + "json_files/think.json")
    up_trajectory = load_traj_from_json(base_path + "json_files/up.json")
    win_trajectory = load_traj_from_json(base_path + "json_files/win.json")


    rospy.spin()


if __name__ == "__main__": main()
