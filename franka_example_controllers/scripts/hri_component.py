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


breath_trajectory = FollowJointTrajectoryGoal()
happy_trajectory = FollowJointTrajectoryGoal()
sad_trajectory = FollowJointTrajectoryGoal()

init_joints = [0.10167984932684593, -1.18724311909567, -0.09535411201373874, -2.180816145388399, -0.025952730392610403, 1.4625436514020722, 0.7860640899928077]

def connect_service(name):
    client = actionlib.SimpleActionClient(name,
                                            FollowJointTrajectoryAction)
    print(client.wait_for_server(timeout=rospy.Duration(1.0)))
    print("Connected to trajectory manager")
    return client

commander = MoveGroupCommander('panda_arm')

def handle_hri_service(req):

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

    game_state = req.game_status
    breath = req.breathing
    
    goal = breath_trajectory
    goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)

    if(breath):
        goal = breath_trajectory
        goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)
        resp = HRIResponse()
        resp.success = True       
    else:
        if game_state < 0.5:
            goal = sad_trajectory
            goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)
            resp = HRIResponse()
            resp.success = True        
        else:
            goal = happy_trajectory
            goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(1)
            resp = HRIResponse()
            resp.success = True

    client.send_goal(goal)
    result = client.wait_for_result(rospy.Duration(100.0))
    print(result)

    rospy.wait_for_service('controller_manager/switch_controller')

    try:
        switch = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        resp1 = switch(['position_joint_trajectory_controller'], ['effort_joint_trajectory_controller'], 2, True, 1)
        print(resp1)
    except rospy.ServiceException as e:
        print("SHUT DOWN THE SYSTEMS IMMIDIEATLY Service call failed: %s"%e)
    
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

    global breath_trajectory
    global happy_trajectory
    global sad_trajectory
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('franka_example_controllers') + "/scripts/"
    breath_trajectory = load_traj_from_json(base_path + "json_files/breath2.json")
    happy_trajectory = load_traj_from_json(base_path + "json_files/positive.json")
    sad_trajectory  = load_traj_from_json(base_path + "json_files/nod.json")

    rospy.spin()


if __name__ == "__main__": main()
