#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray, GoalID
from time import sleep
from franka_msgs.srv import Picker
from franka_gripper.msg import GraspAction, GraspActionGoal, MoveAction
from franka_gripper.msg import GraspGoal, GraspEpsilon, MoveActionGoal, MoveGoal
import geometry_msgs
from std_msgs.msg import Float32
import actionlib
from franka_msgs.srv import SetPositionCommand
import copy

"""
sait_pose = [[-0.043824358700330635, -1.407510863404525, -0.09621197597605756, -2.560739132123672, -0.1060395849076424, 1.8124003873666128, 2.2862498113266243],\
    [-0.043833065425262094, -0.74194375147751, 0.7254064318174334, -2.500672390231455, -0.21748089766392, 2.0266434956365966, -2.5544709886535997], \
        [0.22393067168562034, -0.8255384451087148, -0.9542482062041979, -2.7442310036441735, -0.04799512968135342, 2.44511964003245, -2.862907029917516], \
            [0.35283721951016206, -0.04709378091719478, -0.3728447161703779, -2.069211993736133, 0.027793894768218714, 2.0743370803197214, -2.389936514051067]]
"""
"""
sait_pose = [[0.14300678089996954, -0.10571678531294118, -0.07464030598939565, -2.7972743033780665, -0.09443593322573465, 2.6184868192672726, 0.9441600122749915], \
    [0.20483648538589472, -0.5162024200757345, -0.08793194310179205, -2.832637485306168, -0.09051715415693913, 2.3851425502882315, 1.0068076068858305], \
        [0.17981217271666958, 0.05763151297527059, -0.10087284358557126, -2.42853943685302, -0.09175499143534237, 2.5060579707558346, 1.0189335042933623], \
            [0.1559882919372414, 0.15908839865739535, -0.10338794352820072, -2.43950342163454, -0.09145850948757048, 2.579980320899429, 1.0050701806278939], \
                [0.11066673615685915, -0.5184714858905757, -0.10175694096477053, -2.774070551286664, -0.09504724207849956, 2.6482053709030153, 0.9942121854126451]] 
"""

sait_pose = [[0.14300678089996954, -0.10571678531294118, -0.07464030598939565, -2.7972743033780665, -0.09443593322573465, 2.6184868192672726, 0.9441600122749915], \
    [0.20483648538589472, -0.5162024200757345, -0.08793194310179205, -2.832637485306168, -0.09051715415693913, 2.3851425502882315, 1.0068076068858305], \
        [0.17981217271666958, 0.05763151297527059, -0.10087284358557126, -2.42853943685302, -0.09175499143534237, 2.5060579707558346, 1.0189335042933623], \
            [-0.765061974650935, 0.18078561169013643, 0.7366162493602506, -2.545185200139096, -0.42422513886891605, 2.8533240747453994, 1.3186464768110846], \
                [-0.7496816215180514, -1.0585833804620857, 0.8488889531419989, -2.8082135482587254, -0.4299711964668297, 2.4062439342236264, 2.3150185949305695]]
goal_tolerance = 0.001 # actually I need to take it from MoveIt, but for now this is how it is

def isReachedGoal(current_pose, goal_pose, goal_tolerance):
    for curr, goal in zip(current_pose, goal_pose):
        if abs(curr - goal) > goal_tolerance:
            return False
    return True

goal_x = 0.0
goal_y = 0.0
goal_z = 0.0

def move_service(req):
    global goal_x, goal_y, goal_z
    goal_x += req.x
    goal_y += req.y
    goal_z += req.z
    return True


if __name__ == '__main__':
    
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    s1 = rospy.Service('/franka_go', SetPositionCommand, move_service)

    initial_pose = commander.get_current_joint_values()
    picker = rospy.ServiceProxy('/picker', Picker)
    movement_count = 0
    is_picking = False

    # to neutral pose
    commander.go(wait=True)
    initial_position = commander.get_current_pose()

    while True:
        try:
            print("goal_x: " + str(goal_x) + " goal_y: " + str(goal_y) + " goal_z: " + str(goal_z))
            pose = copy.deepcopy(initial_position)
            pose.pose.position.x += goal_x
            pose.pose.position.y += goal_y
            pose.pose.position.z += goal_z
            commander.set_pose_target(pose)
            #plan1 = commander.plan()

            commander.go(wait=True)
        except Exception as ex:
            print(ex)
            print("Failed to plan")
        
"""
if __name__ == '__main__':
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    initial_pose = commander.get_current_joint_values()
    picker = rospy.ServiceProxy('/picker', Picker)
    movement_count = 0
    is_picking = False
    for pose in sait_pose:
        print("sending command")
        pose = geometry_msgs.msg.Pose()
        pose.target_pose.orientation.w = 1.0
        pose.target_pose.position.x = 0.5
        pose.target_pose.position.y = 0.0
        pose.target_pose.position.z = 0.5
        commander.go(pose)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        print("Command sent!,", movement_count)
        
        while(not isReachedGoal(commander.get_current_joint_values(), pose, goal_tolerance)):
            continue
        
        if(movement_count == 0):
            picker(0.001, 0.5, 50)
            sleep(1)
            is_picking = True
        elif(movement_count == 3):
            picker(0.1, 0.5, 50)
            sleep(1)
            is_picking = False
            

        movement_count += 1
"""