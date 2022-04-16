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
from franka_msgs.srv import SetJointPositionCommand
from franka_msgs.srv import SetOrientationCommand
import copy
from scipy.interpolate import interp1d

#from quaternion import Quaternion

GOAL_TOLERANCE = 0.001 # actually I need to take it from MoveIt, but for now this is how it is
ONLY_TAKE_POSITION = False


def isReachedGoal(current_pose, goal_pose, GOAL_TOLERANCE):
    for curr, goal in zip(current_pose, goal_pose):
        if abs(curr - goal) > GOAL_TOLERANCE:
            return False
    return True

goal_x = 0.0
goal_y = 0.0
goal_z = 0.0
goal_q_x = 0.0
goal_q_y = 0.0
goal_q_z = 0.0
goal_q_w = 0.0
is_there_a_goal = False
is_goal_relative = False
go_to_our_initial = False

is_goal_jointwise = False
goal_joint_1 = 0.0
goal_joint_2 = 0.0
goal_joint_3 = 0.0
goal_joint_4 = 0.0
goal_joint_5 = 0.0
goal_joint_6 = 0.0
goal_joint_7 = 0.0

# This function takes both current joint state and the goal joint positions
# Then, it interpolates the positions, and return a list of joint positions
def interpolate_joint_positions(current_joint_state, goal_joint_state):
    trajectory = []
    # I am dividing the path into 100 parts
    for i in range(100):
        # I am creating a list of joint positions
        joint_positions = []
        
        for j in range(7):
            # difference
            diff = goal_joint_state[j] - current_joint_state[j]
            # I am dividing the difference by 100
            diff = diff / 100
            # I am adding the current joint position to the difference
            joint_positions.append(current_joint_state[j] + diff * i)
        
        # I am appending the joint positions to the trajectory
        trajectory.append(joint_positions)

    return trajectory


def joint_move_service(req):
    global goal_joint_1, goal_joint_2, goal_joint_3, goal_joint_4, goal_joint_5, goal_joint_6, goal_joint_7, is_goal_jointwise, is_there_a_goal
    goal_joint_1 = req.joint_1
    goal_joint_2 = req.joint_2
    goal_joint_3 = req.joint_3
    goal_joint_4 = req.joint_4
    goal_joint_5 = req.joint_5
    goal_joint_6 = req.joint_6
    goal_joint_7 = req.joint_7
    is_goal_jointwise = True
    is_there_a_goal = True
    return True, ""

def move_service(req):
    global goal_x, goal_y, goal_z, is_there_a_goal, is_goal_relative, goal_q_x, goal_q_y, goal_q_z, goal_q_w, go_to_our_initial, is_goal_jointwise
    goal_x = req.x
    goal_y = req.y
    goal_z = req.z

    """ Now there is a bug in orientation
    goal_q_x = req.q_x
    goal_q_y = req.q_y
    goal_q_z = req.q_z
    goal_q_w = req.q_w
    """
    is_goal_relative = req.is_relative
    go_to_our_initial = req.go_to_init
    is_goal_jointwise = False

    is_there_a_goal = True
    return True, ""


if __name__ == '__main__':
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    s1 = rospy.Service('/franka_go', SetPositionCommand, move_service)
    s2 = rospy.Service('/franka_go_joint', SetJointPositionCommand, joint_move_service)

    if ONLY_TAKE_POSITION:
        while True:
            initial_pose = commander.get_current_joint_values()
            initial_position = commander.get_current_pose()
            print("Initial position: " + str(initial_position))
            print("Initial pose: " + str(initial_pose))
            sleep(1)

    initial_pose = commander.get_current_joint_values()
    picker = rospy.ServiceProxy('/picker', Picker)
    movement_count = 0
    is_picking = False

    # to neutral pose
    commander.go(wait=True)
    initial_position = commander.get_current_pose()
    print("get_active_joints: ", commander.get_active_joints())

    # ----------------------
    """
    Initial pose: 
    position: 
        x: 0.3282550835138732
        y: 0.05751661222418812
        z: 0.6330740276406968
    orientation: 
        x: -0.9226185998157332
        y: 0.3843317164009613
        z: -0.029732857849977316
        w: 0.013416713696730436
    """
    our_initial_pose = commander.get_current_pose()
    our_initial_pose.pose.position.x = 0.3282550835138732
    our_initial_pose.pose.position.y = 0.05751661222418812
    our_initial_pose.pose.position.z = 0.6330740276406968
    our_initial_pose.pose.orientation.x = -0.9226185998157332
    our_initial_pose.pose.orientation.y = 0.3843317164009613
    our_initial_pose.pose.orientation.z = -0.029732857849977316
    our_initial_pose.pose.orientation.w = 0.013416713696730436

    commander.set_pose_target(our_initial_pose)
    plan1 = commander.plan()

    commander.go(wait=True)




    # ----------------------


    while True:
        try:
            if not is_there_a_goal:
                sleep(0.1)
                continue
            if is_goal_jointwise:
                pass
            elif go_to_our_initial:
                commander.set_pose_target(our_initial_pose)
                plan1 = commander.plan()
                commander.go(wait=True)
                go_to_our_initial = False
                continue
            elif is_goal_relative:
                pose = commander.get_current_pose()
                pose.pose.position.x += goal_x
                pose.pose.position.y += goal_y
                pose.pose.position.z += goal_z
                pose.pose.orientation.x += goal_q_x
                pose.pose.orientation.y += goal_q_y
                pose.pose.orientation.z += goal_q_z
                pose.pose.orientation.w += goal_q_w
            else:
                pose = commander.get_current_pose()
                pose.pose.position.x = goal_x
                pose.pose.position.y = goal_y
                pose.pose.position.z = goal_z
            """
            pose.pose.orientation.x += or_x
            pose.pose.orientation.y += or_y
            pose.pose.orientation.z += or_z
            pose.pose.orientation.w += or_w
            """
            """
            commander.set_pose_target(pose)
            """
            """
            commander.set_position_target([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            """
            """
            commander.set_pose_target(pose)
            """
            """
            temp_pose = commander.get_current_pose()
            commander.set_workspace([min(temp_pose.pose.position.x, pose.pose.position.x) - 0.1, max(temp_pose.pose.position.x, pose.pose.position.x) + 0.1, \
                                    min(temp_pose.pose.position.y, pose.pose.position.y) - 0.1, max(temp_pose.pose.position.y, pose.pose.position.y) + 0.1, \
                                    min(temp_pose.pose.position.z, pose.pose.position.z) - 0.1, max(temp_pose.pose.position.z, pose.pose.position.z) + 0.1])
            commander.set_position_target([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            """
            """
            temp_pose = commander.get_current_pose()
            commander.set_workspace([min(temp_pose.pose.position.x, pose.pose.position.x) - 0.1, max(temp_pose.pose.position.x, pose.pose.position.x) + 0.1, \
                                    min(temp_pose.pose.position.y, pose.pose.position.y) - 0.1, max(temp_pose.pose.position.y, pose.pose.position.y) + 0.1, \
                                    min(temp_pose.pose.position.z, pose.pose.position.z) - 0.1, max(temp_pose.pose.position.z, pose.pose.position.z) + 0.1])
            commander.set_pose_target(pose)
            """

            """ NOT BAD
            temp_pose = commander.get_current_pose()

            commander.set_workspace([min(temp_pose.pose.position.x, pose.pose.position.x) - 0.01, max(temp_pose.pose.position.x, pose.pose.position.x) + 0.01, \
                                    min(temp_pose.pose.position.y, pose.pose.position.y) - 0.01, max(temp_pose.pose.position.y, pose.pose.position.y) + 0.01, \
                                    min(temp_pose.pose.position.z, pose.pose.position.z) - 0.01, max(temp_pose.pose.position.z, pose.pose.position.z) + 0.01])
            commander.set_pose_target(pose)
            """
            
            #path, fraction = commander.compute_cartesian_path(waypoints=[pose.pose], eef_step=0.01, jump_threshold=0.0)
            path, fraction = commander.compute_cartesian_path(waypoints=[pose.pose], eef_step=0.01, jump_threshold=0.0)
            
            path = commander.retime_trajectory(commander.get_current_state(), path, 0.1)
            from pprint import pprint
            pprint(path)
            commander.execute(path, wait=True)

            print("Moving to goal: ", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            #plan1 = commander.plan()

            #commander.go(wait=True)
            print("Reached goal")
            is_there_a_goal = False
            print(commander.get_current_pose())
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
        
        while(not isReachedGoal(commander.get_current_joint_values(), pose, GOAL_TOLERANCE)):
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
