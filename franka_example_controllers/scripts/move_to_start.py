#!/usr/bin/env python

from pickle import TRUE
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
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
from franka_msgs.srv import SetChessGoal
from franka_msgs.srv import SetTrajectoryCommand
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import RobotTrajectory
import copy
from scipy.interpolate import interp1d
import json
from functools import partial
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
import rospkg

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
goals = []
is_there_a_goal = False
is_goal_relative = False
go_to_our_initial = False
is_goal_trajectory = False
go_to_side_vision_init = False
change_orientation_for_picking = False

is_goal_jointwise = False
goal_joint_1 = 0.0
goal_joint_2 = 0.0
goal_joint_3 = 0.0
goal_joint_4 = 0.0
goal_joint_5 = 0.0
goal_joint_6 = 0.0
goal_joint_7 = 0.0

is_goal_chess_place = False
is_goal_reached = False
goal_trajectory = None

aim_place = None # will be string, as "a1", "d8"
chess_joints = None
rospack = rospkg.RosPack()
base_path = rospack.get_path('franka_example_controllers') + "/scripts/"
with open(base_path+"joint_positions.json", "r") as f:
    chess_joints = json.load(f)

PLAN_SCALER = 0.45



'''
a8
position: 
  x: 0.6507821949711964
  y: 0.19939440800864475
  z: 0.21188719633735942
  
h1
position: 
  x: 0.32620260107480376
  y: -0.10921545181078882
  z: 0.21058383151854868
d4
position: 
  x: 0.459721815394416
  y: 0.06781174084817974
  z: 0.20912632020429586

'''

'''
position: 
  x: 0.19539142069987844
  y: 0.032936914228738325
  z: 0.6607737371090258
orientation: 
  x: -0.9151174542820978
  y: 0.3666390724514755
  z: -0.14648440132947507
  w: 0.08171998279074418

position: 
  x: 0.2488452570021335
  y: 0.03153220737061438
  z: 0.637208593186331
orientation: 
  x: -0.9221565634364416
  y: 0.3753940224220725
  z: -0.07703848385719647
  w: 0.05264667554417027

position: 
  x: 0.336669021224307
  y: 0.03530110281921283
  z: 0.6003148768897769
orientation: 
  x: -0.9237954028187488
  y: 0.38091077977297527
  z: -0.023952300585639404
  w: 0.030582983509536376
'''

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

def chess_move_service(req):
    global aim_place, is_goal_chess_place, is_there_a_goal, is_goal_jointwise, is_goal_trajectory
    aim_place = req.chess_place
    is_goal_chess_place = True
    is_there_a_goal = True
    is_goal_jointwise = False
    is_goal_trajectory = False
    return True, ""


def joint_move_service(req):
    global goal_joint_1, goal_joint_2, goal_joint_3, goal_joint_4, goal_joint_5, goal_joint_6, goal_joint_7, is_goal_jointwise, is_there_a_goal, is_goal_chess_place, is_goal_trajectory
    goal_joint_1 = req.joint_1
    goal_joint_2 = req.joint_2
    goal_joint_3 = req.joint_3
    goal_joint_4 = req.joint_4
    goal_joint_5 = req.joint_5
    goal_joint_6 = req.joint_6
    goal_joint_7 = req.joint_7
    is_goal_jointwise = True
    is_there_a_goal = True
    is_goal_chess_place = False
    is_goal_trajectory = False
    return True, ""

def move_service(req):
    global goal_x, goal_y, goal_z, is_there_a_goal, is_goal_relative, goal_q_x, goal_q_y, goal_q_z, goal_q_w, go_to_our_initial, is_goal_jointwise, is_goal_chess_place, is_goal_reached, is_goal_trajectory, go_to_side_vision_init, change_orientation_for_picking, goals
    goal_x = req.x
    goal_y = req.y
    goal_z = req.z

    """ Now there is a bug in orientation
    goal_q_x = req.q_x
    goal_q_y = req.q_y
    goal_q_z = req.q_z
    goal_q_w = req.q_w
    """
    goal_q_x = req.q_x
    goal_q_y = req.q_y
    goal_q_z = req.q_z
    goal_q_w = req.q_w
    
    is_goal_relative = req.is_relative
    go_to_our_initial = req.go_to_init
    go_to_side_vision_init = req.go_to_side_vision_init
    change_orientation_for_picking = req.change_orientation_for_picking
    
    is_goal_jointwise = False
    is_goal_chess_place = False
    is_goal_trajectory = False
    
    # if we are executing now, and we do not have other goals
    if (req.execute_now and len(goals) == 0) or change_orientation_for_picking:
        is_there_a_goal = True

        # wait!
        is_goal_reached = False
        # I know this is terrible, will decide and complete action client asap
        # I am just not sure if action client is really a good idea
        while not is_goal_reached:
            print("waiting for the movement!")
            sleep(0.5)
        return True, ""
    
    rospy.logerr("added one goal!")
    goals.append([goal_x, goal_y, goal_z, goal_q_x, goal_q_y, goal_q_z, goal_q_w])
    if req.execute_now:
        is_there_a_goal = True
        is_goal_reached = False
        while not is_goal_reached:
            print("waiting for the movement!")
            sleep(0.5)

    else:
        is_there_a_goal = False
    return True, ""


def trajectory_service(req):
    global goal_x, goal_y, goal_z, is_there_a_goal, is_goal_relative, goal_q_x, goal_q_y, goal_q_z, goal_q_w, go_to_our_initial, is_goal_jointwise, is_goal_chess_place, is_goal_reached, is_goal_trajectory, goal_trajectory

    goal_trajectory = req.trajGoal
    
    is_goal_relative = False
    is_goal_jointwise = False
    is_goal_chess_place = False
    is_goal_reached = False

    is_goal_trajectory = True
    is_there_a_goal = True
    
    return True


def addObstacles():
    # I am going to put these obstacles by assuming that the starting position will be init position!
    scene = PlanningSceneInterface(synchronous=True) # It will NOT WORK without synchronous=True, check https://github.com/ros-planning/moveit/issues/2623

    box_name = "left_obstacle"
    collision_object_left = geometry_msgs.msg.PoseStamped()
    collision_object_left.header.frame_id = "panda_link0"
    collision_object_left.pose.position.y = -0.6
    collision_object_left.pose.orientation.w = 1.0
    scene.add_box(box_name, collision_object_left, size=(10, 0.1, 10))

    box_name = "right_obstacle"
    collision_object_left = geometry_msgs.msg.PoseStamped()
    collision_object_left.header.frame_id = "panda_link0"
    collision_object_left.pose.position.y = 0.6
    collision_object_left.pose.orientation.w = 1.0
    scene.add_box(box_name, collision_object_left, size=(10, 0.1, 10))

    box_name = "back_obstacle"
    collision_object_left = geometry_msgs.msg.PoseStamped()
    collision_object_left.header.frame_id = "panda_link0"
    collision_object_left.pose.position.x = -0.50
    collision_object_left.pose.orientation.w = 1.0
    scene.add_box(box_name, collision_object_left, size=(0.1, 10, 10))

    box_name = "low_obstacle"
    collision_object_left = geometry_msgs.msg.PoseStamped()
    collision_object_left.header.frame_id = "panda_link0"
    collision_object_left.pose.position.z = -0.2
    collision_object_left.pose.orientation.w = 1.0
    scene.add_box(box_name, collision_object_left, size=(10, 10, 0.1))

    """
    ## attach an object to gripper
    object_name = "gripper_obstacle"
    object_pose = geometry_msgs.msg.PoseStamped()
    eef_link = commander.get_end_effector_link()
    grasping_group = "hand"

    box_name = "gripper_obstacle"
    collision_object_left = geometry_msgs.msg.PoseStamped()
    collision_object_left.header.frame_id = eef_link
    collision_object_left.pose.position.z = 0.075
    collision_object_left.pose.orientation.w = 1.0
    scene.add_box(box_name, collision_object_left, size=(0.05, 0.05, 0.15))

    object_pose.header.frame_id = eef_link
    scene.attach_box(eef_link, object_name, touch_links=[eef_link])
    """


if __name__ == '__main__':
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    commander.set_named_target('ready')
    commander.set_goal_tolerance(GOAL_TOLERANCE)

    s1 = rospy.Service('/franka_go', SetPositionCommand, move_service)
    s2 = rospy.Service('/franka_go_joint', SetJointPositionCommand, joint_move_service)
    s3 = rospy.Service('/franka_go_chess', SetChessGoal, chess_move_service)
    s4 = rospy.Service('/franka_trajectory', SetTrajectoryCommand, trajectory_service)

    trajectory_client = actionlib.SimpleActionClient('/position_joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    trajectory_client.wait_for_server()
    rospy.loginfo("Connected to trajectory action server")

    #move_action = actionlib.SimpleActionServer('/franka_go_action', MoveToPositionAction, execute_cb=partial(move_action, commander = commander), auto_start=True)

    addObstacles()

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
    #commander.go(wait=True)
    #initial_position = commander.get_current_pose()
    #print("get_active_joints: ", commander.get_active_joints())

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


    #commander.set_pose_target(our_initial_pose)
    #plan1 = commander.plan()

    #commander.go(wait=True)is_goal_jointwise
    #initial_joints = [0.8807196933525263, -0.8014431536310861, -0.42209067821502666, -2.1911595741634415, -0.33243660358600874, 1.4980820045918766, 1.323550466756026]
    #initial_joints = [0.1525394261890009, -0.08124570311546929, -0.07294265516092496, -1.5018837461405505, 0.006697758157634073, 1.4245549732844034, 0.878600022062846]
    #initial_joints = [0.15322086521630415, -0.11260720413400414, -0.07933782860502608, -1.1466092779796002, 0.016516494863563113, 1.0695718866963613, 0.8452164789947625]
    initial_joints = [0.11034795084633617, 0.058631561177854286, 0.005040856288292234, -1.0416756229767974, 0.016914562720391487, 1.1504808479944866, 0.8856570975581803]


    #[-0.3426358018005103, -0.7527097266085973, 0.3211428673806972, -2.1997036707603113, 0.22632897217768974, 1.5394524623788894, 0.7070815159243338]
    #initial_joints = [0.19629807516847006, -0.7055285141228648, 0.010857086453996393, -2.2712353889079404, 0.032713600012991166, 1.5440953356424967, 0.9812845707889041]
    hri_init_joint = [-0.00015594268319585325, -0.7856254591349376, 2.4569001943142155e-05, -2.3559519536390043, 8.081014387428809e-06, 1.5717405031367848, 0.7854055945854093]
    side_vision_joints = [0.10167984932684593, -1.18724311909567, -0.09535411201373874, -2.180816145388399, -0.025952730392610403, 1.4625436514020722, 0.7860640899928077]

    #joint_taken_by_hand = [0.00616382540145848, -0.7852030825311441, 0.03525380717565255, -2.4319639494758376, 0.046750732623868516, 1.670268750106765, 0.847491638365995]
    joint_taken_by_hand = [0.1016070718527865, -0.8463960386755712, -0.026082241943614377, -2.2065270135148762, -0.0218426144487328, 1.3669907948380446, 0.8825968448362962]


    
    # ----------------------


    while True:
        try:
            pose = None
            waypoints = []
            if not is_there_a_goal:
                sleep(0.1)
                continue
            if change_orientation_for_picking:
                rospy.logerr("Going to side vision init")
                commander.set_joint_value_target(joint_taken_by_hand)
                plan1 = commander.plan()
                path2 = commander.retime_trajectory(commander.get_current_state(), plan1[1], PLAN_SCALER)
                commander.execute(path2, wait=True)
                rospy.logerr("PLAN IS EXECUTED!!!!!!")
                go_to_side_vision_init = False
                change_orientation_for_picking = False
                is_there_a_goal = False # be careful
                is_goal_reached = True
                continue
            elif len(goals) != 0:
                pose = commander.get_current_pose() # actually we do not need this, but it solves a bug in the last lines of the loop
                waypoints.append(commander.get_current_pose().pose)
                rospy.loginfo("got the first append!")
                for waypoint in goals:
                    curr_pose = commander.get_current_pose().pose
                    curr_pose.position.x = waypoint[0]
                    curr_pose.position.y = waypoint[1]
                    curr_pose.position.z = waypoint[2]
                    rospy.loginfo("got another append!")
                    
                    waypoints.append(curr_pose)
                goals = []
            elif go_to_side_vision_init:
                rospy.logerr("Going to side vision init")
                commander.set_joint_value_target(side_vision_joints)
                plan1 = commander.plan()
                path2 = commander.retime_trajectory(commander.get_current_state(), plan1[1], PLAN_SCALER)
                commander.execute(path2, wait=True)
                #commander.go(wait=True)
                go_to_side_vision_init = False
                is_there_a_goal = False
                is_goal_reached = True
                continue
            elif is_goal_trajectory:
                rospy.logerr("Going to trajectory")
                """
                # first go to init position
                rospy.logerr("going to hri init")
                commander.set_joint_value_target(hri_init_joint)
                plan = commander.plan()
                rospy.logerr("planning")
                commander.go(wait=True)
                rospy.loginfo("Trajectory goal")
                rospy.loginfo(goal_trajectory)
                rospy.loginfo("start?")
                traj = RobotTrajectory()
                traj.joint_trajectory = goal_trajectory.trajectory
                commander.execute(traj, wait=True)
                is_goal_reached = True
                #trajectory_client.send_goal(goal_trajectory)
                """
                continue
            elif is_goal_jointwise:
                rospy.logerr("Going to jointwise")
                pass
            elif go_to_our_initial:
                rospy.logerr("Going to our initial")
                print("Going to our initial pose")
                commander.set_joint_value_target(initial_joints)
                plan1 = commander.plan()
                path2 = commander.retime_trajectory(commander.get_current_state(), plan1[1], PLAN_SCALER)
                commander.execute(path2, wait=True)
                go_to_our_initial = False
                is_there_a_goal = False
                is_goal_reached = True

                continue
            elif is_goal_chess_place:
                rospy.logerr("Going to chess place")
                print("Going to chess place" + aim_place)
                try:
                    goal_joints = chess_joints[aim_place]
                    print("Goal joints: " + str(goal_joints))
                except:
                    print("There is no such place")
                    is_there_a_goal = False
                    is_goal_chess_place = False
                    continue
                try:
                    print("current joints are " + str(commander.get_current_joint_values()))
                    commander.set_joint_value_target(goal_joints[:7])
                    plan1 = commander.plan()
                    print("Planned and going!")
                    from pprint import pprint
                    rospy.logerr("type is")
                    rospy.logerr(type(plan1))
                    pprint(plan1)
                    rospy.logerr(plan1)
                    commander.go(wait=True)
                except:
                    print("Failed to execute plan...")
                is_there_a_goal = False
                is_goal_chess_place = False    
                continue
            elif is_goal_relative:
                rospy.logerr("Going to relative")
                pose = commander.get_current_pose()
                pose.pose.position.x += goal_x
                pose.pose.position.y += goal_y
                pose.pose.position.z += goal_z
                pose.pose.orientation.x += goal_q_x
                pose.pose.orientation.y += goal_q_y
                pose.pose.orientation.z += goal_q_z
                pose.pose.orientation.w += goal_q_w
            else:
                rospy.loginfo("goal is " + str(goal_x) + " " + str(goal_y) + " " + str(goal_z) + " " + str(goal_q_x) + " " + str(goal_q_y) + " " + str(goal_q_z) + " " + str(goal_q_w))
                rospy.loginfo("current pose is " + str(commander.get_current_pose()))
                rospy.logerr("Going to absolute")
                pose = commander.get_current_pose()
                pose.pose.position.x = goal_x
                pose.pose.position.y = goal_y
                pose.pose.position.z = goal_z

            if len(waypoints) != 0:
                rospy.loginfo("sending the waypoints!")
                path, fraction = commander.compute_cartesian_path(waypoints=waypoints, eef_step = 0.01, jump_threshold = 0.0)
                #path = commander.retime_trajectory(commander.get_current_state(), path, 1.0)
            else:  
                rospy.loginfo("sending the single waypoint!")
                #path, fraction = commander.compute_cartesian_path(waypoints=[pose.pose], eef_step=0.01, jump_threshold=0.0)
                path, fraction = commander.compute_cartesian_path(waypoints=[commander.get_current_pose().pose, pose.pose], eef_step=0.001, jump_threshold=0.0)
                #path, fraction = commander.compute_cartesian_path(waypoints=[pose.pose], eef_step=0.01, jump_threshold=0.0)
            
            path = commander.retime_trajectory(commander.get_current_state(), path, 2.5)
            rospy.loginfo("Moving to goal: %f, %f, %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            commander.execute(path, wait=True)

            #plan1 = commander.plan()

            #commander.go(wait=True)
            rospy.loginfo("Reached goal")
            is_there_a_goal = False
            is_goal_reached = True
        except Exception as ex:
            rospy.logerr(ex)
            rospy.logerr("Failed to move to goal")
            is_there_a_goal = False
            is_goal_reached = True
