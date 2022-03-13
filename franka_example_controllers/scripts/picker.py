#!/usr/bin/env python

import rospy
#from franka_gripper.action import *
from franka_gripper.msg import GraspAction, GraspActionGoal, MoveAction
from franka_gripper.msg import GraspGoal, GraspEpsilon, MoveActionGoal, MoveGoal
from franka_msgs.srv import Picker
from time import sleep, time
import actionlib
from actionlib_msgs.msg import GoalID

is_holding = False
width = 0.03
speed = 0.5
force = 70.0
goalID = 0

def Pick(req):
    global is_holding, width, speed, force
    if is_holding == False:
        width = req.width
        speed = req.speed
        force = req.force

    is_holding = not is_holding
    return True

if __name__ == '__main__':
    rospy.init_node('picker_picker')
    pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
    cancel = rospy.Publisher('/franka_gripper/grasp/cancel', GraspActionGoal, queue_size=10)
    pub2 = rospy.Publisher('franka_gripper/move/goal', MoveActionGoal, queue_size=10)
    
    serv = rospy.Service('/picker', Picker, Pick)
    curr = time()
    while(True):
        if is_holding:
            pub.publish(GraspActionGoal(goal=GraspGoal(width = 0.01, speed = 0.5, force=-10.0, epsilon=GraspEpsilon(inner = 0.0001, outer = 0.0001))))
            print("picking")
        else:
            pub2.publish(MoveActionGoal(goal = MoveGoal(width=0.03, speed = speed)))
            #pub.publish(GraspActionGoal(goal=GraspGoal(width = 0.04, speed = 0.5, force=-10.0, epsilon=GraspEpsilon(inner = 0.0001, outer = 0.0001))))
            print("not picking")
            pass