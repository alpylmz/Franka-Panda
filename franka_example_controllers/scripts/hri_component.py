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

def handle_hri_service(req):
    ## how to decide which traj to send?=?=?=?=?
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


    rospy.spin()


if __name__ == "__main__": main()
