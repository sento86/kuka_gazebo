#!/usr/bin/python

import numpy as np
import math
import time
import rospy
import tf
import message_filters

from geometry_msgs.msg import (
    Point,
    Quaternion,
    WrenchStamped,
    Pose,
    PoseStamped,
)
from std_msgs.msg import Header
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)
#from pyquaternion import Quaternion

#from pid import PID
#from meld.misc import position_menu_under_widget

class KR6control:
    def __init__(self):

        self.ns='kr6'
        self.pubTrajectory = rospy.Publisher("/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.pubTrajectoryNS = rospy.Publisher(self.ns+"/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        
        #self.grp_pose = Pose()
        #self.cmd_pose = Pose()
        #self.ft_fake = WrenchStamped()
        
        #self.x_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.y_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.z_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        
        self.iter = 0
        
        self.trajectory = JointTrajectory()
        self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
        #self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.0,-1.57,1.57,-0.0,-1.57,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))
        self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.5,-1.57,1.57,-0.0,-1.57,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))
        #self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.0,-1.6,1.0,-0.0,-0.9,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))

        rate = rospy.Rate(100) # 1 Hz
        # Do stuff, maybe in a while loop

        for i in range(0,100):
            #self.pubTrajectory.publish(self.trajectory)
            self.pubTrajectoryNS.publish(self.trajectory)
            self.iter+=1
            rate.sleep() # Sleeps for 1/rate sec
            print self.iter
            #rospy.spin()

def main():
    rospy.init_node("kr6_control")

    kr6_control = KR6control()

    #rospy.spin()

if __name__ == '__main__':
    main()
