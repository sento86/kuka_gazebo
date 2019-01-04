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
from sensor_msgs.msg import (
    Joy,
    JointState
)
from pyquaternion import Quaternion

#from pid import PID
#from meld.misc import position_menu_under_widget

class KR6control:
    def __init__(self):

        self.ns='kr6'
        self.pubTrajectory = rospy.Publisher("/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.pubTrajectoryNS = rospy.Publisher(self.ns+"/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        self._cbCmdJoy = rospy.Subscriber("joy", Joy, self.callback_cmd_joy, queue_size=1)
        self._cbCmdJoint = rospy.Subscriber(self.ns+"/cmd_joint", JointState, self.callback_cmd_joint, queue_size=1)

        #self.x_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.y_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.z_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        
        self._cmd_joy = Joy
        self._cmd_joint = JointState
        #self.tip_pose = Pose
        #self.ft_sensor = WrenchStamped
        
        self.iter = 0
        
#         self.q1=0.0
#         self.q2=0.0
#         self.q3=0.0
#         self.q4=0.0
#         self.q5=0.0
#         self.q6=0.0
        self.q1=0.0
        self.q2=-1.57
        self.q3=1.57
        self.q4=0.0
        self.q5=-1.57
        self.q6=0.0
        
        self.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
        self.joint_positions = np.array([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])
        
        self.trajectory = JointTrajectory()
        #self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.joint_names = self.joint_names
        #self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.0,-1.57,1.57,-0.0,-1.57,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))
        self.trajectory.points.append(JointTrajectoryPoint(positions = self.joint_positions, time_from_start = rospy.Duration(0,500000000)))
        
    def callback_cmd_joy(self, data):
        self._cmd_joy = data
        #print(self._joy)
        
    def callback_cmd_joint(self, data):
        self._cmd_joint = data
        #print(self._cmd_joint)
        # Set angular velocities
        self.q1 = self._cmd_joint.velocity[0] # joint_a1
        self.q2 = self._cmd_joint.velocity[1] # joint_a2
        self.q3 = self._cmd_joint.velocity[2] # joint_a3
        self.q4 = self._cmd_joint.velocity[3] # joint_a4
        self.q5 = self._cmd_joint.velocity[4] # joint_a5
        self.q6 = self._cmd_joint.velocity[5] # joint_a6
        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_joint as true
        #if (self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
        if (math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
            self.new_cmd_joint = True
            self.record_pose = True

    def loop(self, fc = 100.0):
        
        rate = rospy.Rate(fc) # 100 Hz
        
        # While loop
        while not rospy.is_shutdown():
        #for i in range(0,300):
        
            self.iter+=1
 
#             self.q1=0.5 #0.5
#             self.q2=-1.4 #-1.4
#             self.q3=1.7 #2.0
#             self.q4=0.5 #0.5
#             self.q5=-1.3 #-1.3
#             self.q6=0.5 #0.5
        
            self.joint_positions = np.array([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])

            self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
            #self.trajectory.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
            #self.trajectory.points[0] = JointTrajectoryPoint(positions = np.array([j1,j2,j3,j4,j5,j6]), time_from_start = rospy.Duration(0,500000000))
            self.trajectory.points[0] = JointTrajectoryPoint(positions = self.joint_positions, time_from_start = rospy.Duration(0,500000000))
        
            self.pubTrajectory.publish(self.trajectory)
            self.pubTrajectoryNS.publish(self.trajectory)
            rate.sleep() # Sleeps for 1/rate sec
            
            #rospy.spin()

def main():
    rospy.init_node("kr6_control")

    kr6_control = KR6control()

    kr6_control.loop()

if __name__ == '__main__':
    main()
