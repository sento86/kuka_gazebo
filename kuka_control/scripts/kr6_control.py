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

from pid import PID
#from meld.misc import position_menu_under_widget

from kuka_pykdl import kuka_kinematics
from autobahn.twisted.util import sleep
from geometry_msgs.msg._TwistStamped import TwistStamped

class KR6control:
    def __init__(self, fc = 100.0, ns = 'kr6'):
        
        self.fc = fc
        self.rate = rospy.Rate(self.fc) # 100 Hz
        self.Tc = 1.0/self.fc

        self.ns = ns
        self.pubTrajectory_ = rospy.Publisher("/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.pubTrajectory = rospy.Publisher(self.ns+"/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        
        self._cbJointState = rospy.Subscriber(self.ns+"/joint_states", JointState, self.callback_joint_states, queue_size=1)
        self._cbCmdJoy = rospy.Subscriber("joy", Joy, self.callback_cmd_joy, queue_size=1)
        self._cbCmdJoint = rospy.Subscriber(self.ns+"/cmd_joint", JointState, self.callback_cmd_joint, queue_size=1)
        self._cbCmdVel = rospy.Subscriber(self.ns+"/cmd_vel", TwistStamped, self.callback_cmd_vel, queue_size=1)
        
        self._cmd_joy = Joy
        self._cmd_joint = JointState
        self._joint_states = JointState
        self._cmd_vel = TwistStamped
        #self.tip_pose = Pose
        #self.ft_sensor = WrenchStamped

        #self.x_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.y_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        #self.z_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        
        self.q1_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q2_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q3_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q4_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q5_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q6_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        
        self.iter = 0
        
        self.home = np.array([0.0,-1.57,1.57,0.0,-1.57,0.0]).reshape((6,1))
        #self.home = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape((6,1))
        
        self.q = self.home.copy()
        
#         self.q1 = self.q[0]
#         self.q2 = self.q[1]
#         self.q3 = self.q[2]
#         self.q4 = self.q[3]
#         self.q5 = self.q[4]
#         self.q6 = self.q[5]
        
#         self.w = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.w = np.zeros((6,1))
        
#         self.w1 = self.w[0]
#         self.w2 = self.w[1]
#         self.w3 = self.w[2]
#         self.w4 = self.w[3]
#         self.w5 = self.w[4]
#         self.w6 = self.w[5]
        
#         self.e = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.e = np.zeros((6,1))

#         self.e1 = self.e[0]
#         self.e2 = self.e[1]
#         self.e3 = self.e[2]
#         self.e4 = self.e[3]
#         self.e5 = self.e[4]
#         self.e6 = self.e[5]
        
#         self.u = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.u = np.zeros((6,1))

#         self.u1 = self.u[0]
#         self.u2 = self.u[1]
#         self.u3 = self.u[2]
#         self.u4 = self.u[3]
#         self.u5 = self.u[4]
#         self.u6 = self.u[5]
        
#         self.v = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.v = np.zeros((6,1))
        
        self.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
        #self.joint_positions = np.array([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])
        self.joint_positions = self.home
        
        self.trajectory = JointTrajectory()
        #self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.joint_names = self.joint_names
        #self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.0,-1.57,1.57,-0.0,-1.57,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))
        self.trajectory.points.append(JointTrajectoryPoint(positions = self.joint_positions, time_from_start = rospy.Duration(0,500000000)))
        
        self.kin = kuka_kinematics('kr6')
        
#         print '\n*** Kuka Description ***\n'
#         self.kin.print_robot_description()
#         print '\n*** Kuka KDL Chain ***\n'
#         self.kin.print_kdl_chain()
#         # FK Position
#         print '\n*** Kuka Position FK ***\n'
#         print self.kin.forward_position_kinematics()
#         # FK Velocity
#         # print '\n*** Kuka Velocity FK ***\n'
#         # kin.forward_velocity_kinematics()
#         # IK
#         print '\n*** Kuka Position IK ***\n'
#         pos = [4.45426035e-01, 0.0, 9.69999830e-01]
#         rot = [0.0, 3.98163384e-04, 0.0, 9.99999921e-01]
#         print self.kin.inverse_kinematics(pos)  # position, don't care orientation
#         print '\n*** Kuka Pose IK ***\n'
#         print self.kin.inverse_kinematics(pos, rot)  # position & orientation
#         # Jacobian
#         print '\n*** Kuka Jacobian ***\n'
#         print self.kin.jacobian()
#         # Jacobian Transpose
#         print '\n*** Kuka Jacobian Tranpose***\n'
#         print self.kin.jacobian_transpose()
#         # Jacobian Pseudo-Inverse (Moore-Penrose)
#         print '\n*** Kuka Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
#         print self.kin.jacobian_pseudo_inverse()
#         # Joint space mass matrix
#         print '\n*** Kuka Joint Inertia ***\n'
#         print self.kin.inertia()
#         # Cartesian space mass matrix
#         print '\n*** Kuka Cartesian Inertia ***\n'
#         print self.kin.cart_inertia()
        
        self.go_home()
        
        self.new_cmd_joint = False
        self.new_cmd_vel = False

    def callback_joint_states(self, data):
        self._joint_states = data
        #print self._joint_states
        # Get angular position
#         self.qft = self._joint_states.position[0] # joint_ft
#         self.q1 = self._joint_states.position[1] # joint_a1
#         self.q2 = self._joint_states.position[2] # joint_a2
#         self.q3 = self._joint_states.position[3] # joint_a3
#         self.q4 = self._joint_states.position[4] # joint_a4
#         self.q5 = self._joint_states.position[5] # joint_a5
#         self.q6 = self._joint_states.position[6] # joint_a6
#         self.q = np.array(self._joint_states.position[1:7]).reshape((6,1))
#         self.q = np.array(self._joint_states.position[0:6]).reshape((6,1))
        for i in range(len(self._joint_states.name)):
            for j in range(len(self.joint_names)):
                if self._joint_states.name[i]==self.joint_names[j]:
                    self.q[j] = self._joint_states.position[i]
        
    def callback_cmd_joy(self, data):
        self._cmd_joy = data
        #print(self._joy)
        
    def callback_cmd_joint(self, data):
        self._cmd_joint = data
        #print(self._cmd_joint)
        # Set angular velocities
#         self.w1 = self._cmd_joint.velocity[0] # joint_a1
#         self.w2 = self._cmd_joint.velocity[1] # joint_a2
#         self.w3 = self._cmd_joint.velocity[2] # joint_a3
#         self.w4 = self._cmd_joint.velocity[3] # joint_a4
#         self.w5 = self._cmd_joint.velocity[4] # joint_a5
#         self.w6 = self._cmd_joint.velocity[5] # joint_a6
        self.w = np.array(self._cmd_joint.velocity[0:6]).reshape((6,1))
        # Reset cartesian velocities
#         self.vx = 0.0
#         self.vy = 0.0
#         self.vz = 0.0
#         self.wx = 0.0
#         self.wy = 0.0
#         self.wz = 0.0
#         self.v = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.v = np.zeros((6,1))
        # Set new_cmd_joint as true
        #if (self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
#         if (math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
        if (math.fabs(self.w[0])>0.01 or math.fabs(self.w[1])>0.01 or math.fabs(self.w[2])>0.01 or math.fabs(self.w[3])>0.01 or math.fabs(self.w[4])>0.01 or math.fabs(self.w[5])>0.01):
            self.new_cmd_joint = True
            self.new_cmd_vel = False
            #self.record_pose = True
        
    def callback_cmd_vel(self, data):
        self._cmd_vel = data
        #print(self._cmd_vel)
        # Set cartesian velocities
#         self.vx = self._cmd_vel.twist.linear.x
#         self.vy = self._cmd_vel.twist.linear.y
#         self.vz = self._cmd_vel.twist.linear.z
#         self.wx = self._cmd_vel.twist.angular.x
#         self.wy = self._cmd_vel.twist.angular.y
#         self.wz = self._cmd_vel.twist.angular.z  
        self.v = np.array([self._cmd_vel.twist.linear.x,self._cmd_vel.twist.linear.y,self._cmd_vel.twist.linear.z,self._cmd_vel.twist.angular.x,self._cmd_vel.twist.angular.y,self._cmd_vel.twist.angular.z]).reshape((6,1))
        # Reset angular velocities
#         self.w1 = 0.0 # joint_a1
#         self.w2 = 0.0 # joint_a2
#         self.w3 = 0.0 # joint_a3
#         self.w4 = 0.0 # joint_a4
#         self.w5 = 0.0 # joint_a5
#         self.w6 = 0.0 # joint_a6
#         self.w = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.w = np.zeros((6,1))
        # Set new_cmd_vel as true
        #if (self.vx!=0.0 or self.vy!=0.0 or self.vz!=0.0 or self.wx!=0.0 or self.wy!=0.0 or self.wz!=0.0):
#         if (math.fabs(self.vx)>0.01 or math.fabs(self.vy)>0.01 or math.fabs(self.vz)>0.01 or math.fabs(self.wx)>0.01 or math.fabs(self.wy)>0.01 or math.fabs(self.wz)>0.01):
        if (math.fabs(self.v[0])>0.01 or math.fabs(self.v[1])>0.01 or math.fabs(self.v[2])>0.01 or math.fabs(self.v[3])>0.01 or math.fabs(self.v[4])>0.01 or math.fabs(self.v[5])>0.01):
            self.new_cmd_vel = True
            self.new_cmd_joint = False
            #self.record_pose = True
            
    def inv_kinematics(self, pose):

        print '\n*** Kuka Position IK ***\n'
        pos = pose.position
        rot = pose.orientation
        print self.kin.inverse_kinematics(pos)  # position, don't care orientation
        print '\n*** Kuka Pose IK ***\n'
        print self.kin.inverse_kinematics(pos, rot)  # position & orientation
        
    def get_joint_vel(self, vel_cartesian):
        
        return self.kin.jacobian_pseudo_inverse() * vel_cartesian
        
    def get_cartesian_vel(self, vel_joints):
        
        return self.kin.jacobian() * vel_joints
        
    def go_home(self, delay = 2.0):
        
        iters = int(round(delay/self.Tc))

        for i in range(0,iters):
         
            self.iter+=1
 
            self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
            self.trajectory.points[0] = JointTrajectoryPoint(positions = self.home, time_from_start = rospy.Duration(delay-i*self.Tc,0))
         
            self.pubTrajectory.publish(self.trajectory) # KR6 (simulated)
            self.pubTrajectory_.publish(self.trajectory) # KR6 (real)
            self.rate.sleep() # Sleeps for 1/rate sec
            
    def go_pose(self, cmd):
        
        self.iter+=1

        self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.points[0] = JointTrajectoryPoint(positions = cmd, time_from_start = rospy.Duration(self.Tc,0))
        
        self.pubTrajectory.publish(self.trajectory) # KR6 (simulated)
        self.pubTrajectory_.publish(self.trajectory) # KR6 (real)
        self.rate.sleep() # Sleeps for 1/rate sec

    def loop(self, fc = 100.0):
        
        # Read current state
#         self.u1 = self.q1
#         self.u2 = self.q2
#         self.u3 = self.q3
#         self.u4 = self.q4
#         self.u5 = self.q5
#         self.u6 = self.q6
        
        self.u = self.q.copy()
        
        # While loop
        while not rospy.is_shutdown():

#             self.e1 = self.q1 - q_joints[self._limb_name+'_s1']
#             self.e2 = self.q2 - q_joints[self._limb_name+'_e0']
#             self.e3 = self.q3 - q_joints[self._limb_name+'_e1']
#             self.e4 = self.q4 - q_joints[self._limb_name+'_w0']
#             self.e5 = self.q5 - q_joints[self._limb_name+'_w1']
#             self.e6 = self.q6 - q_joints[self._limb_name+'_w2']
#  
#             self.u1 = -self.q1_pid.update_PID(self.e1)
#             self.u2 = -self.q2_pid.update_PID(self.e2)
#             self.u3 = -self.q3_pid.update_PID(self.e3)
#             self.u4 = -self.q4_pid.update_PID(self.e4)
#             self.u5 = -self.q5_pid.update_PID(self.e5)
#             self.u6 = -self.q6_pid.update_PID(self.e6)

#             if self.new_cmd_joint==True:
# 
#                 self.u1 = self.w1*self.Tc + self.u1
#                 self.u2 = self.w2*self.Tc + self.u2
#                 self.u3 = self.w3*self.Tc + self.u3
#                 self.u4 = self.w4*self.Tc + self.u4
#                 self.u5 = self.w5*self.Tc + self.u5
#                 self.u6 = self.w6*self.Tc + self.u6
#                 
#                 self.new_cmd_joint = False

            if self.new_cmd_vel==True:
                
# #                 v = np.array([self.vx, self.vy, self.vz, self.wx, self.wy, self.wz])[np.newaxis]
#                 v = np.array([self.v[0], self.v[1], self.v[2], self.v[3], self.v[4], self.v[5]])[np.newaxis]
#                 w = self.get_joint_vel(v.T) # Define a column vector
#                 v = np.array([self.v[0], self.v[1], self.v[2], self.v[3], self.v[4], self.v[5]]).reshape((6,1))
                self.w = self.get_joint_vel(self.v) # Define a column vector
                #v = np.array([[self.vx], [self.vy], [self.vz], [self.wx], [self.wy], [self.wz]]) # Define a column vector)
                #w = self.get_joint_vel(v) # Define a column vector

#                 self.u1 = w[0]*self.Tc + self.u1
#                 self.u2 = w[1]*self.Tc + self.u2
#                 self.u3 = w[2]*self.Tc + self.u3
#                 self.u4 = w[3]*self.Tc + self.u4
#                 self.u5 = w[4]*self.Tc + self.u5
#                 self.u6 = w[5]*self.Tc + self.u6

#                 self.w1 = w[0]
#                 self.w2 = w[1]
#                 self.w3 = w[2]
#                 self.w4 = w[3]
#                 self.w5 = w[4]
#                 self.w6 = w[5]

#                 self.w = np.squeeze(np.array(w), axis=1)
#                 self.w = np.reshape(w, 6)
#                 self.w[0] = w[0]
#                 self.w[1] = w[1]
#                 self.w[2] = w[2]
#                 self.w[3] = w[3]
#                 self.w[4] = w[4]
#                 self.w[5] = w[5]
#                 self.w = np.array([w[0], w[1], w[2], w[3], w[4], w[5]])
                
                self.new_cmd_vel = False
   
#             self.u1 = self.w1*self.Tc + self.u1
#             self.u2 = self.w2*self.Tc + self.u2
#             self.u3 = self.w3*self.Tc + self.u3
#             self.u4 = self.w4*self.Tc + self.u4
#             self.u5 = self.w5*self.Tc + self.u5
#             self.u6 = self.w6*self.Tc + self.u6

            # Update joint_positions only if there's a big difference between actual and reference joint positions (to avoid stuck configurations)
#             error_norm = np.linalg.norm(np.array(self.u).transpose()-np.array(self.q).transpose())
#             error_norm = np.linalg.norm(np.array(self.u).transpose()-np.array(self.q).transpose())
#             print error_norm
#             if(error_norm>0.001):
#                 self.u = self.q.copy()
                
            self.u = self.w*self.Tc + self.u
#             self.u[0] = self.w[0]*self.Tc + self.u[0]
#             self.u[1] = self.w[1]*self.Tc + self.u[1]
#             self.u[2] = self.w[2]*self.Tc + self.u[2]
#             self.u[3] = self.w[3]*self.Tc + self.u[3]
#             self.u[4] = self.w[4]*self.Tc + self.u[4]
#             self.u[5] = self.w[5]*self.Tc + self.u[5]
#             self.u[0] = self.w1*self.Tc + self.u[0]
#             self.u[1] = self.w2*self.Tc + self.u[1]
#             self.u[2] = self.w3*self.Tc + self.u[2]
#             self.u[3] = self.w4*self.Tc + self.u[3]
#             self.u[4] = self.w5*self.Tc + self.u[4]
#             self.u[5] = self.w6*self.Tc + self.u[5]
            self.go_pose(self.u)
        
#             self.joint_positions = np.array([self.u1,self.u2,self.u3,self.u4,self.u5,self.u6])
#             self.go_pose(self.joint_positions)
 
#             print "New command"
#             print self.u
#             print self.joint_positions
            
            #rospy.spin()

def main():
    rospy.init_node("kr6_control")

    kr6_control = KR6control()

    kr6_control.loop()

if __name__ == '__main__':
    main()
