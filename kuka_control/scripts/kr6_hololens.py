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
    Twist
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
# from pyquaternion import Quaternion

from pid import PID
# from meld.misc import position_menu_under_widget

from kuka_pykdl import kuka_kinematics
from autobahn.twisted.util import sleep
from geometry_msgs.msg._TwistStamped import TwistStamped

from utilities_hololens import *

#from scipy.spatial.transform import Rotation as R
from transformations import *

class KR6control:

    def __init__(self, fc=100.0, ns='kr6'):
        
        self.fc = fc
        self.rate = rospy.Rate(self.fc)  # 100 Hz
        self.Tc = 1.0 / self.fc

        self._joint_states = JointState
        self._cmd_joint = JointState
        self._cmd_vel = TwistStamped
        # self.tip_pose = Pose
        self.ft_sensor = WrenchStamped()
        self.ft_twist = Twist()

        # self.x_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        # self.y_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        # self.z_pid = PID(30.0, 0.0, 1.0, 1.0, -1.0)
        
        self.q1_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q2_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q3_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q4_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q5_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        self.q6_pid = PID(10.0, 0.0, 0.0, 1.0, -1.0)
        
        self.iter = 0
        self.iter_data = 0
        
        self.home = np.array([0.0, -math.pi / 2, math.pi / 2, 0.0, -math.pi / 2 + 0.3, 0.0]).reshape((6, 1))
        # self.home = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).reshape((6,1))
        
        # self.q0 = self.home.copy()
        self.w = np.zeros((6, 1))
        self.e = np.zeros((6, 1))
        self.u = np.zeros((6, 1))
        self.v = np.zeros((6, 1))
        
        self.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        # self.joint_positions = np.array([self.q1,self.q2,self.q3,self.q4,self.q5,self.q6])
        self.joint_positions = self.home
        
        self.trajectory = JointTrajectory()
        # self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.joint_names = self.joint_names
        # self.trajectory.points.append(JointTrajectoryPoint(positions = np.array([0.0,-1.57,1.57,-0.0,-1.57,-0.0]), time_from_start = rospy.Duration(1.0,0.0)))
        self.trajectory.points.append(JointTrajectoryPoint(positions=self.joint_positions, time_from_start=rospy.Duration(0, 500000000)))
        
#         self.kin = kuka_kinematics('kr6')
        
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
        
        self.new_cmd_joint = False
        self.new_cmd_vel = False
        
        self.DH = []
        self.Tn = []
        self.Twe = np.eye(4)
        self.p = np.zeros((6, 1))
        self.pd = np.zeros((6, 1))
        
        self.base = np.eye(4)
        self.base[0, 3] = -0.46
        self.base[1, 3] = 0
        self.base[2, 3] = 2.05075  # 2.05275
        # self.base[2,3]=2.055
        
        self.J0 = []
        self.Jn = []
        self.Jrec = []
        
        self.n_joints = 6
        self.name = [None] * self.n_joints
        self.q = [None] * self.n_joints
        
        self.tool = 0.127 + 0.035  # +0.0225
        
        self.p_ee = Point()
        self.tool_pose = PoseStamped()
        self.object_pose = PoseStamped()

        self.ns = ns
        self.pubTrajectory_ = rospy.Publisher("/position_trajectory_controller/command", JointTrajectory, queue_size=1)
        self.pubTrajectory = rospy.Publisher(self.ns + "/position_trajectory_controller/command", JointTrajectory, queue_size=1)

        self.pub_coords = rospy.Publisher('/point', Point, queue_size=1)
        self.pub_tool = rospy.Publisher('/tool', PoseStamped, queue_size=1)
        self.pub_object = rospy.Publisher('/object', PoseStamped, queue_size=1)
        self.pub_ft = rospy.Publisher('/ft', Twist, queue_size=1)
        
        self._cbJointState = rospy.Subscriber(self.ns + "/joint_states", JointState, self.callback_joint_states, queue_size=1)
        self._cbCmdJoint = rospy.Subscriber(self.ns + "/cmd_joint", JointState, self.callback_cmd_joint, queue_size=1)
        self._cbCmdVel = rospy.Subscriber(self.ns + "/cmd_vel", TwistStamped, self.callback_cmd_vel, queue_size=1)
        self._cbFTSensor= rospy.Subscriber(self.ns + "/axia80/ft_sensor_topic", WrenchStamped, self.callback_ft_sensor, queue_size=1) #Simulation
        
        self.go_home()
        
    def get_joint_info(self):

        self.DH = Kuka_DenHartStandard_mod(self.q, self.tool)
        self.Tn = Kuka_fkine(self.DH, self.n_joints)
        R = t2r(self.Tn)
        self.Rot = R[:]
        self.RPY = R2rpy(R)
        if self.RPY[0] < 0:
            self.RPY[0] = self.RPY[0] + 2.0 * math.pi
        if self.RPY[2] > 0:
            self.RPY[2] = self.RPY[2] - 2.0 * math.pi
                
        self.Twe = self.base.dot(self.Tn)
    
        self.Jn = Kuka_jacobn(self.q, self.DH, self.n_joints)
        self.J0 = Kuka_jacob0(self.q, self.DH, self.n_joints)
        self.Jrec = jacobn_rec(self.J0, self.RPY)
             
        # Publishing end-effector coords:
        self.p_ee.x = self.Twe[0, 3]
        self.p_ee.y = self.Twe[1, 3]
        self.p_ee.z = self.Twe[2, 3]
         
        self.pub_coords.publish(self.p_ee)

    def callback_joint_states(self, data):
        self._joint_states = data
        # Get angular position
        for i in range(len(self._joint_states.name)):
            for j in range(len(self.joint_names)):
                if self._joint_states.name[i] == self.joint_names[j]:
                    self.q[j] = self._joint_states.position[i]
        
    def callback_cmd_joint(self, data):
        self._cmd_joint = data
        # Set angular velocities
        self.w = np.array(self._cmd_joint.velocity[0:6]).reshape((6, 1))
        # Reset cartesian velocities
        self.v = np.zeros((6, 1))
        # Set new_cmd_joint as true
        if (math.fabs(self.w[0]) > 0.01 or math.fabs(self.w[1]) > 0.01 or math.fabs(self.w[2]) > 0.01 or math.fabs(self.w[3]) > 0.01 or math.fabs(self.w[4]) > 0.01 or math.fabs(self.w[5]) > 0.01):
            self.new_cmd_joint = True
            self.new_cmd_vel = False
            # self.record_pose = True
        
    def callback_cmd_vel(self, data):
        self._cmd_vel = data
        # Set cartesian velocities
        self.v = np.array([self._cmd_vel.twist.linear.x, self._cmd_vel.twist.linear.y, self._cmd_vel.twist.linear.z, self._cmd_vel.twist.angular.x, self._cmd_vel.twist.angular.y, self._cmd_vel.twist.angular.z]).reshape((6, 1))
        # Reset angular velocities
        self.w = np.zeros((6, 1))
        # Set new_cmd_vel as true
        if (math.fabs(self.v[0]) > 0.01 or math.fabs(self.v[1]) > 0.01 or math.fabs(self.v[2]) > 0.01 or math.fabs(self.v[3]) > 0.01 or math.fabs(self.v[4]) > 0.01 or math.fabs(self.v[5]) > 0.01):
            self.new_cmd_vel = True
            self.new_cmd_joint = False
            # self.record_pose = True
            
    def callback_ft_sensor(self, data):
        self.ft_sensor = data
        
        self.ft_twist.linear = self.ft_sensor.wrench.force
        self.ft_twist.angular = self.ft_sensor.wrench.torque
        
        #print self.ft_sensor
        #print self.ft_twist
        
        self.pub_ft.publish(self.ft_twist)
            
#     def inv_kinematics(self, pose):
# 
#         print '\n*** Kuka Position IK ***\n'
#         pos = pose.position
#         rot = pose.orientation
#         print self.kin.inverse_kinematics(pos)  # position, don't care orientation
#         print '\n*** Kuka Pose IK ***\n'
#         print self.kin.inverse_kinematics(pos, rot)  # position & orientation
#         
#     def get_joint_vel(self, vel_cartesian):
#         
#         return self.kin.jacobian_pseudo_inverse() * vel_cartesian
#         
#     def get_cartesian_vel(self, vel_joints):
#         
#         return self.kin.jacobian() * vel_joints

    def get_joint_vel(self, vel_cartesian):
        
        return linalg.pinv(self.J0, rcond=0.01).dot(vel_cartesian)
        
    def go_home(self, delay=2.0):
        
        iters = int(round(delay / self.Tc))

        for i in range(0, iters):
         
            self.iter += 1
 
            self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
            self.trajectory.points[0] = JointTrajectoryPoint(positions=self.home, time_from_start=rospy.Duration(delay - i * self.Tc, 0))
         
            self.pubTrajectory.publish(self.trajectory)  # KR6 (simulated)
            self.pubTrajectory_.publish(self.trajectory)  # KR6 (real)
            self.rate.sleep()  # Sleeps for 1/rate sec
            
    def go_pose(self, cmd):
        
        self.iter += 1

        self.trajectory.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
        self.trajectory.points[0] = JointTrajectoryPoint(positions=cmd, time_from_start=rospy.Duration(self.Tc, 0))
        
        self.pubTrajectory.publish(self.trajectory)  # KR6 (simulated)
        self.pubTrajectory_.publish(self.trajectory)  # KR6 (real)
        self.rate.sleep()  # Sleeps for 1/rate sec
        
    def publish_data(self):
        
        self.iter_data += 1
        
        self.object_pose.pose.position.x = 0.0
        self.object_pose.pose.position.y = 0.0
        self.object_pose.pose.position.z = 0.0
        self.object_pose.pose.orientation.x = 0.0
        self.object_pose.pose.orientation.y = 0.0
        self.object_pose.pose.orientation.z = 0.0
        self.object_pose.pose.orientation.w = 1.0
        self.object_pose.header = Header(stamp=rospy.Time.now(), frame_id='base', seq=self.iter)
            
        self.tool_pose.pose.position.x = self.p_ee.x
        self.tool_pose.pose.position.y = self.p_ee.y
        self.tool_pose.pose.position.z = self.p_ee.z

        R = euler_matrix(numpy.pi, 0.0, 0.0)
        q = quaternion_from_matrix(self.Rot)
        
        self.tool_pose.pose.orientation.x = q[0]
        self.tool_pose.pose.orientation.y = q[1]
        self.tool_pose.pose.orientation.z = q[2]
        self.tool_pose.pose.orientation.w = q[3]
        self.tool_pose.header = self.object_pose.header
            
        self.pub_object.publish(self.object_pose)  # Object's pose
        self.pub_tool.publish(self.tool_pose)  # Tool's pose

    def loop(self, fc=100.0):
        
        # Read current state  
        # self.u = self.q.copy()
        for i in range(0, len(self.q)):
            self.u[i] = self.q[i]
        
        # While loop
        while not rospy.is_shutdown():
                
            self.get_joint_info()
            
            self.publish_data()

            if self.new_cmd_vel == True:
                
                self.w = self.get_joint_vel(self.v)
                
                self.new_cmd_vel = False

            self.u = self.w * self.Tc + self.u

            self.go_pose(self.u)
        
#             self.joint_positions = np.array([self.u1,self.u2,self.u3,self.u4,self.u5,self.u6])
#             self.go_pose(self.joint_positions)
            
            # rospy.spin()


def main():
    rospy.init_node("kr6_hololens")

    kr6_control = KR6control()

    kr6_control.loop()


if __name__ == '__main__':
    main()
