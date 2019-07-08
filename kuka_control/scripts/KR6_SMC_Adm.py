#!/usr/bin/python

import numpy as np
import math
import time
import rospy
import tf
import message_filters
import sys


from geometry_msgs.msg import (
    Point,
    Quaternion,
    WrenchStamped,
    Pose,
    PoseStamped,
    PoseArray
)
from std_msgs.msg import (
     Header,
     Bool
     )
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)
from sensor_msgs.msg import (
    Joy,
    JointState
)
from pyquaternion import Quaternion

from kuka_pykdl import kuka_kinematics
from autobahn.twisted.util import sleep
from geometry_msgs.msg._TwistStamped import TwistStamped
from utilities_HRCS import *
from trollius.tasks import wait_for
from numpy import concatenate
from scipy.spatial.transform import Rotation 
from numpy.linalg.linalg import svd


class KR6controller:
    def __init__(self):
        
        
        self._FTwrench = ForceSensorClass()
        self._cbFTSensor = rospy.Subscriber("/netft_data_172_1_10_1", WrenchStamped, self._FTwrench, queue_size=1) #Real
       #self._cbFTSensor= rospy.Subscriber("/kr6/axia80/ft_sensor_topic", WrenchStamped, self._FTwrench, queue_size=1) #Simulation
       
        self.p_ee= Point()
        self._done = False
        self.fc = 100.0 
        self.rate = rospy.Rate(self.fc)
        self.n_joints=6
        self.name = [None]*self.n_joints
        self.q = [None]*self.n_joints
        self.q_temp = np.zeros((self.n_joints,1))
        self.qp=np.zeros((self.n_joints,1))
        self.qd = np.zeros((self.n_joints,1))
        self.q_ant=np.zeros((self.n_joints,1))
        
        
        self.q_cmd=[]
        self.home=np.array([0.0, -math.pi/2, math.pi/2, 0, -math.pi/2, 0]).reshape((6,1))
        self.DH=[]
        self.Tn=[]
        self.Twe=np.eye(4)
        self.p=np.zeros((6,1))
        self.pd=np.zeros((6,1))
        
        self.base=np.eye(4)
        self.base[0,3]=-0.46
        self.base[1,3]=0
        self.base[2,3]=2.05075   #2.05275
        #self.base[2,3]=2.055
        
        self.J0=[]
        self.Jn=[]
        self.Jrec=[]
        self.iter=0
        self.trajectory = JointTrajectory()
        self.joint_names = ['joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6']
        self.trajectory.joint_names = self.joint_names
        self.trajectory.points.append(JointTrajectoryPoint(positions = self.home, time_from_start = rospy.Duration(0,500000000)))
        self.RPY=np.zeros((3,1))
        self.Rot=[]
        self.tool=0.127 +0.035 #+0.0225
        self.p_surf=np.zeros((3,1))
        self.normal_=np.zeros((3,1))
        self.joint_acc = np.zeros((self.n_joints,1))
        self.joint_acc_2 = np.zeros((self.n_joints,1))
        self.joint_acc_3 = np.zeros((self.n_joints,1))
        self.joint_acc_4 = np.zeros((self.n_joints,1))
        self.joint_acc_5 = np.zeros((self.n_joints,1))
        self.joint_acc_total = np.zeros((self.n_joints,1))
        self.error_orient=np.zeros((3,1))
        self.derror_orient=np.zeros((3,1))


        self._WC = 0.0 # object weight compensation in N. Just in case you forget to compensate the sensor bias

        #1sr Level Continuous control parameters
        self._Kp = 1.5
        self._Kpd = 1.8
        self._Kpd_L1cc=0.01
        self._p_ref = np.zeros((6,1))
        self._pd_ref = np.zeros((6,1))
        self._pdd_ref = np.zeros((6,1))
        self.M_aux=np.eye(3)
        self.M_aux[2,2]=0
        self.M_constraints=concatenate( ( concatenate( (zeros((3,3)),zeros((3,3))) ,1) , concatenate( (zeros((3,3)),self.M_aux) ,1) ))
        

        #2nd Level SMC:
        self.epsilon=0.002
        self.sigma_dist=self.epsilon+0.01
        self.sigma_dist_d=0
        self.sigma_dist_ant=0
        self.phi_dist=0
        self.Jv=[]
        self.Kd_dist=2.5
        self.u_dist_plus=0.65
        self.dist=0

        #3d Level SMC Force Control
        self._FH = np.zeros((6,1))
        self.K_FC=np.identity(2)
        self.H_FC=np.zeros((2,6))
        self.Kgs=1
        self.sigma_FC=0
        self.sigma_yaw_FC=0
        self.sigma_yaw_FC_ant=0
        self.sigma_yaw_FC_d=0
        self.sigma_FC_ant=0
        self.sigma_FC_d=0
        self.phi_FC=np.zeros((2,1))
        self.FC_thres=1.0 #N
        #self.FC_thres=3.0 #N
        self.FC_yaw_thres= 0.2 #Nm
        self.Kd_FC=0.2
        self.Kd_yaw_FC=0.05
        self.W_FC=np.zeros((2,2))
        self.W_FC[0,0]=8
        self.W_FC[1,1]=4
        self.u_FC_plus=0.01
        
        self.f_ant_FC=np.zeros((3,1))
        self.Fu=np.zeros((3,1))
        self.Fd=np.zeros((3,1))
        
        
        #3d Level Admittance Control
        self._Fh = np.zeros((6,1))
        self._Fh[0] = 20
        self._Fh[1] = 20 
        self._Fh[2] = 20 
        self._Fh[3] = 0.5 
        self._Fh[4] = 0.5 
        self._Fh[5] = 0.5 

        self._Vr = np.zeros((6,1))
        self._Vr[0] = 0.3 
        self._Vr[1] = 0.3 
        self._Vr[2] = 0.3 
        self._Vr[3] = 0.5 
        self._Vr[4] = 0.5 
        self._Vr[5] = 0.5
 
        self._Cd = np.zeros((6,6))
        self._Cd[0][0] = self._Fh[0]/self._Vr[0]
        self._Cd[1][1] = self._Fh[1]/self._Vr[1]
        self._Cd[2][2] = self._Fh[2]/self._Vr[2]
        self._Cd[3][3] = self._Fh[3]/self._Vr[3]
        self._Cd[4][4] = self._Fh[4]/self._Vr[4]
        self._Cd[5][5] = self._Fh[5]/self._Vr[5]
        
        self._tau = np.zeros((6,1))
        self._tau[0] = 0.15 
        self._tau[1] = 0.15 
        self._tau[2] = 0.15 
        self._tau[3] = 0.15
        self._tau[4] = 0.15 
        self._tau[5] = 0.15 

        self._Md = np.zeros((6,6))
        self._Md[0][0] = self._Cd[0][0]*self._tau[0]
        self._Md[1][1] = self._Cd[1][1]*self._tau[1]
        self._Md[2][2] = self._Cd[2][2]*self._tau[2]
        self._Md[3][3] = self._Cd[3][3]*self._tau[3]
        self._Md[4][4] = self._Cd[4][4]*self._tau[4]
        self._Md[5][5] = self._Cd[5][5]*self._tau[5]   
        
        self.f_FC=np.zeros((3,1))
        self.F=0.0
        self.Fnew=np.zeros((6,1))
        self.T_yaw_new=0.0
        self._u3_plus = 0.01
        self.gain_Fnew =1.5
        #4th Level Speed reduction
        self._Kv_L4=1.1
        self._u_plus_L4=0.01

        #5th Level Position control
        self.M_aux2=np.identity(3)
        self.M_constraints2=concatenate( ( concatenate( (self.M_aux2,zeros((3,3))) ,1) , concatenate( (zeros((3,3)),zeros((3,3))) ,1) ))
        self._Kp_L4 = 2
        self._Kpd_L4 = 4.2
        self._Kpd_L4cc=0.01
        self._p_ref_L4 = np.zeros((6,1))
        self._pd_ref_L4 = np.zeros((6,1))
        self._pdd_ref_L4 = np.zeros((6,1))
        
        #States machine-Automatic Sanding:
        self.err_cuad=1
        self.polished=0
        self.fase2=0
        self.down=0
        self.polishing=0
        self.up=0
        self.cont_pol=0
        self.p_ref_attack=np.zeros((6,1))
        self.norm_attack=np.zeros((3,1))
        self.point_list=[]
        self.norm_list=[]
        self.point_ind=0
        self.last_point=3
        self.v_auto=0.5
        self.inc=0
        self.ref_inc=np.zeros((6,1))
        self.finished_polishing=0
        self.init_list=PoseArray()
        self.p_ref_final=np.zeros((6,1))
        
        # Auxiliary varibles
        self.count=0
        self.guidance=0
        self._time = 0.0
        self.first_iter=False
        self.new_msg=False
        self.new_cp=False
        self.new_norm=False
        self.first_list=False
        self.new_list=False
        self.Ts=1.0/self.fc
        self.cont=0
        self.rcond=0.01
        self._saveData = open("/home/idf/ros_ws/kuka_ws/src/results_p2.txt","w")
        
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback) #Real
        #rospy.Subscriber('kr6/joint_states', JointState, self.joint_states_callback) #Simulation
        rospy.Subscriber('closest_point', Point, self.closest_point_callback)
        rospy.Subscriber('closest_normal', Point, self.closest_normal_callback)
        rospy.Subscriber('final_list', PoseArray, self.cmd_points_callback)
        rospy.Subscriber('new_list', Bool, self.ifnewlist)

        
        self.pub=rospy.Publisher('/kr6/position_trajectory_controller/command',JointTrajectory,queue_size=1)
        self.pub_real=rospy.Publisher('position_trajectory_controller/command',JointTrajectory,queue_size=1)
        self.pub_coords=rospy.Publisher('/input_coords',Point,queue_size=1)
        
    def ifnewlist(self,msg_l):
        if msg_l.data==True:
         self.new_list=msg_l.data
         
    
    def joint_states_callback(self,msg):
        for i in range (1,7):
         if 'joint_a'+str(i) in msg.name:
              index=msg.name.index('joint_a'+str(i))
              self.name[i-1]= msg.name[index]
              self.q[i-1]=msg.position[index]

    def closest_point_callback(self,msg1):
        self.p_surf[0]=msg1.x
        self.p_surf[1]=msg1.y
        self.p_surf[2]=msg1.z
        self.new_cp=True

    def closest_normal_callback(self,msg2):
        #self.normal_[0]=msg2.x
        #self.normal_[1]=msg2.y
        #self.normal_[2]=msg2.z
        self.new_norm=True
        
    def cmd_points_callback(self,msg3):
        self.init_list=msg3
        
        if self.first_list==False:
         self.point_list=np.zeros((len(self.init_list.poses)/2,3))
         self.norm_list=np.zeros((len(self.init_list.poses)/2,3))
         self.last_point=len(self.init_list.poses)/2
         for i in range (0,len(self.init_list.poses)/2):
            self.point_list[i,0]=self.init_list.poses[2*i].position.x 
            self.point_list[i,1]=self.init_list.poses[2*i].position.y 
            self.point_list[i,2]=self.init_list.poses[2*i].position.z 
            self.norm_list[i,0]=self.init_list.poses[2*i+1].position.x
            self.norm_list[i,1]=self.init_list.poses[2*i+1].position.y
            self.norm_list[i,2]=self.init_list.poses[2*i+1].position.z
         self.p_ref_attack[0]=self.point_list[0,0]+0.1*self.norm_list[0,0]
         self.p_ref_attack[1]=self.point_list[0,1]+0.1*self.norm_list[0,1]
         self.p_ref_attack[2]=self.point_list[0,2]+0.1*self.norm_list[0,2]
         self.norm_attack[0]=self.norm_list[0,0]
         self.norm_attack[1]=self.norm_list[0,1]
         self.norm_attack[2]=self.norm_list[0,2]
         self._p_ref_L4=self.p_ref_attack[:]   
         
        else:
         if self.new_list==False:
          e_c=0
          e_c1=0
          for i in range (0,len(self.init_list.poses)/2):
             e_c=pow(self.point_list[i,0]-self.init_list.poses[2*i].position.x,2)+pow(self.point_list[i,1]-self.init_list.poses[2*i].position.y,2)+pow(self.point_list[i,2]-self.init_list.poses[2*i].position.z,2)
             if e_c>e_c1:
              e_c1=e_c
          print "err matching"
          print e_c1
          if e_c1>0.001:
             print "misma lista"
             for i in range (0,len(self.init_list.poses)/2):
                 self.point_list[i,0]=self.init_list.poses[2*i].position.x 
                 self.point_list[i,1]=self.init_list.poses[2*i].position.y 
                 self.point_list[i,2]=self.init_list.poses[2*i].position.z
                 self.norm_list[i,0]=self.init_list.poses[2*i+1].position.x
                 self.norm_list[i,1]=self.init_list.poses[2*i+1].position.y
                 self.norm_list[i,2]=self.init_list.poses[2*i+1].position.z
             self.p_ref_attack[0]=self.point_list[self.point_ind,0]+0.1*self.norm_list[self.point_ind,0]
             self.p_ref_attack[1]=self.point_list[self.point_ind,1]+0.1*self.norm_list[self.point_ind,1]
             self.p_ref_attack[2]=self.point_list[self.point_ind,2]+0.1*self.norm_list[self.point_ind,2]
             self.norm_attack[0]=self.norm_list[self.point_ind,0]
             self.norm_attack[1]=self.norm_list[self.point_ind,1]
             self.norm_attack[2]=self.norm_list[self.point_ind,2]
             self._p_ref_L4=self.p_ref_attack[:]   
         elif self.new_list==True:
          print "nueva lista"
          self.point_list=np.zeros((len(self.init_list.poses)/2,3))
          self.norm_list=np.zeros((len(self.init_list.poses)/2,3))
          self.last_point=len(self.init_list.poses)/2
          for i in range (0,len(self.init_list.poses)/2):
            self.point_list[i,0]=self.init_list.poses[2*i].position.x
            self.point_list[i,1]=self.init_list.poses[2*i].position.y 
            self.point_list[i,2]=self.init_list.poses[2*i].position.z
            self.norm_list[i,0]=self.init_list.poses[2*i+1].position.x
            self.norm_list[i,1]=self.init_list.poses[2*i+1].position.y
            self.norm_list[i,2]=self.init_list.poses[2*i+1].position.z 
          self.p_ref_attack[0]=self.point_list[0,0]+0.1*self.norm_list[0,0]
          self.p_ref_attack[1]=self.point_list[0,1]+0.1*self.norm_list[0,1]
          self.p_ref_attack[2]=self.point_list[0,2]+0.1*self.norm_list[0,2]
          self.norm_attack[0]=self.norm_list[0,0]
          self.norm_attack[1]=self.norm_list[0,1]
          self.norm_attack[2]=self.norm_list[0,2]
          self._p_ref_L4=self.p_ref_attack[:]  
          self.finished_polishing=0
          self.polished=0
          self.point_ind=0
          self.new_list=False
          
            
        if self.first_list==False:
            self.first_list=True
            self.new_list=False
     
    def get_joint_info(self):
        if self.name == [None]*self.n_joints:
         print "No robot_state messages received!\n"
        else :
         self.new_msg=True
         self.DH=Kuka_DenHartStandard_mod(self.q,self.tool)
         self.Tn=Kuka_fkine(self.DH,self.n_joints)
         R = t2r(self.Tn)
         self.Rot=R[:]
         self.RPY = R2rpy(R)
         if self.RPY[0] < 0:
            self.RPY[0] = self.RPY[0] + 2.0*math.pi
         if self.RPY[2] > 0:
            self.RPY[2] = self.RPY[2] - 2.0*math.pi
            
         self.Twe=self.base.dot(self.Tn)
         
         
         self.Jn=Kuka_jacobn(self.q,self.DH,self.n_joints)
         self.J0=Kuka_jacob0(self.q,self.DH,self.n_joints)
         self.Jrec=jacobn_rec(self.J0,self.RPY)
         
         #Publishing end-effector coords:
         self.p_ee.x=self.Twe[0,3]
         self.p_ee.y=self.Twe[1,3]
         self.p_ee.z=self.Twe[2,3]
         
         self.pub_coords.publish(self.p_ee)
    
    def pos(self,phi, num_el):
       if num_el>1:  
        pos_phi=np.zeros((num_el,1))
        for i in range (0,num_el):
         if phi[i]>0:
            pos_phi[i]=1
         else:
            pos_phi[i]=0
       else:
         pos_phi=0
         if phi>0:
            pos_phi=1
         else:
            pos_phi=0  
     
       return pos_phi
    
    def v2dm(self,u, num_el):
        dm=np.zeros((num_el,num_el))
        for i in range (0,num_el):
            dm[i,i]=u[i]
        
        return dm    
    
    def _SMC_acc_Cartesian(self,Jn,J):
        
        #Computing the pose in Cartesian and Joint workspaces
        p = np.zeros((6,1))
        q  = np.array(self.q).reshape((6,1))
        
        qd = self.qd[:]
        self.Twe=self.base.dot(self.Tn)
        for i in range(0,6,1):
            if i < 3:
               p[i] = self.Twe[i,3]
            else:
               p[i] = self.RPY[i-3]

        pd = J.dot(qd)
        pdn = Jn.dot(qd)
        
        self.p=p[:]
        self.pd=pd[:]

        error = self._p_ref - p
        error_L5=self._p_ref_L4 - p
        error_L5[5]=0

        self.error_orient=error[3:6]
        m_e = np.sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2])
        m_e_L5 = np.sqrt(error_L5[0]*error_L5[0] + error_L5[1]*error_L5[1] + error_L5[2]*error_L5[2])
        sat_error=0.05
        if self.fase2==1:
         sat_error_L4=0.025
        else:
         sat_error_L4=0.055
        if m_e > sat_error:
           for i in range(0,3):
               error[i] = error[i]/m_e*sat_error
        if m_e_L5 > sat_error_L4:
           for i in range(0,3):
               error_L5[i] = error_L5[i]/m_e_L5*sat_error_L4


        derror=(self._pd_ref - pd)
        derror_L5=(self._pd_ref_L4 - pd)

        self.derror_orient=derror[3:6]
      
        #1st Level Orientation continous control:
        A1=self.M_constraints.dot(J)
        b1=self.M_constraints.dot(self._pdd_ref + self._Kpd*derror + self._Kp*error + np.sign(derror + self._Kp/self._Kpd*error)*self._Kpd_L1cc)
        joint_acc = linalg.pinv(A1,rcond = self.rcond).dot(b1)
        #joint_acc=np.zeros((6,1))
   
        #2nd level: SMC distance
        N1 = np.eye(6) - linalg.pinv((self.M_constraints).dot(J),rcond = self.rcond).dot((self.M_constraints).dot(J))
        A2= self.pos(self.phi_dist*(1-self.polishing),1)*self.Kd_dist*(J[0:3,:].T.dot(-self.normal_)).T
        b2= - self.pos(self.phi_dist*(1-self.polishing),1)*self.u_dist_plus
        joint_acc_2= linalg.pinv(A2.dot(N1),rcond=self.rcond).dot(b2-A2.dot(joint_acc)) 
        
        #joint_acc_2=np.zeros((6,1)) #Uncomment if L2 blocked

        #3d level: Admittance control for guidance OR Automatic Continuos Control for XYZ
        N2=N1*(np.eye(6)-linalg.pinv(A2.dot(N1),rcond=0.01).dot(A2.dot(N1)))
        if self.F>=self.FC_thres or np.abs(self.yaw_FC)>=self.FC_yaw_thres:
         A3=self._Md.dot(Jn)
         
         #W/O THRESHOLD:
         #b3=self._FH-(self._Cd.dot(Jn)).dot(qd)-self._u3_plus*np.sign((self._Cd.dot(Jn)).dot(qd)-self._FH)
         
         #WITH THRESHOLD:
         b3=self.Fnew-(self._Cd.dot(Jn)).dot(qd)-self._u3_plus*np.sign((self._Cd.dot(Jn)).dot(qd)-self.Fnew)
         self.guidance=1
         self.count=0
        #joint_acc_3 = linalg.pinv(A3.dot(N2),rcond = 0.01).dot(b3 - A3.dot(joint_acc + joint_acc_2))
        else:
         A3=J
         if self.count>=3*self.fc:
          b3=(self._pdd_ref_L4 + self._Kpd_L4*derror_L5 + self._Kp_L4*error_L5*(1-self.polishing) + np.sign(derror_L5 + (1-self.polishing)*self._Kp_L4/self._Kpd_L4*error_L5)*self._Kpd_L4cc)
         else:
          b3=(self._pdd_ref_L4 + self._Kpd_L4*derror_L5  + np.sign(derror_L5)*self._Kpd_L4cc)
          self.count=self.count+1
         
         self.guidance=0
        joint_acc_3 = linalg.pinv(A3.dot(N2),rcond=self.rcond).dot(b3 - A3.dot(joint_acc + joint_acc_2))
        

        self.joint_acc=joint_acc[:]
        self.joint_acc_2=joint_acc_2[:]
        self.joint_acc_3=joint_acc_3[:]
        #self.joint_acc_5=joint_acc_5[:]
        
        self.joint_acc_total=self.joint_acc+self.joint_acc_2+self.joint_acc_3  #+joint_acc_5
        self.qp = self.qp + (joint_acc + joint_acc_2 + joint_acc_3 )/self.fc
        #self.qp = self.qp + (joint_acc + joint_acc_2 + joint_acc_3 + self.joint_acc_4+joint_acc_5)/self.fc
        self.q_cmd = self.q_cmd + self.qp/self.fc
        
        self.trajectory.joint_names = self.name
        self.trajectory.header = Header(stamp=rospy.Time.now(),seq=self.iter)
        self.trajectory.points[0] = JointTrajectoryPoint(positions=self.q_cmd,time_from_start = rospy.Duration(self.Ts,0))
        self.pub_real.publish(self.trajectory)
        #self.pub.publish(self.trajectory)

        
    def ctrl_actions(self):
        
        self.get_joint_info()
        if self.new_msg == True:
         self.iter+=1
         
         msg = self._FTwrench.get_msg()
        
         self.normal_[0]=0
         self.normal_[1]=0
         self.normal_[2]=1
         nRot=normal2rot(-self.normal_)   
         #nRot=normal2rot([0,0,-1])
         RotRef=R2rpy(nRot)
         
         if RotRef[0] < 0:
          RotRef[0] = RotRef[0] + 2.0*math.pi
         if RotRef[2] > 0:
          RotRef[2] = RotRef[2] - 2.0*math.pi
         for i in range(0,6,1):
          if i < 3:
            self._p_ref[i] = self.p_surf[i]
          else:
            self._p_ref[i] = RotRef[i-3]
        
         
         #SMC distance:
         self.Twe=self.base.dot(self.Tn)
         
         
         #Sensor data for guidind the robot according to the end-effector's frame 
         self._FH[0] =0# -msg.wrench.force.x 
         self._FH[1] =0# -msg.wrench.force.y 
         self._FH[2] = (msg.wrench.force.z - self._WC)
         self._FH[5] =0# msg.wrench.torque.z 
         
         #SMC distance control:
         d=(self.Twe[0:3,3]-self._p_ref[0:3]).T.dot(self.normal_)[0,0]
         #print d 
         self.dist=d
         self.sigma_dist=-d+self.epsilon;
         if self.iter==1 :
          self.sigma_dist_ant=self.sigma_dist
         self.sigma_dist_d=(self.sigma_dist-self.sigma_dist_ant)*self.fc
         self.phi_dist=self.sigma_dist+self.Kd_dist*self.sigma_dist_d
         
         
         #SMC Force Control:
         #Rot=t2r(self.Tn)
         self.f_FC=self._FH[0:3]
         self.F=np.sqrt(self.f_FC.T.dot(self.f_FC))
         self.yaw_FC=self._FH[5]
         
         #print self.yaw_FC
         #WITH THRESHOLD:
         self.Fnew[0:3]=max((self.F-self.FC_thres)/self.F,0)*self.gain_Fnew*self.f_FC
         self.T_yaw_new=max(((np.abs(self.yaw_FC)-self.FC_yaw_thres)/np.abs(self.yaw_FC)),0)*self.yaw_FC
         self.Fnew[5]=self.T_yaw_new
         
         #print self._FH
         
         
         #self.sigma_FC = (self.f_FC.T.dot(self.f_FC)-self.FC_thres)[0,0]
         
         
         self.qd = (np.array(self.q).reshape((6,1))-self.q_ant)*self.fc
         
         
         #Automatic sanding:
         
         #Checks if it has to change the attack point:
         if self.first_list==True:
          if self.polished==1:
             if self.point_ind<(self.last_point-1):
                self.point_ind=self.point_ind+1
                self.p_ref_attack[0]=self.point_list[self.point_ind,0]+0.1*self.norm_list[self.point_ind,0]
                self.p_ref_attack[1]=self.point_list[self.point_ind,1]+0.1*self.norm_list[self.point_ind,1]
                self.p_ref_attack[2]=self.point_list[self.point_ind,2]+0.1*self.norm_list[self.point_ind,2]
                self.norm_attack[0]=self.norm_list[self.point_ind,0]
                self.norm_attack[1]=self.norm_list[self.point_ind,1]
                self.norm_attack[2]=self.norm_list[self.point_ind,2]
             else:
                self.finished_polishing=1
             self.polished=0
             self._p_ref_L4=self.p_ref_attack[:]
             self.inc=0
             self.ref_inc=np.zeros((6,1))
         #Quadratic error:
          self.Twe=self.base.dot(self.Tn)
          p_= np.zeros((6,1))
          for i in range(0,6,1):
            if i < 3:
               p_[i] = self.Twe[i,3]
            else:
               p_[i] = self.RPY[i-3]
          err_=(self.p_ref_attack-p_)[0:3]
          self.err_cuad=np.sqrt(err_.T.dot(err_))
          #Performs polishing task if any point of the list remains not polished:
          #print "Fuerza:"
          #print (np.sqrt(self.Fd.T.dot(self.Fd)))[0,0]
          if self.finished_polishing==0:
           #print self.F[0,0]
           if self.F[0,0]<self.FC_thres:       
            #print "errcuad:"
            #print self.err_cuad
            #print "fase 2:"
            #print self.fase2
            #print "bajada:"
            #print self.down
            #print "stop:"
            #print self.polishing
            #print "up:"
            #print self.up
            #print "d:"
            #print d
            if self.err_cuad<=0.0005:
             if self.up==0:
                 self.fase2=1
                 self.down=1
                 self.p_ref_final[0]=self.point_list[self.point_ind,0]
                 self.p_ref_final[1]=self.point_list[self.point_ind,1]
                 self.p_ref_final[2]=self.point_list[self.point_ind,2]
             elif self.up==1:
                 self.fase2=0
                 self.up=0
                 self.polished=1
            if self.fase2==1:
             if self.down==1:
                 #self.inc=self.v_auto/self.fc
                 #self.ref_inc[0:3]=((d-self.inc)*(self.normal_))[:]
                 #print (d-self.inc)
                 #self._p_ref_L4=self.p_ref_final+self.ref_inc
                 self._p_ref_L4=self.p_ref_final
                 dist_ref=(self.Twe[0:3,3]-self._p_ref_L4[0:3]).T.dot(self.norm_attack)[0,0]
                 
                 #print "-----"
                 #print "distancia:"
                 #print dist_ref
                 #print "----"
                 if dist_ref<=0.0095:
                     self.down=0
                     self.polishing=1
             elif self.polishing==1:
                 self.cont_pol=self.cont_pol+1
                 if self.cont_pol==(10.0*self.fc):
                     self.polishing=0
                     self.up=1
             elif self.up==1:
                 self.cont_pol=0
                 self.inc=0
                 self.ref_inc=np.zeros((6,1))
                 self._p_ref_L4=self.p_ref_attack[:]
           else:
             print "Fuerza accionada"
             self.fase2=0
             self.inc=0
             self.ref_inc=np.zeros((6,1))
             self._p_ref_L4=self.p_ref_attack[:]
             if self.up==1:
                 self.polished=1
                 self.cont_pol=0
                 self.up=0
             if self.down==1 or self.polishing==1:
                 self.down=0
                 self.polishing=0
                 
                 
                 
         
                 
         
         self._SMC_acc_Cartesian(self.Jn,self.Jrec) 
         
         self.q_ant = np.array(self.q).reshape((6,1))
         self.sigma_dist_ant=self.sigma_dist
         self.f_ant_FC=self.f_FC
         self.sigma_FC_ant=self.sigma_FC
         self.sigma_yaw_FC_ant=self.sigma_yaw_FC
         
         self.cont = self.cont + 1
         
         if np.absolute(msg.wrench.force.z) > 40 or np.absolute(msg.wrench.force.x) > 40 or np.absolute(msg.wrench.force.y) > 40:
             self._done = True

        
         new_msg=False
        else :
         print "Not published\n"
            
if __name__ == '__main__':
    rospy.init_node('KR6_SMC_new3')
    kr6_control = KR6controller()
    while kr6_control.new_msg==False:
        kr6_control.get_joint_info()
        sleep(0.1)
    kr6_control.get_joint_info()
    while kr6_control.new_cp==False or  kr6_control.new_norm==False:
        kr6_control.get_joint_info()
        sleep(0.1)
        
    kr6_control.get_joint_info()
    normal_temp=-kr6_control.normal_
    nRot=normal2rot(normal_temp)
    
    RotRef=R2rpy(nRot)
    if RotRef[0] < 0:
        RotRef[0] = RotRef[0] + 2.0*math.pi
    if RotRef[2] > 0:
        RotRef[2] = RotRef[2] - 2.0*math.pi
    for i in range(0,6,1):
        if i < 3:
            kr6_control._p_ref[i] = kr6_control.p_surf[i]
        else:
            kr6_control._p_ref[i] = RotRef[i-3]
     
     
    for i in range(0,6,1):
        if i < 3:
            kr6_control.p_ref_attack[i] = kr6_control.Twe[i,3]
        else:
            kr6_control.p_ref_attack[i] = RotRef[i-3]

    kr6_control._p_ref_L4=kr6_control.p_ref_attack[:]
    
    kr6_control.q_cmd = np.array(kr6_control.q).reshape((6,1))
    kr6_control.q_ant = np.array(kr6_control.q).reshape((6,1))
    
    kr6_control._done=False
    kr6_control.cont=0
    #Save data: header
    a = "Period " + str(1.0/kr6_control.fc) + "\n"
    a += "Time (1x1) " + "Sensor (1x6) " + "p_ref (1x6) " + "dist (1x1) " + "Sigma_dist (1x1) " + "Sigma_dist_d (1x1) " 
    a += "phi_dist (1x1) "  + "p (1x6) " + "pd (1x6) "
    a += "q (1x6) " + "qd (1x6) " + "joint_acc Level 1 (1x6) " + "joint_acc Level 2 (1x6) "
    a += "joint_acc Level 3 (1x6) " +  "joint_acc Total (1x6) " 
    a += "u_dist_plus (1x1) " +  "Kd_dist (1x1) "
    a += "orientation error (1x3) " +  "orientation d_error (1x3) " + "Kp orientation cc (1x1)" + "Kpd orientation cc (1x1)" + "Kpd L1cc orientation (1x1)" + "epsilon d (1x1)"
    a += "qd command (1x6) " +  "f FC (1x3) " + "sigma_FC (1x1) "+"sigma_FC_d (1x1) "+"sigma_FC_yaw (1x1) "+"sigma_FC_yaw_d (1x1) "
    a += "phi FC (1x2) " + "K FC diagonal (1x2)" + "H FC elements!=0 (1x4) " + "threshold FC (1x1)" + "threshold FC yaw (1x1)"
    a += "Kd FC (1x1) " + "Kd FC yaw (1x1) " + "diagonal W FC (1x2)" + "u plus FC (1x1)" + "p ref L5 (1x6)"
    a += "Kp L5 (1x1)" + "Kd L5 (1x1)" + "Kpd L5 cc (1x1)" + "q cmd (1x6)" + "guidance (1x1)"
    _time = 0.0
    while not rospy.is_shutdown() and not kr6_control._done:
     start = rospy.get_time()
     kr6_control.ctrl_actions()
     _waittime=0.01-(rospy.get_time()-start)
     if np.sign(_waittime)==1 :
      time.sleep(_waittime)
     _time += rospy.get_time() - start
     #Saving data:
     a += "\n"+ str(np.round(float(_time),4))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control._FH.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control._p_ref.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control.dist,decimals = 8))
     a += " " + str(np.round(kr6_control.sigma_dist,decimals = 8))
     a += " " + str(np.round(kr6_control.sigma_dist_d,decimals = 8))
     a += " " + str(np.round(kr6_control.phi_dist,decimals = 8))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.p.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.pd.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.q)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.qd.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.joint_acc.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.joint_acc_2.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.joint_acc_3.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.joint_acc_total.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control.u_dist_plus,decimals = 4))
     a += " " + str(np.round(kr6_control.Kd_dist,decimals = 4))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.error_orient.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.derror_orient.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control._Kp,decimals = 4))
     a += " " + str(np.round(kr6_control._Kpd,decimals = 4))
     a += " " + str(np.round(kr6_control._Kpd_L1cc,decimals = 4))
     a += " " + str(np.round(kr6_control.epsilon,decimals = 4))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.qp.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.f_FC.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control.sigma_FC,decimals = 8))
     a += " " + str(np.round(kr6_control.sigma_FC_d,decimals = 8))
     a += " " + str(np.round(kr6_control.sigma_yaw_FC,decimals = 8))
     a += " " + str(np.round(kr6_control.sigma_yaw_FC_d,decimals = 8))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.phi_FC.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.K_FC.diagonal())),decimals = 4)))
     a += " " + str(np.round(kr6_control.H_FC[0,0],decimals = 8))
     a += " " + str(np.round(kr6_control.H_FC[0,1],decimals = 8))
     a += " " + str(np.round(kr6_control.H_FC[0,2],decimals = 8))
     a += " " + str(np.round(kr6_control.H_FC[1,5],decimals = 8))
     a += " " + str(np.round(kr6_control.FC_thres,decimals = 8))
     a += " " + str(np.round(kr6_control.FC_yaw_thres,decimals = 8))
     a += " " + str(np.round(kr6_control.Kd_FC,decimals = 8))
     a += " " + str(np.round(kr6_control.Kd_yaw_FC,decimals = 8))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.W_FC.diagonal())),decimals = 4)))
     a += " " + str(np.round(kr6_control.u_FC_plus,decimals = 8))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control._p_ref_L4.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control._Kp_L4,decimals = 8))
     a += " " + str(np.round(kr6_control._Kpd_L4,decimals = 8))
     a += " " + str(np.round(kr6_control._Kpd_L4cc,decimals = 8))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.q_cmd.T)),decimals = 4)))
     a += " " + str(np.round(kr6_control.guidance,decimals = 1))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(kr6_control.Fnew.T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray((kr6_control._Cd.dot(kr6_control.Jn.dot(kr6_control.qd))).T)),decimals = 4)))
     a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray((kr6_control._Cd.dot(kr6_control.Jrec.dot(kr6_control.qd))).T)),decimals = 4)))
     
     
     kr6_control._saveData.write(a)
     a = ""