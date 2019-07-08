#!/usr/bin/env python

import rospy
import threading

import math
from numpy import *
import numpy as np #import math library
from tf.transformations import euler_from_quaternion
#from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix
from numpy.linalg import norm

#from hrl_geom.transformations import quaternion_from_euler
from operator import itemgetter

class ForceSensorClass(object):

      def __init__(self):
          self._event = threading.Event()
          self._msg = None

      def __call__(self, msg):
          self._msg = msg
          self._event.set()
      
      def get_msg(self, timeout = None):
          self._event.wait(timeout)
          return self._msg

def Kuka_DenHartStandard_mod(ang,tool):

    DH = [[ ang[0],             -0.400,  0.025,    math.pi/2],
          [ ang[1],              0.000,  0.455,    0.0      ],
          [ ang[2] - math.pi/2,  0.000,  0.035,    math.pi/2],
          [ ang[3],             -0.420,  0.000,   -math.pi/2],
          [ ang[4],              0.000,  0.000,    math.pi/2],
          [ ang[5],             -0.080-tool,  0.000,    math.pi  ],
         ]
    return DH

def Kuka_fkine(DH,n_joints):

    T=[None]*n_joints
    Tf=np.identity(4)
    for i in range (0,n_joints):
        T[i]=_j_1_T_j(DH[i][0],DH[i][1],DH[i][2],DH[i][3])

    for i in range(0,n_joints):
        Tf=Tf*np.mat(T[i])
    return Tf

def _j_1_T_j(theta,d,a,alpha):
    
    T = [[math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta)],
         [math.sin(theta),math.cos(theta)*math.cos(alpha),-math.cos(theta)*math.sin(alpha),a*math.sin(theta)],
         [0,math.sin(alpha),math.cos(alpha),d],
         [0,0,0,1],
        ]
    
    return T

def Kuka_jacobn(joints,DH,n_joints):

    U = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    #J = mat([None]*n_joints)
    J = mat([[],[],[],[],[],[]])
            
    for i in range (n_joints-1,-1,-1):
        U = _j_1_T_j(DH[i][0],DH[i][1],DH[i][2],DH[i][3])*U
        d = matrix([[-U[0,0]*U[1,3] + U[1,0]*U[0,3]],\
                    [-U[0,1]*U[1,3] + U[1,1]*U[0,3]],\
                    [-U[0,2]*U[1,3] + U[1,2]*U[0,3]]])

        delta = U[2,0:3].T   # nz  oz  az

        J = concatenate((concatenate((d,delta)),J),1)

    return J

def Kuka_jacob0(joints,DH,n_joints):
    
    Jn = Kuka_jacobn(joints,DH,n_joints)
    Tn = Kuka_fkine(DH,n_joints)
    R = t2r(Tn)
    Jo = concatenate( ( concatenate( (R,zeros((3,3))) ,1) , concatenate( (zeros((3,3)),R) ,1) ))*Jn
    
    #for i in range (0,6,1):
        #Jo[i,3]=-Jo[i,3]
    
    return Jo

def t2r(T):

    try:
       return T[0:3,0:3]
    
    except:
       return np.array(T)[0:3,0:3]
   
def R2rpy(m):
   # print "m:\n"
   # print m
    for i in range (0,3):
     for j in range (0,3):
         if abs(m[i,j])<1e-4:
             m[i,j]=0
             
    try:
        m00 = m[0][0] 
        m01 = m[0][1] 
        m02 = m[0][2]
        m10 = m[1][0] 
        m11 = m[1][1] 
        m12 = m[1][2]
        m20 = m[2][0] 
        m21 = m[2][1] 
        m22 = m[2][2]
    except:
        m00 = m[0,0] 
        m01 = m[0,1] 
        m02 = m[0,2]
        m10 = m[1,0] 
        m11 = m[1,1] 
        m12 = m[1,2]
        m20 = m[2,0] 
        m21 = m[2,1] 
        m22 = m[2,2]
    
        
    rpy = [0,0,0]
    
    if norm(m22)<finfo(float).eps and norm(m12)<finfo(float).eps:
       # singularity
       rpy[0] = 0
       rpy[1] = arctan2(m02, m22)
       rpy[2] = arctan2(m10, m11)
      # print "singularity\n"
      # print rpy
    else:
       rpy[0] = arctan2(-m12,m22)
       sr = sin(rpy[0])
       cr = cos(rpy[0])
       rpy[1] = arctan2(m02, cr*m22 - sr*m12)
       rpy[2] = arctan2(-m01, m00)
       #print "no singularity\n"
      # print rpy
    
    return rpy

def jacobn_rec(JO,RPY):

    m = [[1,           0,              sin(RPY[1])],\
         [0, cos(RPY[0]), -sin(RPY[0])*cos(RPY[1])],\
         [0, sin(RPY[0]),  cos(RPY[0])*cos(RPY[1])]]

    #print "matr: "
    #print m

    Jn_rec = np.mat(concatenate( (concatenate((np.eye(3), zeros((3,3))),1), concatenate((zeros((3,3)), np.linalg.pinv(m,rcond=0.015)),1))))*np.mat(JO)

    return Jn_rec

def normal2rot (n):
    
    #Ref is -n
    nx=n[0]
    ny=n[1]
    nz=n[2]
    
    R=np.eye(3)
    
    r13=nx
    r23=ny
    r33=nz
    den=sqrt(nx*nx+nz*nz)
    
    r12=-nx*ny/den
    r22=((nx*nx)+(nz*nz))/den
    r32=-ny*nz/den
    
    r11=nz/den
    r21=0
    r31=-nx/den
    
    R[0,0]=r11
    R[0,1]=r12
    R[0,2]=r13
    R[1,0]=r21
    R[1,1]=r22
    R[1,2]=r23
    R[2,0]=r31
    R[2,1]=r32
    R[2,2]=r33
    
    
    return R
    
    