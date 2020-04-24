#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 11:58:37 2020

@author: jars
"""


import sys 
import numpy as np
import sim, simConst
import time


opmode=sim.simx_opmode_oneshot_wait

def get_joint_data(clientID):
    joints={}
    _, ids,_,_,names= sim.simxGetObjectGroupData(clientID,simConst.sim_object_joint_type, 0,opmode)
    for i in range(len(names)):
        joints[names[i]]= sim.simxGetObjectOrientation(clientID, ids[i],-1, opmode)[1]
    return joints

def get_effector_data(clientID):
    ef_id=sim.simxGetObjectHandle(clientID, 'effector', opmode)[1]
    return sim.simxGetObjectPosition(clientID, ef_id, -1, opmode)[1]

def rotX(theta):
    M=np.array([[1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)]])    
    return M
    
def rotY(theta):
    M=np.array([[np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta),0, np.cos(theta)]])    
    return M

def rotZ(theta):
    M=np.array([[np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta),0],
                [0, 0, 1]])    
    return M
    
def transform(theta, vec):
    M=np.ones([4,4])
    M[0:3, 0:3]=rotX(theta[0])*rotY(theta[1])*rotZ(theta[2])
    M[0,3]=vec[0]
    M[1,3]=vec[1]
    M[2,3]=vec[2]
    return M

def DH(theta,d, alpha,a):
    
    Td = np.ones([4,4])
    Td[2,3] = d
    Ta = np.ones([4,4])
    Ta[0,3] = a
    Rtheta = np.ones([4,4])
    Rtheta[0:3,0:3] = rotZ(theta)
    Ralpha = np.ones([4,4])
    Ralpha[0:3,0:3] = rotX(alpha)
    G = Td * Rtheta * Ta * Ralpha
    
    return G

sim.simxFinish(-1) #Terminar todas las conexiones
try:
    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim 
    if clientID!=-1:
        print ('Conexion establecida')
        res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
 
    else:
        sys.exit("Error: no se puede conectar") #Terminar este script
except:
    print('Check if CoppeliaSim is open')

l1,l2=0.3,0.15
#1.25
# while(True):
#     j1=get_joint_data(clientID)['j1'][2]
#     j2=get_joint_data(clientID)['j2'][2]
    
#     print("Real effector position :", np.round(get_effector_data(clientID), 3))
#     print("Geometrical estimation :", )
    
