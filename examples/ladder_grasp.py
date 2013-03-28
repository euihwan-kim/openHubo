#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

#Basics for OpenRAVE
from openravepy import *
from numpy import *
from numpy.linalg import inv

#useful libraries
from str2num import *
import time
import datetime
import sys
import os

#comps-plugins
from rodrigues import *
from TransformMatrix import *
from TSR import *

#OpenHubo python modules
import openhubo
from generalik import *
from cbirrt import *
from openhubo import pause
from planning import *

def makeGripTransforms(links):
    grips = []
    for k in range(len(links)):
        print links[k]
        T = mat(links[k].GetTransform())
        if links[k].GetName()[:-1] == 'post':
            for j in range(8):
                #create one grip above each rung
                grips.append(T*(MakeTransform(mat(eye(3)),
                                              (mat([0,0,.3])*j+mat([0,0,.07])).T)))
        elif links[k].GetName()[:-1] == 'rung':
            grips.append(T*(MakeTransform(mat(eye(3)),
                                          mat([0,.12,0]).T)))
            grips.append(T*(MakeTransform(mat(eye(3)),
                                          mat([0,-.12,0]).T)))
    return grips

if __name__=='__main__':

    (env,options)=openhubo.setup('qtcoin')
    env.SetDebugLevel(3)

    # Load the environment
    [robot, ctrl, ind,ref,recorder]=openhubo.load_scene(env,options.robotfile,'ladderclimb.env.xml',True)
    pose=zeros(robot.GetDOF())
    robot.SetDOFValues(pose)

    print "Position the robot"
    #pause()
    stairs=env.GetKinBody('ladder')
    links=stairs.GetLinks()
    ind=openhubo.makeNameToIndexConverter(robot)
    #Make any adjustments to initial pose here
    handles=[]
    for k in links:
        handles.append(openhubo.plotBodyCOM(env,k))
    #pause()
    
    grips = makeGripTransforms(links) 
    griphandles=plotTransforms(env,grips,array([0,0,1]))
    
    #pause()
    # make a list of Link transformations
    
    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    
    env.LoadProblem(probs_cbirrt,robot.GetName())
    
    setInitialPose(robot)
    time.sleep(1)
    
    #Define manips used and goals
    z1=.05
    theta=0.5
    LH=0    
    RH=8
    POST=8
    RUNG0=16
    RUNG=2
    LF=0
    RF=1

    #Post grips at shoulder height
    rgrip1=TSR(grips[3+RH],MakeTransform(eye(3),matrix([.0,-.015,0]).T),mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),1)
    lgrip1=TSR(grips[3],MakeTransform(eye(3),matrix([.0,.015,0]).T),mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),0)
    
    #Rung grips at shoulder height
    rgrip2=TSR(grips[RUNG0+2*RUNG+RH],MakeTransform(eye(3),matrix([.0,-.015,0]).T),mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),1)
    lgrip2=TSR(grips[RUNG0+2*RUNG],MakeTransform(eye(3),matrix([.0,.015,0]).T),mat([0,0, 0,0, -z1,z1, 0,0 ,0,0, -theta,theta]),0)
    
    #Step onto first rung
    lstep1=TSR(grips[RUNG0+LF],MakeTransform(eye(3),matrix([.0,0,.014]).T),mat([0,0, 0,0, 0,0, 0,0 ,-pi/3,0, 0,0]),2)
    rstep1=TSR(grips[RUNG0+RF],MakeTransform(eye(3),matrix([.0,0,0.014]).T),mat([0,0, 0,0, 0,0, 0,0 ,-theta,theta, 0,0]),3)

    # Define keyframe poses in terms of manips
    pose1={'rightArm':rgrip1,'leftArm':lgrip1,'leftFoot':lstep1}
    pose2={'rightArm':rgrip2,'leftArm':lgrip2,'leftFoot':lstep1}

    print "Place the robot in the desired starting position"
    #TODO: Sample the Right foot floating base pose, passed in as a separate TSR chain?
    
    
    #solveWholeBodyPose(robot,probs_cbirrt,pose1)
    
    first_pose=Cbirrt(probs_cbirrt)

    chains=[TSRChain(0,1,0) for x in range(3)]
    chains[0].insertTSR(pose1['leftArm'])
    chains[1].insertTSR(pose1['rightArm'])
    chains[2]=MakeInPlaceConstraint(robot,'leftFoot')

    for c in chains:
        first_pose.insertTSRChain(c)

    #Activate useful DOFs
    activedofs=first_pose.ActivateManipsByIndex(robot,[0,1])
    #Add in hip pitches
    activedofs.append(ind('LHP'))
    activedofs.append(ind('RHP'))
    robot.SetActiveDOFs(activedofs) 

    first_pose.supportlinks=['leftFoot','rightFoot']
    first_pose.filename='firstpose.traj'
    print first_pose.Serialize()
    success=first_pose.run()
    openhubo.pause()
    
    RunTrajectoryFromFile(robot,first_pose)

    CloseLeftHand(robot,pi/3)
    CloseRightHand(robot,pi/3)

