#!/usr/bin/env python
from numpy import pi,array
import openravepy as rave
from TransformMatrix import *
import time
from recorder import viewerrecorder
import datetime
import openhubo

def set_finger_torque(robot,maxT,dt=0.0005):
    names=[u'rightIndexKnuckle1', u'rightIndexKnuckle2', u'rightIndexKnuckle3', u'rightMiddleKnuckle1', u'rightMiddleKnuckle2', u'rightMiddleKnuckle3', u'rightRingKnuckle1', u'rightRingKnuckle2', u'rightRingKnuckle3', u'rightPinkyKnuckle1', u'rightPinkyKnuckle2', u'rightPinkyKnuckle3', u'rightThumbKnuckle1', u'rightThumbKnuckle2', u'rightThumbKnuckle3']
    #Rough calculation for now, eventually get this from finger models
    Iz0=0.000002
    maxA=30
    maxV=3

    for n in names:
        robot.GetJoint(n).SetTorqueLimits([maxT])
        #robot.GetJoint(n).SetVelocityLimits([maxV])
        #robot.GetJoint(n).SetAccelerationLimits([maxA])
        i=robot.GetJoint(n).GetDOFIndex()
        #TODO: Figure out actual finger stiffness?
        robot.GetController().SendCommand('set gainvec {} 100.0 0.0 0.0 '.format(i))

def get_timestamp(lead='_'):
    return lead+datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

def pause(t=-1):
    """ A simple pause function to emulate matlab's pause(t). 
    Useful for debugging and program stepping"""
    if t==-1:
        raw_input('Press any key to continue...')
    elif t>=0:
        time.sleep(t)
        
def makeNameToIndexConverter(robot):
    """ A closure to easily convert from a string joint name to the robot's
    actual DOF index, for use in creating/editing trajectories."""
    def convert(name):
        j=robot.GetJoint(name)
        if not(j==None):
            return robot.GetJoint(name).GetDOFIndex()
        else:
            return -1
    return convert


def load_scene(env,scenename=None,stop=False,physics=True):
    """ Load a robot model into the given environment, configuring a
    trajectorycontroller and a reference robot to show desired movements vs. actual
    pose. The returned tuple contains:
        robot: handle to the created robot
        controller: either trajectorycontroller or idealcontroller depending on physics
        name-to-joint-index converter
        ref_robot: handle to visiualization "ghost" robot
        recorder: video recorder python class for quick video dumps
    """

    # Set the robot controller and start the simulation
    recorder=viewerrecorder(env)
    #Default to "sim-timed video" i.e. plays back much faster
    recorder.videoparams[0:2]=[1024,768]
    recorder.realtime=False

    with env:
        if stop:
            env.StopSimulation()

        if type(scenename) is list:
            for n in scenename:
                loaded=env.Load(n)
        elif type(scenename) is str:
            loaded=env.Load(scenename)

    time.sleep(1)
    #Explicitly disable physics if option is selected
    with env:
        if not physics:
            env.SetPhysicsEngine(rave.RaveCreatePhysicsEngine(env,'GenericPhysicsEngine'))
        robot = env.GetRobots()[0]
        pose=ones(robot.GetDOF())*.6
        robot.SetDOFValues(pose)
        collisionChecker = rave.RaveCreateCollisionChecker(env,'pqp')
        if collisionChecker==None:
            collisionChecker = rave.RaveCreateCollisionChecker(env,'ode')
            print 'Note: Using ODE collision checker since PQP is not available'
        env.SetCollisionChecker(collisionChecker)

        if env.GetPhysicsEngine().GetXMLId()!='GenericPhysicsEngine' and physics:
            controller=rave.RaveCreateController(env,'servocontroller')
            robot.SetController(controller)
            controller.SendCommand('set gains 100 0 0')
            controller.SetDesired(pose/pose*pi/4*1.1)
    
    time.sleep(.5)
    ind=makeNameToIndexConverter(robot)

    return (robot,controller,ind,recorder)


if __name__=='__main__':
    from openravepy import *
    from servo import *
    env=Environment()
    env.SetDebugLevel(4)
    (env,options)=openhubo.setup('qtcoin')
    [robot,ctrl,ind,recorder]=load_scene(env,'gripper.env.xml',True,True)
    rod=env.GetKinBody('rod')
    trans=rod.GetTransform()
    pose=ones(robot.GetDOF())*.4
    pose[ind('rightIndexKnuckle1')]=.6
    pose[ind('rightMiddleKnuckle1')]=.6
    pose[ind('rightRingKnuckle1')]=.6
    pose[ind('rightPinkyKnuckle1')]=.6
    pose[ind('rightThumbKnuckle1')]=.6
    fail=False

    robot.SetDOFValues(pose)
    rod.SetLinkVelocities((zeros(6),zeros(6))) 
    rod.SetTransform(trans)
    T=.01
    set_finger_torque(robot,T)
    rod.Enable(False)
    
    ctrl.SetDesired(pose*2)
    print "Finger torque is {}".format(T)
    print "Mass is {}".format(rod.GetLinks()[0].GetMass())
    env.StartSimulation(timestep=0.0005)
