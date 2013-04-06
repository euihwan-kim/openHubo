""" Simple test script to run some of the functions above. """

# boilerplate openhubo imports, avoid "import *" to allow pylint to check for
# undefined functions
import openhubo
from numpy import pi
from openravepy import planningutils

#example-specific imports
import openhubo.trajectory as trajectory

(env,options)=openhubo.setup()
env.SetDebugLevel(3)

timestep=openhubo.TIMESTEP

[robot,controller,ind,ref,recorder]=openhubo.load_scene(env,options)

pose0=openhubo.Pose(robot,controller)
pose0.update()
pose0.send()

env.StartSimulation(timestep=timestep)

pose1=openhubo.Pose(robot,controller)

pose1['LAP']=-pi/8
pose1['RAP']=-pi/8

pose1['LKP']=pi/4
pose1['RKP']=pi/4

pose1['LHP']=-pi/8
pose1['RHP']=-pi/8

[traj,config]=trajectory.create_trajectory(robot)

#Note the new waypoint-building syntax
traj.Insert(0,pose0.to_waypt(dt=0.0))
traj.Insert(1,pose1.to_waypt(dt=1.0))
traj.Insert(2,pose0.to_waypt(dt=1.0))

#Need to do this to add timing information for interpolating
planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

print 'Dump all DOFs to youngbum format'
trajectory.write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum.traj')

print 'Only use a selection of DOF\'s instead of everything'
trajectory.write_youngbum_traj(traj,robot,0.005,'traj_example_youngbum2.traj',dofs=range(28))

print 'Write to hubo-read-trajectory compatible format'
trajectory.write_hubo_traj(traj,robot,0.025,'traj_example_hubo.traj')

print 'Test reading of trajectories'
traj_in_yb=trajectory.read_youngbum_traj('traj_example_youngbum2.traj',robot,0.005)
traj_in_text=trajectory.read_text_traj('traj_example_youngbum2.traj',robot,0.005)
