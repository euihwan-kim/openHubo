from openhubo import *
from openravepy import *
from openhubo import comps

#Enable interactive mode and load a simple environment
(env,options)=setup('qtcoin')
env.SetDebugLevel(3)

#Options structure is populated by command line as well as easily in code
[robot,ctrl,ind,ghost,recorder]=load_scene_from_options(env,options)

pose=Pose(robot,ctrl)

env.StartSimulation(TIMESTEP)
