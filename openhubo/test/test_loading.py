#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__license__ = 'GPLv3 license'

import openravepy as _rave
from numpy import pi,zeros,sqrt
import time
import unittest

def model_test_factory(filename=None):
    class TestLoading(unittest.TestCase):
        def setUp(self):
            self.env=_rave.Environment()
            env=self.env
            with env:
                env.StopSimulation()
                print('\nTesting model {}'.format(filename))
                result=env.Load(filename)
                self.assertTrue(result)
                self.robot=env.GetRobots()[0]
                physics = _rave.RaveCreatePhysicsEngine(env,'ode')
                env.SetCollisionChecker(_rave.RaveCreateCollisionChecker(env,'pqp'))
                self.env.SetDebugLevel(1)
                physics.SetGravity([0,0,0])
                env.SetPhysicsEngine(physics)
                self.robot.SetDOFValues(zeros(self.robot.GetDOF()))

        def tearDown(self):
            with self.env:
                self.env.StopSimulation()
            self.env.Destroy()

        def test_loading(self):
            """Apply a physics engine and look for drift in joints. If joints
            move, then the physics engine is trying to resolve conflicting
            constraints.  This typically means that adjacent bodies are
            colliding and have not be declared to be officially adjacent.
            """

            self.env.StartSimulation(timestep=0.001)
            initialPose=self.robot.GetDOFValues()
            time.sleep(5)
            finalPose=self.robot.GetDOFValues()
            err=sqrt(sum(pow(initialPose-finalPose,2)))
            #Somewhat arbitrary tolerance here
            self.assertLess(err,0.0001)

        def test_zeroheight(self):
            """Make sure that the robot is correctly positioned so that it's
            lowest point is at or above the floor plane (XY plane).
            """
            aabb=self.robot.ComputeAABB()
            self.assertGreaterEqual(aabb.pos()[-1]-aabb.extents()[-1],0)


    return TestLoading

test_load1=model_test_factory('huboplus.robot.xml')
test_load2=model_test_factory('rlhuboplus.robot.xml')
test_load3=model_test_factory('rlhuboplus.noshell.robot.xml')
test_load4=model_test_factory('rlhuboplus.fingerless.robot.xml')
test_load5=model_test_factory('hubo2.robot.xml')
test_load6=model_test_factory('rlhubo2.robot.xml')
test_load7=model_test_factory('rlhuboplus.cushionhands.robot.xml')

if __name__=='__main__':

    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())
