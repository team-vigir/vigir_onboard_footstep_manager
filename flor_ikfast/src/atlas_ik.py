#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Shows how to use lookat inverse kinematics to have multiple robots maintain line of sight while avoiding collisions.

.. examplepre-block:: tutorial_iklookat_multiple

.. examplepost-block:: tutorial_iklookat_multiple
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

from itertools import izip
import time
import sys
import random as stdrandom
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from openravepy import ikfast

from openravepy.misc import sequence_cross_product

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robots = env.GetRobots()
    print "Robots:",robots

    robot = env.GetRobot('atlas')

    manips = [robot.GetManipulator('atlas_left_leg')]

    print "Manips:",manips
    #print "    Robot      :",manips[0]
    #print "    Base       :",manips[0].getBase()
    #print "    EndEffector:",manips[0].GetEndEffector()

    #kinbody = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    #env.Add(kinbody)

    print "Solver ..."
    solver = ikfast.IKFastSolver(kinbody=robot)
    print "solver=",solver
    print "solvers:"
    for isolv,solv in enumerate(solver.GetSolvers()):
        print isolv," : ",solv

    # Generate the requested IK solution
    chaintree = solver.generateIkSolver(baselink=0,eelink=33,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_6D)

    print "Write solver ..."
    code = solver.writeIkSolver(chaintree)

    # Define a preamble to C++ file that lists the robot info
    preamble = "//------------- Atlas Robot -----------------------\n"
    links = robot.GetLinks();
    joints = robot.GetJoints();
    #print "    Robot      :",robot
    preamble += "//   Links:\n"
    for ilink,link in enumerate(links):
        preamble += "//      "+ str(ilink)+" : "+str(link)+"\n"
    preamble += "//   Joints    :\n"
    for ijoint,joint in enumerate(joints):
        preamble += "//      "+ str(ijoint)+" : "+str(joint)+"\n"
    preamble += "//-------------------------------------------------\n"
    open('ik_pelvis_left_foot.cpp','w').write(preamble+code)
    print "exit"
    sys.exit(0);

    print "while True"
    while True:
        maxsolutions = 40
        goodsolutions = []
        with env:
            # move the robot in a random collision-free position and call the IK
            while True:
                target=ikmodel.manip.GetTransform()[0:3,3]+(random.rand(3)-0.5)
                robotsolutions = []
                for ikmodel in ikmodels:
                    for ikmodel2 in ikmodels:
                        ikmodel2.robot.Enable(ikmodel==ikmodel2)
                    solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.Lookat3D),IkFilterOptions.CheckEnvCollisions)
                    if len(solutions) == 0:
                        break
                    robotsolutions.append(solutions)
                for ikmodel2 in ikmodels:
                    ikmodel2.robot.Enable(True)
                if len(robotsolutions) == len(robots):
                    print 'found solutions for all manipulators, search for a joint collision-free one'
                    goodsolutions = []
                    # permute randomly to get more interesting solutions
                    allsols = [sols for sols in sequence_cross_product(*robotsolutions)]
                    stdrandom.shuffle(allsols)
                    for sols in allsols:
                        for ikmodel,sol in izip(ikmodels,sols):
                            ikmodel.robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
                        if not any([ikmodel.robot.CheckSelfCollision() or env.CheckCollision(ikmodel.robot) for ikmodel in ikmodels]):
                            goodsolutions.append(sols)
                            if len(goodsolutions) >= maxsolutions:
                                break
                    if len(goodsolutions) > 0: # found solutions, so break!
                        break

        handles = [env.plot3(array([target]),20.0)]
        for sols in goodsolutions:
            handlerays = []
            with env:
                for ikmodel,sol in izip(ikmodels,sols):
                    ikmodel.robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
                    T = ikmodel.manip.GetTransform()
                    globaldir = numpy.dot(T[0:3,0:3],ikmodel.manip.GetDirection())
                    dist = linalg.norm(T[0:3,3]-target)+0.4
                    handlerays.append(env.drawlinelist(array([T[0:3,3], T[0:3,3]+dist*globaldir]),5,colors=[0.1,0.1,1]))
                env.UpdatePublishedBodies()
            time.sleep(0.1)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to use different IK solutions for arms with few joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='../robot/atlas_left_leg.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
