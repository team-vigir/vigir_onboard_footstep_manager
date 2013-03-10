#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2013 David Conner (conner@torcrobotics.com)
#
#based on tutorial code
#.. examplepre-block:: tutorial_iklookat_multiple
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
#

from __future__ import with_statement # for python 2.5
__author__ = 'David Conner'

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
    print "Loading scene <",options.scene,">"
    env.Load(options.scene)
    robots = env.GetRobots()
    print "Robots:",robots

    robot = env.GetRobot('atlas')

    print "Solvers ..."
    solver = ikfast.IKFastSolver(kinbody=robot)
    for isolv,solv in enumerate(solver.GetSolvers()):
        print isolv," : ",solv

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

    print preamble


    #print "Press Enter to continue"
    #raw_input();

    # Generate the requested IK solution
    print "Generate solution from pelvis to left foot ..."
    chaintree = solver.generateIkSolver(baselink=0,eelink=33,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
    code = solver.writeIkSolver(chaintree)
    open('ik_pelvis_left_foot.cpp','w').write(preamble+code)

    print "Generate solution from right foot to pelvis ... "
    chaintree = solver.generateIkSolver(baselink=39,eelink=0,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
    code = solver.writeIkSolver(chaintree)
    open('ik_right_foot_pelvis.cpp','w').write(preamble+code)

    # Generate the requested IK solution
    print "Now try the left arm ..."

    print "Not writing symbolic code for this solver yet"
    chaintree = solver.generateIkSolver(baselink=4,eelink=9,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_TranslationDirection5D) #solveFullIK_6D)
    code = solver.writeIkSolver(chaintree)
    open('ik_clav_left_hand.cpp','w').write(preamble+code)

    print "Now try the right arm ..."
    #chaintree = solver.generateIkSolver(baselink=3,eelink=25,freeindices=[],solvefn=ikfast.IKFastSolver.solveFullIK_Translation3D) #.solveFullIK_TranslationDirection5D) #solveFullIK_6D)
    #code = solver.writeIkSolver(chaintree)
    #open('ik_utorso_right_arm.cpp','w').write(preamble+code)

    print "exit"
    sys.exit(0);


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
