#!/usr/bin/env python

import sys
import rospy
#from beginner_tutorials.srv import *
import time, math, struct
from numpy import *
#import osu_ros_adept 
from osu_ros_adept.srv import *

def send_command(params, joints):

	rospy.wait_for_service('send_robot_movements')
	try:
		send_cmd = rospy.ServiceProxy('send_robot_movements', robot_movement_command)
		temp = []
		for i in range(6):
			temp.append(joints[i])
		
		length = send_cmd(params[0], params[1], params[2], params[3], params[4], temp)
		print str(length) + " bytes sent"

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	#print "ch1"

#def send_parameters(params):

#	rospy.wait_for_service('send_robot_parameters')
#	try:
#		send_cmd = rospy.ServiceProxy('send_robot_parameters', robot_parameters_command)
#		length = send_cmd(params[0], params[1], params[2], params[3], params[4])
		
#		print str(length) + " bytes sent"

#	except rospy.ServiceException, e:
#		print "Service call failed: %s"%e
	#print "ch2"

#def set_speed(spd):
	
#	cmd = []
#	cmd.append(20)  #Message Length
#	cmd.append(11)  #Message Type
#	cmd.append(spd)  #Message Command
#	cmd.append(0)  #Message Reply 
#	cmd.append(0)  #Discard bytes

#	send_parameters(cmd)

def send_joints(pos):
	last = []
	#pos = [0, 0, 0, 0, 0, 0]
	last = pos
		
	f_angles = array(pos, dtype=float32)

	cmd = []
	cmd.append(60)  #Message Length
	cmd.append(1)  #Message Type
	cmd.append(0)  #Message Command
	cmd.append(0)  #Message Reply 
	cmd.append(0)  #Discard bytes

	send_command(cmd, f_angles)

	
	return last

def zero_all_joints():
	pos = [0, 0, 0, 0, 0, 0]
	send_joints(pos)
	print "bitch"

def shake_home_pos():
	pos = [0, -0.80, 2.49, 0, -0.45, 0]
	send_joints(pos)
	print "shit"

def shake_left():
#	set_speed(500)
	pos = [-0.65, -0.80, 2.49, 0, -0.45, 0.55]
	send_joints(pos)

def shake_right():
#	set_speed(500)
	pos = [0, -0.80, 2.49, -0.35, -0.10, -0.21]
	send_joints(pos)

if __name__ == "__main__":
	print "HI!!"
	zero_all_joints()
	print "1"
	#time.sleep(2)
	shake_home_pos()
	print "2"
	#time.sleep(2)
	shake_left()
	print "3"
	#time.sleep(2)
	#shake_right()
	#time.sleep(2)
	#shake_left()
	#time.sleep(2)
	#shake_right()
	#time.sleep(2)
	#set_speed(100)
	zero_all_joints()
	print "4"
	




