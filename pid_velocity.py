#!/usr/bin/env python

#An implementation of a PID controller to control the velocity of prius
#Author: Shivam Sood
#Date: 31/08/2020

import rospy
from prius_msgs.msg import Control    #msg type for prius
from nav_msgs.msg import Odometry     
import math

target_vel = 3
vx, vy = 0,0
kp =1 		#Proportional Term
ki = 0.000034   #Integral Term
kd = 0.002      #Derivative Term (Note that delta_t is considered to be already multiplied with them hence the small values)
error,sum_error,diff_error =0,0,0 

def vel_callback(Odometry):
	global vx,vy,error,sum_error,diff_error
	vx = Odometry.twist.twist.linear.x      
	vy = Odometry.twist.twist.linear.y     #Get the x,y components of velocity of prius
	vel = math.sqrt(vx**2 + vy**2)       
	prev_error = error                     #For the  derivative term
	error = target_vel - vel
	sum_error += error
	dif_error = error-prev_error
	print(vel)

def pid_vel():
	rospy.init_node('move_car',anonymous=True)
	pub = rospy.Publisher('/prius', Control, queue_size =10)			#Topic on which prius' control message is published
	rospy.Subscriber('/base_pose_ground_truth' , Odometry, vel_callback)	#Returns the Odometry of the vehicle
	rate=rospy.Rate(10)								#Rate : 10Hz
	move = Control()								#Initialize a message instance 
	while not rospy.is_shutdown():
		val = kp*error+ki*sum_error+kd*diff_error				#Calculate the input
		#val = 1/(1+math.e**(-1*val))						#Could be mapped to 0 to 1 using a sigmoid fxn(Will need to change the tuning parameters)
		if val>1:
			val = 1
		if val<0:
			val = 0
		move.throttle =val
		pub.publish(move)

	rate.sleep()

if __name__ == '__main__':
	try:
		pid_vel()
	except rospy.ROSInterruptException:
		pass
