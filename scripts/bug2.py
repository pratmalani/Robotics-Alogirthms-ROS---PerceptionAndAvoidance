#!/usr/bin/env python3
import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

def laser_callback(msgs):
	global orien
	global pos
	pos=msgs.pose.pose.position
	orien=msgs.pose.pose.orientation

def callback(msgs):
	global front   
	global left    
	r=msgs.ranges
	center_front=0
	center_left=0
	i=100
	for i in range(100):
		if(r[120+i]<1):
			center_front+=1
	for i in range(90):
		if(r[i]<1):
			center_left+=1
	front=center_front>1
	left=center_left>10

def bug2():
	global pos
	global orien
	global front
	global right_left

	pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
	sub=rospy.Subscriber("base_pose_ground_truth",Odometry,laser_callback)
	threshold=0.85 
	flag="GOAL_SEEK"
	rate=rospy.Rate(10)
	target=False   
	
	while(target!=True):
		if(orien!=0):
			a = (endpoints[1,0]-pos.x)
			b = (endpoints[1,1]-pos.y)
			z1=math.sqrt(a**2 + b**2)
			angle=math.atan(a/b) - (2*np.arcsin(orien.z))
			aplha=endpoints[0,:]
			beta=endpoints[1,:]
			gamma=np.array([pos.x,pos.y])

			available_area=abs(0.5*((aplha[0]-gamma[0])*(beta[1]-aplha[1])-(aplha[0]-beta[0])*(gamma[1]-aplha[1])))
			
			right_left=available_area<threshold
			tw=Twist()

			if (z1>=threshold):
				if(front==True):   
					tw.linear.x=0
				else:
					tw.linear.x=1
				bot=0     

				if(flag=="GOAL_SEEK") and (right_left==True):  
					bot=min(angle,1)
				elif(flag=="GOAL_SEEK") and (front==True):
					bot=1
				elif(flag=="GOAL_SEEK") and (right_left!=True)and (front!=True):
					bot=min(angle,1)
				tw.angular.z=bot

				if((flag=="GOAL_SEEK")  and ((front==True) or (left==True))):
					flag="WALL_FOLLOW"

				elif (flag!="GOAL_SEEK") and (front==True):
					bot =0.5
				elif(flag !='GOAL_SEEK') and (front!=True) and (left==True):
					bot = 0
				elif(flag !='GOAL_SEEK') and (front!=True) and (left!=True):
					bot=-1*0.5
				if (flag !='GOAL_SEEK'):
					tw.angular.z=-1*bot

				if(flag!="GOAL_SEEK") and ((right_left==True) and front!=True): 
						flag="GOAL_SEEK"
			if(z1<threshold):    
				tw.linear.x=0
				tw.linear.y=0
				tw.linear.z=0
				target=True   
				break
			pub.publish(tw)
			rate.sleep()		

if __name__ == '__main__':
		rospy.init_node('GOAL_SEEK',anonymous=True)
		orien=0
		endpoints=np.array([[-8,-2],[4.5,9.0]])
		front=False
		left=False
		right_left=False

		rospy.init_node('GOAL_SEEK',anonymous=True)
		rospy.Subscriber("base_scan",LaserScan,callback)
		bug2()
