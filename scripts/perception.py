#!/usr/bin/env python3


import roslib
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import random
import math

def laser_callback(data):
	pub = rospy.Publisher('/ransac_vis',Marker, queue_size = 10)
	sub = rospy.Subscriber('/base_scan',LaserScan,laser_callback)
	m = Marker()
	m.header.frame_id = "/base_link"
	m.ns="ransac"
	m.id= 0
	m.type=Marker.LINE_LIST
	m.action = Marker.ADD
	
	m.pose.orientation.x = 0.0
	m.pose.orientation.y = 0.0
	m.pose.orientation.z = 0.0
	m.pose.orientation.w = 1.0
	m.scale.x = 0.1
	m.color.r = 1.0
	m.color.a = 1.0	

	lista=[]
	for i in range(120,240):
		lista.append(data.ranges[i])
	a=[], theta = [], x =[], y =[]
	x3=[],y3=[]
	sample_angle=-1
	for i in range(0,360):
		a.append(data.ranges[i])
		theta.append(sample_angle)
		sample_angle=sample_angle+0.009
		Xo = a[i] * math.cos(theta[i])
		Yo = a[i] * math.sin(theta[i])	
		x.append(Xo)
		y.append(Yo)
		
		if a[i]<1.8:
			x3.append(Xo)   
			y3.append(Yo)			
			
	r=len(x3)
	itr = 50
	if r!=0:
		max_dis=0
		for j in range(itr):
			p1, p2=random.randint(0,r-1), random.randint(0,r-1)
			if p1!=p2:
				rand_p=[(x3[p1],y3[p1]),(x3[p2],y3[p2])]
				x_cordinate, y_cordinate = zip(*rand_p)
				x1 = (y_cordinate[0] - y_cordinate[1])/(x_cordinate[0] - x_cordinate[1])
				y1 = y_cordinate[0] - x1*x_cordinate[0]
				inliers, outliers=0, 0
				x4, y4=[], []
				x5, y5=[], []
				for i in range(r-1):	
					dist = abs((x1*x3[i])+(-1*y3[i])+y1)/(x1*x1+1)**0.5
					if dist>=0.5:
						outliers += 1
						x5.append(x3[i])
						y5.append(y3[i])
					else:
						inliers +=1
						x4.append(x3[i])
						y4.append(y3[i])
						
				if inliers>max_dis:
					max_dis=inliers
					x6=x4[:]
					y6=y4[:]			
		if p1!=p2:
			d=len(x6)
			max_dis,maxi_dis,maxj_dis = 0
			for i in range(0,d-1):
				for j in range(0,d-1):	
					distance = math.hypot(x6[i]-x6[j], y6[i]-y6[j])
					if distance>max_dis:
						max_dis=distance
						maxi_dis=i
						maxj_dis=j
			start = maxi_dis
			end = maxj_dis
			m.id=1

			endpoints=[]	
			point1=Point()
			point1.x=x6[start]
			point1.y=y6[start]
			point1.z=0
			endpoints.append(point1)	
			point2=Point()
			point2.x=x6[end]
			point2.y=y6[end]
			point2.z=0
			endpoints.append(point2)
			m.points=endpoints
			pub.publish(m)
					
def Ransac():
	rospy.init_node('ransac',anonymous=True)
	rate = rospy.Rate(10) 

if __name__ =="__main__":
		Ransac()
	
	

	