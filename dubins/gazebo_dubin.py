#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import numpy as np
import math
import sys

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

class A():
	def __init__(self):
		rospy.init_node('odometry')
		odom_sub=rospy.Subscriber('/odom',Odometry, self.callback)
		self.rate = rospy.Rate(10)
		self.PI=np.pi

	def callback(self, msg):
		self.current_x = round(msg.pose.pose.position.x, 4)
		self.current_y = round(msg.pose.pose.position.y, 4)
		self.q_w = round(msg.pose.pose.orientation.w ,4)
		self.q_x = round(msg.pose.pose.orientation.x ,4)
		self.q_y = round(msg.pose.pose.orientation.y ,4)
		self.q_z = round(msg.pose.pose.orientation.z ,4)
		siny = +2.0 * (self.q_w * self.q_z + self.q_x * self.q_y);
		cosy = +1.0 - 2.0 * (self.q_y * self.q_y + self.q_z * self.q_z);  
		yaw = math.atan2(siny, cosy);
		self.current_th = yaw

	def LSL(self):
		c1_x=self.current_x-self.r_min*math.cos((self.current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=self.current_y-self.r_min*math.sin((self.current_th)-(self.PI/2))		

		c2_x=self.goal_x-self.r_min*math.cos(math.radians(self.goal_th)-(self.PI/2))		#center of second circle of min_turning radius
		c2_y=self.goal_y-self.r_min*math.sin(math.radians(self.goal_th)-(self.PI/2))
		#print c1_x,c1_y,c2_x,c2_y, c2_y-c1_y, c2_x-c1_x
		theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x=c1_x+self.r_min*math.cos(theta)		#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y-self.r_min*math.sin(theta)

		tan_pt2_x=c2_x+self.r_min*math.cos(theta)		#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y-self.r_min*math.sin(theta)

		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		Ti = np.arctan2(self.current_x-c1_x,self.current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(self.goal_x - c2_x,self.goal_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi
		ans=[Ti,Tf, c1_x,c2_x,c1_y,c2_y,tan_pt1_x,tan_pt2_x,tan_pt1_y,tan_pt2_y,1,1,total_dist,S]

		return ans

	def RSR(self):
		c1_x=self.current_x+self.r_min*math.cos((self.current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=self.current_y+self.r_min*math.sin((self.current_th)-(self.PI/2))

		c2_x=self.goal_x+self.r_min*math.cos(math.radians(self.goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=self.goal_y+self.r_min*math.sin(math.radians(self.goal_th)-(self.PI/2))

		theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x=c1_x-self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y+self.r_min*math.sin(theta)

		tan_pt2_x=c2_x-self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.r_min*math.sin(theta)

		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(self.current_x-c1_x,self.current_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(self.goal_x - c2_x,self.goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle
		
		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, c1_x,c2_x,c1_y,c2_y,tan_pt1_x,tan_pt2_x,tan_pt1_y,tan_pt2_y,-1,-1,total_dist,S]
		return ans

	def LSR(self):
		c1_x=self.current_x-self.r_min*math.cos((self.current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=self.current_y-self.r_min*math.sin((self.current_th)-(self.PI/2))

		c2_x=self.goal_x+self.r_min*math.cos(math.radians(self.goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=self.goal_y+self.r_min*math.sin(math.radians(self.goal_th)-(self.PI/2))

		if c2_x-c1_x<0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*self.r_min))
		elif c2_x-c1_x>0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*self.r_min))

		tan_pt1_x=c1_x+self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y-self.r_min*math.sin(theta)

		tan_pt2_x=c2_x-self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.r_min*math.sin(theta)


		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

		S = 2*((-self.r_min**2 + (S1**2)/4.0 )**0.5)

		Ti = np.arctan2(self.current_x-c1_x,self.current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(self.goal_x - c2_x,self.goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, c1_x,c2_x,c1_y,c2_y,tan_pt1_x,tan_pt2_x,tan_pt1_y,tan_pt2_y,1,-1,total_dist,S]
		return ans

	def RSL(self):
		c1_x=self.current_x+self.r_min*math.cos((self.current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=self.current_y+self.r_min*math.sin((self.current_th)-(self.PI/2))

		c2_x=self.goal_x-self.r_min*math.cos(math.radians(self.goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=self.goal_y-self.r_min*math.sin(math.radians(self.goal_th)-(self.PI/2))

		if c2_x-c1_x<0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*self.r_min))
		elif c2_x-c1_x>0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*self.r_min))

		tan_pt1_x=c1_x-self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y+self.r_min*math.sin(theta)

		tan_pt2_x=c2_x+self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y-self.r_min*math.sin(theta)

		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5	#Distance from (c1_x,c1_y) to (c2_x,c2_y)

		S = 2*((-self.r_min**2 + (S1**2)/4.0 )**0.5)
		
		Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(self.current_x-c1_x,self.current_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(self.goal_x - c2_x,self.goal_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, c1_x,c2_x,c1_y,c2_y,tan_pt1_x,tan_pt2_x,tan_pt1_y,tan_pt2_y,-1,1,total_dist,S]
		return ans
	def run(self):
		self.goal_x = input("x goal:")
		self.goal_y = input("y goal:")
		self.goal_th = input("theta goal: (Input in Degrees )")		

		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		#rospy.init_node('p3dx_mover')

		twist = Twist()
		
		self.r_min = 2
		
		#samp=self.LSL()
			
		

		paths=[self.LSL,self.RSR,self.LSR,self.RSL] 
		distance=sys.maxint
		ans=[]
		count=0
		ans_count=1
		path=""
		for i in paths:
			path_ans=i()
			count+=1
			if distance>path_ans[-2]:
				ans=path_ans
				distance=path_ans[-2]
				ans_count=count

		#print "Optimal Path: " ,paths[ans_count-1], "Total distance to cover: ",distance
		ans=self.LSL()																		#SELECT THE PATH HERE
		Ti=ans[0]
		Tf=ans[1]
		c1_x=ans[2]
		c2_x=ans[3]
		c1_y=ans[4]
		c2_y=ans[5]
		tan_pt1_x=ans[6]
		tan_pt2_x=ans[7]
		tan_pt1_y=ans[8]
		tan_pt2_y=ans[9]
		LoR1=ans[10]
		LoR2=ans[11]
		# c1_x=self.current_x-r_min*cos(self.current_th-self.PI/2)
		# c1_y=self.current_y-r_min*sin(self.current_th-self.PI/2)

		# c2_x=goal_x-r_min*cos(goal_th-self.PI/2)
		# c2_y=goal_y-r_min*sin(goal_th-self.PI/2)

		# theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

		# tan_pt1_x=c1_x+r_min*cos(theta)
		# tan_pt1_y=c1_y-r_min*sin(theta)

		# tan_pt2_x=c2_x+r_min*cos(theta)
		# tan_pt2_y=c2_y-r_min*sin(theta)
		

		# Ti = np.arctan2(self.current_x-c1_x,self.current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)

		# Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)

		# if Ti<0:
		# 	Ti+=2*np.pi
		# if Tf<0:
		# 	Tf+=2*np.pi

		# print c1_x,c1_y,tan_pt1_x,tan_pt1_y, tan_pt2_x,tan_pt2_y,Ti,Tf
		print ans
		dele=input("checkpoint (Enter any Number to continue)");
		print "initial configuration: ", self.current_x,self.current_y,self.current_th*180/self.PI		

		linear_speed=0.2
		angular_speed=0.2/(self.r_min/0.808)
		
		twist.linear.x = 0
		twist.linear.y =0
   		twist.linear.z =0
   		twist.angular.x = 0
   		twist.angular.y = 0
   		twist.angular.z = 0
		
		twist.linear.x = abs(linear_speed)
		if LoR1==1:
			twist.angular.z = abs(angular_speed)
		elif LoR1==-1:
			twist.angular.z = -abs(angular_speed)

		# print ang_total

		err = math.sqrt( math.pow((tan_pt1_x-self.current_x),2) + math.pow((tan_pt1_y-self.current_y),2) )

		"""while (err>0.01):
			pub.publish(twist)
			print self.current_x,",",self.current_y, "error", err
			err = sqrt( pow((tan_pt1_x-self.current_x),2) + pow((tan_pt1_y-self.current_y),2) )
			self.rate.sleep()"""
		errorf= math.sqrt( math.pow((tan_pt1_x-self.current_x),2) + math.pow((tan_pt1_y-self.current_y),2) )
		curr_x = self.current_x
		curr_y = self.current_y
		curr_yaw=self.current_th
		error_angle=(curr_yaw+Ti)*180/self.PI
		t1=rospy.get_time()
		while (errorf>0.1):
			pub.publish(twist)
			curr_x = self.current_x
			curr_y = self.current_y
			error_angle-=self.current_th
			print curr_x,",",curr_y, "error", errorf, error_angle
			errorf= math.sqrt( math.pow((tan_pt1_x-curr_x),2) + math.pow((tan_pt1_y-curr_y),2) )
			self.rate.sleep()

		twist.linear.x =0
		twist.angular.z = 0
		pub.publish(twist)
		
		print "configuration after first curve: ", self.current_x,self.current_y,self.current_th		
		dele=input("checkpoint (Enter any Number to continue)");
		#exit()

		twist.linear.x = linear_speed

		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5
		distance=S*0.808#7.107#sqrt( pow((tan_pt2_x-tan_pt1_x),2) + pow((tan_pt2_y-tan_pt1_y),2) )
		distance_covered=0
		t1=rospy.get_time()
		while (distance_covered<distance):
			pub.publish(twist)
			t2=rospy.get_time()
			distance_covered+=linear_speed*(t2-t1)
			errorf= math.sqrt( math.pow((tan_pt2_x-self.current_x),2) + math.pow((tan_pt2_y-self.current_y),2) )
			if errorf<0.1:
				break
			#print distance_covered	
			print self.current_x,self.current_y,errorf	
			self.rate.sleep()
			t1=t2
		twist.linear.x = 0
		pub.publish(twist)
		
		print "configuration after curve and straight line: ", self.current_x,self.current_y,self.current_th, distance_covered
		

		dele=input("checkpoint (Enter any Number to continue)");
		twist.linear.x = abs(linear_speed)
		if LoR2==1:
			twist.angular.z = abs(angular_speed)
		elif LoR2==-1:
			twist.angular.z = -abs(angular_speed)


		# print ang_total

		errorf= math.sqrt( math.pow((tan_pt2_x-self.current_x),2) + math.pow((tan_pt2_y-self.current_y),2) )
		while (errorf>0.1):
			pub.publish(twist)
			err_goal = math.sqrt( math.pow((self.goal_x-self.current_x),2) + math.pow((self.goal_y-self.current_y),2) )
			if err_goal<0.5:
				break
			print self.current_x,",",self.current_y, "error", err_goal,errorf
			errorf= math.sqrt( math.pow((tan_pt2_x-self.current_x),2) + math.pow((tan_pt2_x-self.current_y),2) )
			self.rate.sleep()

		twist.linear.x =0
		twist.angular.z = 0
		pub.publish(twist)
		
		print "final configuration: ", self.current_x,self.current_y,self.current_th


a=A()
a.run()