#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import numpy as np
import math
import random
import sys
import copy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

class Node():
	def __init__(self,x,y):
		self.x=x
		self.y=y
		self.p=None
		self.tan_pt1_x=None
		self.tan_pt1_y=None
		self.tan_pt2_x=None
		self.tan_pt2_y=None
		self.Ti=None
		self.S=None


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

	def LSL(self,current_x,current_y,current_th,goal_x,goal_y,goal_th):
		c1_x=current_x-self.r_min*math.cos((current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=current_y-self.r_min*math.sin((current_th)-(self.PI/2))		

		c2_x=goal_x-self.r_min*math.cos(math.radians(goal_th)-(self.PI/2))		#center of second circle of min_turning radius
		c2_y=goal_y-self.r_min*math.sin(math.radians(goal_th)-(self.PI/2))
		#print c1_x,c1_y,c2_x,c2_y, c2_y-c1_y, c2_x-c1_x
		theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x=c1_x+self.r_min*math.cos(theta)		#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y-self.r_min*math.sin(theta)

		tan_pt2_x=c2_x+self.r_min*math.cos(theta)		#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y-self.r_min*math.sin(theta)

		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi
		ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LL"]

		return ans

	def RSR(self,current_x,current_y,current_th,goal_x,goal_y,goal_th):
		c1_x=current_x+self.r_min*math.cos((current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=current_y+self.r_min*math.sin((current_th)-(self.PI/2))

		c2_x=goal_x+self.r_min*math.cos(math.radians(goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=goal_y+self.r_min*math.sin(math.radians(goal_th)-(self.PI/2))

		theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

		tan_pt1_x=c1_x-self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y+self.r_min*math.sin(theta)

		tan_pt2_x=c2_x-self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.r_min*math.sin(theta)

		S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

		Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle
		
		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RR"]
		return ans

	def LSR(self,current_x,current_y,current_th,goal_x,goal_y,goal_th):
		c1_x=current_x-self.r_min*math.cos((current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=current_y-self.r_min*math.sin((current_th)-(self.PI/2))

		c2_x=goal_x+self.r_min*math.cos(math.radians(goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=goal_y+self.r_min*math.sin(math.radians(goal_th)-(self.PI/2))

		if c2_x-c1_x<0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*self.r_min))
		elif c2_x-c1_x>0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*self.r_min))

		tan_pt1_x=c1_x+self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y-self.r_min*math.sin(theta)

		tan_pt2_x=c2_x-self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y+self.r_min*math.sin(theta)


		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

		if(-self.r_min**2 + (S1**2)/4.0 )<0:
			# print"Holaaaaaa"
			return None

		S = 2*((-self.r_min**2 + (S1**2)/4.0 )**0.5)

		Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LR"]
		return ans

	def RSL(self,current_x,current_y,current_th,goal_x,goal_y,goal_th):
		c1_x=current_x+self.r_min*math.cos((current_th)-(self.PI/2))	#center of first circle of min_turning radius
		c1_y=current_y+self.r_min*math.sin((current_th)-(self.PI/2))

		c2_x=goal_x-self.r_min*math.cos(math.radians(goal_th)-(self.PI/2))	#center of second circle of min_turning radius
		c2_y=goal_y-self.r_min*math.sin(math.radians(goal_th)-(self.PI/2))

		if c2_x-c1_x<0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*self.r_min))
		elif c2_x-c1_x>0:
			theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*self.r_min))

		tan_pt1_x=c1_x-self.r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
		tan_pt1_y=c1_y+self.r_min*math.sin(theta)

		tan_pt2_x=c2_x+self.r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
		tan_pt2_y=c2_y-self.r_min*math.sin(theta)

		S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5	#Distance from (c1_x,c1_y) to (c2_x,c2_y)

		if(-self.r_min**2 + (S1**2)/4.0 )<0:
			# print"Holaaaaaa"
			return None

		S = 2*((-self.r_min**2 + (S1**2)/4.0 )**0.5)
		
		Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y)	#Initial Turning angle

		Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)	#Final Turning angle

		if Ti<0:
			Ti+=2*np.pi
		if Tf<0:
			Tf+=2*np.pi

		total_dist = Ti*self.r_min + Tf*self.r_min + S 	#Total Distance from current to Goal

		#Ti= Ti*180/np.pi

		ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RL"]
		#	  0. 1.   2        3.  4.       5.        6.         7.       8.    9.   10.  11   12
		return ans

	def findNearest(self,sample,tree):
		min_d = sys.maxint
		ind=0
		min_node=tree[0]
		for indd,node in enumerate(tree):
			dis = ((node.x-sample.x)**2 + (node.y-sample.y)**2)**0.5
			if min_d>dis:
				min_d=dis
				min_node=node
				ind=indd
		return min_node,ind

	def run(self):
		self.goal_x = input("x goal:")
		self.goal_y = input("y goal:")
		self.goal_th = input("theta goal: (Input in Degrees )")		

		current_x=0
		current_y=0
		current_th=0

		pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		#rospy.init_node('p3dx_mover')

		step=5

		twist = Twist()
		
		self.r_min = 1
		
		#samp=self.LSL()
			
		tree = [Node(current_x,current_y)]

		initial=[]

		j=0
		while True:
			j+=1
			sample = Node(random.uniform(0,11),random.uniform(0,11))
			nearestNode,ind = self.findNearest(sample,tree)
			newNode = copy.deepcopy(nearestNode)
			
			theta = (np.arctan2(sample.y-nearestNode.y, sample.x-nearestNode.x))
			newNode.x+=step*math.cos(theta)
			newNode.y+=step*math.sin(theta)

			theta_start = theta
			theta_end = (np.arctan2(newNode.y-self.goal_y, newNode.x-self.goal_x))


			paths=[self.LSL,self.RSR,self.LSR,self.RSL] 
			minS=sys.maxint
			ans=[]
			count=0
			curve=""
			ans_count=1
			for i in paths:
				path_ans=i(nearestNode.x,nearestNode.y,theta_start,newNode.x,newNode.y,theta_end)
				count+=1
				if path_ans:
					if minS>path_ans[3]:
						ans=path_ans
						minS=path_ans[3]
						ans_count=count
						curve=path_ans[12]
			
			newNode.p=ind
			nearestNode.tan_pt1_x=ans[4]
			nearestNode.tan_pt1_y=ans[5]
			nearestNode.tan_pt2_x=ans[6]
			nearestNode.tan_pt2_y=ans[7]
			nearestNode.S=ans[3]
			nearestNode.curve=curve
			nearestNode.Ti=ans[0]

			if j==1:
				initial.append([ans[4],ans[5],ans[6],ans[7],ans[3],curve])

			# if ((newNode.x-3)**2 + (newNode.y-3)**2)**0.5 < 1:
			# 	continue
			# if ((newNode.x-6)**2 + (newNode.y-5)**2)**0.5 < 1:
			# 	continue			
			# if newNode.x>10 or newNode.y>10 or newNode.x<0 or newNode.y<0:
			# 	continue
			tree.append(newNode)
			if ((newNode.x-self.goal_x)**2 + (newNode.y-self.goal_y)**2)**0.5 < step:
				break

		paths=[self.LSL,self.RSR,self.LSR,self.RSL]
		minS=sys.maxint
		ans=[]
		count=0
		ans_count=1
		for i in paths:
			path_ans=i(newNode.x,newNode.y,0,self.goal_x,self.goal_y,self.goal_th)
			count+=1
			if path_ans:
				if minS>path_ans[3]:
					ans=path_ans
					minS=path_ans[3]
					ans_count=count

		newNode.tan_pt1_x=ans[4]
		newNode.tan_pt1_y=ans[5]
		newNode.tan_pt2_x=ans[6]
		newNode.tan_pt2_y=ans[7]
		newNode.S=ans[3]
		newNode.curve=curve
		newNode.Ti=ans[0]


		# path=[[newNode.x,newNode.y]]
		# while True:
		# 	if not newNode.p:
		# 		break
		# 	path.append([tree[newNode.p].x,tree[newNode.p].y])
		# 	newNode=tree[newNode.p]

		path_w_angles=[[newNode.tan_pt1_x,newNode.tan_pt1_y,newNode.tan_pt2_x,newNode.tan_pt2_y,newNode.S,newNode.curve,newNode.Ti]]
		while True:
			if not newNode.p:
				break
			path_w_angles.append([tree[newNode.p].tan_pt1_x,tree[newNode.p].tan_pt1_y,tree[newNode.p].tan_pt2_x,tree[newNode.p].tan_pt2_y,tree[newNode.p].S,tree[newNode.p].curve,tree[newNode.p].Ti])
			newNode=tree[newNode.p]

		path_w_angles.append(initial[0])


		# x=[self.goal_x]
		# y=[self.goal_y]
		# for i in path:
		# 	x.append(i[0])
		# 	y.append(i[1])
		# x.append(current_x)
		# y.append(current_y)
		# print x,y


		for ans in path_w_angles:

			tan_pt1_x=ans[0]
			tan_pt1_y=ans[1]
			tan_pt2_x=ans[2]
			tan_pt2_y=ans[3]
			S = ans[4]
			LoR1=ans[5][0]
			LoR2=ans[5][1]
		
			dele=input("checkpoint");
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
			if LoR1=='L':
				twist.angular.z = abs(angular_speed)
			elif LoR1=='R':
				twist.angular.z = -abs(angular_speed)

			
			errorf= math.sqrt( math.pow((tan_pt1_x-self.current_x),2) + math.pow((tan_pt1_y-self.current_y),2) )
			curr_x = self.current_x
			curr_y = self.current_y
			curr_yaw=self.current_th
			
			t1=rospy.get_time()
			while (errorf>0.1):
				pub.publish(twist)
				curr_x = self.current_x
				curr_y = self.current_y
				print curr_x,",",curr_y, "error", errorf
				errorf= math.sqrt( math.pow((tan_pt1_x-curr_x),2) + math.pow((tan_pt1_y-curr_y),2) )
				self.rate.sleep()

			twist.linear.x =0
			twist.angular.z = 0
			pub.publish(twist)
			
			print "configuration after first curve: ", self.current_x,self.current_y,self.current_th		
			dele=input("checkpoint");

			twist.linear.x = linear_speed

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
			

			dele=input("checkpoint");
			twist.linear.x = abs(linear_speed)
			if LoR2=='L':
				twist.angular.z = abs(angular_speed)
			elif LoR2=='R':
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