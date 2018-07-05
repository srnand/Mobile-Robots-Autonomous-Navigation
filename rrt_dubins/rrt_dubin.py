import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import Minkowski
import random
from matplotlib.patches import Arc
import sys
# import dubins
import copy

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

plt.axis([-20, 20, -20, 20])
# rectangle = plt.Rectangle((0.2, 0.2), 0.2, 0.2)
# plt.gca().add_patch(rectangle)

obs = plt.Circle((3, 3), radius=1, fc='r')
plt.gca().add_patch(obs)

obs = plt.Circle((6, 5), radius=1, fc='r')
plt.gca().add_patch(obs)

# obs = plt.Circle((7, 3), radius=1, fc='r')
# plt.gca().add_patch(obs)

# obs = plt.Circle((6, 5), radius=1, fc='r')
# plt.gca().add_patch(obs)

rectangle = plt.Rectangle((0, 0), 1, 1,fill=False)
plt.gca().add_patch(rectangle)


current_x=0
current_y=0
current_th=0		#current angle in Degrees

PI=3.14
r_min=0.2
step=2

goal_x=8
goal_y=8
goal_th=0			#goal angle in Degrees

goal_circle = plt.Circle((8, 8), radius=1, fc='y')
plt.gca().add_patch(goal_circle)

plt.axis('scaled')
# goal_circle = 
#plt.show()

tree = [Node(current_x,current_y)]

def check_obstacle(x,y,obs_x,obs_y):
	obstacle_radius = Minkowski.minkowski_sum_circle(0.2,0.6)
	if ((x-obs_x)**2 + (y-obs_y)**2)**0.5 < obstacle_radius+0.5:
		return True
	return False
def LSL(current_x,current_y,current_th,goal_x,goal_y,goal_th):
	c1_x=current_x-r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y-r_min*math.sin(math.radians(current_th)-(PI/2))		

	c2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(PI/2))		#center of second circle of min_turning radius
	c2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(PI/2))
	#print c1_x,c1_y,c2_x,c2_y, c2_y-c1_y, c2_x-c1_x
	theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

	tan_pt1_x=c1_x+r_min*math.cos(theta)		#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y-r_min*math.sin(theta)

	tan_pt2_x=c2_x+r_min*math.cos(theta)		#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y-r_min*math.sin(theta)

	S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5
	# print S,theta
	# print (tan_pt1_x,tan_pt1_y)
	# print (tan_pt2_x,tan_pt2_y)
	# print (c1_x,c1_y,c2_x,c2_y)

	Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)	#Final Turning angle

	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	# Ti= Ti*180/np.pi

	# print Ti,Tf, total_dist,S
	ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LL"]

	return ans

def RSR(current_x,current_y,current_th,goal_x,goal_y,goal_th):
	c1_x=current_x+r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y+r_min*math.sin(math.radians(current_th)-(PI/2))

	c2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(PI/2))	#center of second circle of min_turning radius
	c2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(PI/2))

	theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

	tan_pt1_x=c1_x-r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y+r_min*math.sin(theta)

	tan_pt2_x=c2_x-r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y+r_min*math.sin(theta)

	S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

	# print (tan_pt1_x,tan_pt1_y)
	# print (tan_pt2_x,tan_pt2_y)
	# print (c2_x,c2_y)
	Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle
	
	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	Ti= Ti*180/np.pi


	ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RR"]
	return ans

def LSR(current_x,current_y,current_th,goal_x,goal_y,goal_th):
	c1_x=current_x-r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y-r_min*math.sin(math.radians(current_th)-(PI/2))

	c2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(PI/2))	#center of second circle of min_turning radius
	c2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(PI/2))

	# if c2_x-c1_x<0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*r_min))
	# elif c2_x-c1_x>0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*r_min))
	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

	if(-r_min**2 + (S1**2)/4.0 )<0:
		# print"Holaaaaaa"
		return None

	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)
	theta =   (np.arctan2((c2_x-c1_x), c2_y-c1_y)) - np.arctan2(r_min,S/2.0)


	tan_pt1_x=c1_x+r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y-r_min*math.sin(theta)

	tan_pt2_x=c2_x-r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y+r_min*math.sin(theta)


	Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle

	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	Ti= Ti*180/np.pi
	print Ti, current_x,current_y,current_th,goal_x,goal_y,goal_th

	ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LR"]
	return ans

def RSL(current_x,current_y,current_th,goal_x,goal_y,goal_th):
	c1_x=current_x+r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y+r_min*math.sin(math.radians(current_th)-(PI/2))

	c2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(PI/2))	#center of second circle of min_turning radius
	c2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(PI/2))

	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5	#Distance from (c1_x,c1_y) to (c2_x,c2_y)
	# print (tan_pt1_x,tan_pt1_y)
	# print (tan_pt2_x,tan_pt2_y)
	if(-r_min**2 + (S1**2)/4.0 )<0:
		# print"Holaaaaaa"
		return None
	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)

	# if c2_x-c1_x<0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*r_min))
	# elif c2_x-c1_x>0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*r_min))

	theta =   + (np.arctan2((c2_x-c1_x), c2_y-c1_y)) + np.arctan2(r_min,S/2.0)

	print "THETA ", theta, "c1_x =",c1_x, c1_y,c2_x,c2_y,"S=",S

	tan_pt1_x=c1_x-r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y+r_min*math.sin(theta)

	tan_pt2_x=c2_x+r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y-r_min*math.sin(theta)

	
	Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)	#Final Turning angle

	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	Ti= Ti*180/np.pi
	print Ti, current_x,current_y,current_th,goal_x,goal_y,goal_th

	ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RL"]
	#    0   1.  2.        3. 4.        5.        6.        7.        8.   9.    10.  11
	return ans


def findNearest(sample):
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

def RRT():
	j=0
	initial=[]
	theta_end=None
	while True:
		j+=1
		# if j>5:
		# 	break
		sample = Node(random.uniform(0,11),random.uniform(0,11))
		nearestNode,ind = findNearest(sample)
		newNode = copy.deepcopy(nearestNode)
		
		theta = (np.arctan2(sample.y-nearestNode.y, sample.x-nearestNode.x))
		newNode.x+=step*math.cos(theta)
		newNode.y+=step*math.sin(theta)

		# ansLSL=LSL(nearestNode.x,nearestNode.y,0,newNode.x,newNode.y,0)
		# if 

		theta_start = theta
		theta_end = (np.arctan2(newNode.y-goal_y, newNode.x-goal_x))


		paths=[LSL,RSR,LSR,RSL] 
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

		plt.plot([ans[4],ans[6]],[ans[5],ans[7]])

		start_angle1 = np.arctan2(nearestNode.y - ans[9], nearestNode.x - ans[8]) * 180 / np.pi
		end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi
		
		if curve[0]=="L":
			arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1)
		else:
			arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1)
		plt.gca().add_patch(arc1)

		start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
		end_angle2 = np.arctan2(newNode.y - ans[11], newNode.x - ans[10]) * 180 / np.pi
		
		if curve[1]=="L":
			arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2)
		else:
			arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2)

		plt.gca().add_patch(arc2)

		plt.gca().set_aspect('equal')

		# ans = LSL(nearestNode.x,nearestNode.y,0,newNode.x,newNode.y,0)
		
		newNode.p=ind
		nearestNode.tan_pt1_x=ans[4]
		nearestNode.tan_pt1_y=ans[5]
		nearestNode.tan_pt2_x=ans[6]
		nearestNode.tan_pt2_y=ans[7]
		nearestNode.S=ans[3]
		nearestNode.curve=curve
		nearestNode.Ti=ans[0]
		if j==1:
			initial.append([ans[4],ans[5],ans[6],ans[7],ans[3],curve,ans[0]])

		# if ((newNode.x-3)**2 + (newNode.y-3)**2)**0.5 < 1:
		# 	continue
		# if ((newNode.x-6)**2 + (newNode.y-5)**2)**0.5 < 1:
		# 	continue
		if check_obstacle(newNode.x,newNode.y,3,3):
			continue
		if check_obstacle(newNode.x,newNode.y,6,5):
			continue
		# if check_obstacle(newNode.x,newNode.y,3,6):
		# 	continue		
		# if check_obstacle(newNode.x,newNode.y,6,5):
		# 	continue		
		if newNode.x>10 or newNode.y>10 or newNode.x<0 or newNode.y<0:
			continue
		tree.append(newNode)
		if ((newNode.x-goal_x)**2 + (newNode.y-goal_y)**2)**0.5 < step:
			break

	# print tree[0].Ti

	paths=[LSL,RSR,LSR,RSL] 
	minS=sys.maxint
	ans=[]
	count=0
	ans_count=1
	for i in paths:
		path_ans=i(newNode.x,newNode.y,theta_end,goal_x,goal_y,goal_th)
		count+=1
		if path_ans:
			if minS>path_ans[3]:
				ans=path_ans
				minS=path_ans[3]
				ans_count=count
				curve=path_ans[12]

	newNode.tan_pt1_x=ans[4]
	newNode.tan_pt1_y=ans[5]
	newNode.tan_pt2_x=ans[6]
	newNode.tan_pt2_y=ans[7]
	newNode.S=ans[3]
	newNode.curve=curve
	newNode.Ti=ans[0]

	plt.plot([ans[4],ans[6]],[ans[5],ans[7]])

	start_angle1 = np.arctan2(newNode.y - ans[9], newNode.x - ans[8]) * 180 / np.pi
	end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi
	
	if curve[0]=="L":
		arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1)
	else:
		arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1)
	plt.gca().add_patch(arc1)

	start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
	end_angle2 = np.arctan2(goal_y - ans[11], goal_x - ans[10]) * 180 / np.pi
	
	if curve[1]=="L":
		arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2)
	else:
		arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2)

	plt.gca().add_patch(arc2)

	plt.gca().set_aspect('equal')

	newNode1 = copy.deepcopy(newNode)

	path=[[newNode.x,newNode.y]]
	while True:
		if not newNode.p:
			break
		#plt.plot([newNode.x,newNode.y],[tree[newNode.p].x,tree[newNode.p].y],'ro-')
		path.append([tree[newNode.p].x,tree[newNode.p].y])
		newNode=tree[newNode.p]

	path_w_angles=[[newNode1.tan_pt1_x,newNode1.tan_pt1_y,newNode1.tan_pt2_x,newNode1.tan_pt2_y,newNode1.S,newNode1.curve,newNode1.Ti]]
	while True:
		if not newNode1.p:
			break
		path_w_angles.append([tree[newNode1.p].tan_pt1_x,tree[newNode1.p].tan_pt1_y,tree[newNode1.p].tan_pt2_x,tree[newNode1.p].tan_pt2_y,tree[newNode1.p].S,tree[newNode1.p].curve,tree[newNode1.p].Ti])
		newNode1=tree[newNode1.p]
	path_w_angles.append(initial[0])

	path_w_angles=path_w_angles[::-1]
	#path_w_angles=[[0.11382975330055782, 0.03707606850741252, 1.550462272816677, 1.0601573107627982, 1.76369164662819, 'LR', 34.829836137384575], [1.8537386028505958, 1.3667442952423527, 1.3133630638948597, 2.820859157341914, 1.5512755252631716, 'LR', 108.68731498468217], [1.7125078645071232, 3.2591611478524074, 1.802594876855204, 3.8311177818676017, 0.579007824634343, 'LR', 83.07015853891144]]

	x=[goal_x]
	y=[goal_y]
	for i in path:
		x.append(i[0])
		y.append(i[1])
	x.append(current_x)
	y.append(current_y)
	# print x,y
	plt.plot(x, y, '-ro',alpha=0.2)
	print path_w_angles, len(path_w_angles), len(path)
	plt.show()
RRT()
# RSL(0.151237017246, 1.99427364336, 1.20720741966, 0.862498625581, 3.86352668255, -2.74749039623)