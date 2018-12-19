import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import random
import sys
import copy

class Node():
	def __init__(self,x,y):
		self.x=x
		self.y=y
		self.p=None
		self.cost=0

plt.axis([-0.1, 1.1, -0.1, 1.1])
# rectangle = plt.Rectangle((0.2, 0.2), 0.2, 0.2)
# plt.gca().add_patch(rectangle)

obs = plt.Circle((0.3, 0.3), radius=0.1, fc='r')
plt.gca().add_patch(obs)

obs = plt.Circle((0.6, 0.5), radius=0.2, fc='r')
plt.gca().add_patch(obs)

obs = plt.Circle((0.3, 0.5), radius=0.1, fc='r')
plt.gca().add_patch(obs)

obstacles = [(0.3,0.3,0.1),(0.6,0.5,0.2),(0.3,0.5,0.1)]

rectangle = plt.Rectangle((0, 0), 1, 1,fill=False)
plt.gca().add_patch(rectangle)


current_x=0
current_y=0
current_th=0		#current angle in Degrees

PI=3.14
r_min=0.02
step=0.3

goal_x=0.8
goal_y=0.8
goal_th=0			#goal angle in Degrees

goal_circle = plt.Circle((0.8, 0.8), radius=0.1, fc='b')
plt.gca().add_patch(goal_circle)

plt.axis('scaled')
# goal_circle = 
#plt.show()

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
	# print Ti, current_x,current_y,current_th,goal_x,goal_y,goal_th

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

	# print "THETA ", theta, "c1_x =",c1_x, c1_y,c2_x,c2_y,"S=",S

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
	# print Ti, current_x,current_y,current_th,goal_x,goal_y,goal_th

	ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RL"]
	#    0   1.  2.        3. 4.        5.        6.        7.        8.   9.    10.  11
	return ans

tree = [Node(current_x,current_y)]

def check_for_obstacles(x,y):
	for ox,oy,orad in obstacles:
		if ((x-ox)**2 + (y-oy)**2)**0.5 < orad*2:
			return False
	return True

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

def find_nearby_nodes(newNode):
	radius = step*2
	dist_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in tree]
	nearinds = [dist_list.index(i) for i in dist_list if i <= (radius ** 2)]
	return nearinds

def check_collision_extend(node, theta, d):
	
	temp_node = copy.deepcopy(node)
	
	for i in range(int(d / step)):
		temp_node.x += step * math.cos(theta)
		temp_node.y += step * math.sin(theta)
		if not check_for_obstacles(temp_node.x,temp_node.y):
			return False
		if temp_node.x>1 or temp_node.y>1 or temp_node.x<0 or temp_node.y<0:
			return False
	return True

def rewire(node, nearinds):
	for i in nearinds:
		nearNode = tree[i]
		dx = node.x - nearNode.x
		dy = node.y - nearNode.y
		d = math.sqrt(dx ** 2 + dy ** 2)

		if nearNode.cost>node.cost+d:
			theta = np.arctan2(dy, dx)
			if check_collision_extend(tree[i], theta, d):
				nearNode.p=tree.index(node)
				nearNode.cost=node.cost+d

def choose_parent(newNode, nearinds):
	if not nearinds:
		return newNode
	dist_list = []
	for i in nearinds:
		dx = newNode.x - tree[i].x
		dy = newNode.y - tree[i].y
		d = math.sqrt(dx ** 2 + dy ** 2)
		theta = np.arctan2(dy, dx)
		if check_collision_extend(tree[i], theta, d):
			dist_list.append(tree[i].cost + d)

	if not dist_list:
		return newNode

	mincost = min(dist_list)
	minind = nearinds[dist_list.index(mincost)]

	newNode.cost = mincost
	newNode.p = minind

	return newNode

def RRT():
	while True:
		sample = Node(random.uniform(0,1),random.uniform(0,1))

		nearestNode,ind = findNearest(sample)
		newNode = copy.deepcopy(nearestNode)
		theta = (np.arctan2(sample.y-nearestNode.y, sample.x-nearestNode.x))
		newNode.x+=step*math.cos(theta)
		newNode.y+=step*math.sin(theta)
		newNode.p=ind
		newNode.cost+=step
		# plt.plot(newNode.x,newNode.y,'r+')
		# plt.plot(nearestNode.x,nearestNode.y,'bo')
		# plt.pause(2)
		# plt.plot(nearestNode.x,nearestNode.y,'wo')
		# plt.pause(2)

		# if ((newNode.x-0.3)**2 + (newNode.y-0.3)**2)**0.5 < 0.2:
		# 	# plt.plot(newNode.x,newNode.y,'w+')
		# 	# plt.pause(2)
		# 	continue
		# if ((newNode.x-0.6)**2 + (newNode.y-0.5)**2)**0.5 < 0.2:
		# 	# plt.plot(newNode.x,newNode.y,'w+')
		# 	# plt.pause(2)
		# 	continue
		# if ((newNode.x-0.3)**2 + (newNode.y-0.5)**2)**0.5 < 0.2:
		# 	# plt.plot(newNode.x,newNode.y,'w+')
		# 	# plt.pause(2)
		# 	continue
		if not check_for_obstacles(newNode.x,newNode.y):
			continue
		if newNode.x>1 or newNode.y>1 or newNode.x<0 or newNode.y<0:
			# plt.plot(newNode.x,newNode.y,'w+')
			# plt.pause(2)
			continue
		nearbyNodes = find_nearby_nodes(newNode)
		newNode = choose_parent(newNode, nearbyNodes)
		# plt.plot(newNode.x,newNode.y,'bo')
		# plt.pause(2)
		# plt.plot(newNode.x,newNode.y,'wo')
		# plt.pause(2)
		tree.append(newNode)
		rewire(newNode,nearbyNodes)

		if ((newNode.x-goal_x)**2 + (newNode.y-goal_y)**2)**0.5 < 0.1:
			break
		# DrawGraph(sample)
	path=[[newNode.x,newNode.y]]
	while True:
		if not newNode.p:
			break
		#plt.plot([newNode.x,newNode.y],[tree[newNode.p].x,tree[newNode.p].y],'ro-')
		path.append([tree[newNode.p].x,tree[newNode.p].y])
		newNode=tree[newNode.p]
	x=[goal_x]
	y=[goal_y]
	for i in path:
		x.append(i[0])
		y.append(i[1])
		# plt.plot(i[0],i[1],'o-')
		# plt.pause(0.5)
	x.append(current_x)
	y.append(current_y)
	# print x,y
	nx=x[::-1][1:]
	ny=y[::-1][1:]

	sx=0
	sy=0
	st=0
	for a,b in zip(nx,ny):
		ex,ey=a,b
		et=(np.arctan2(ey-goal_y, ex-goal_x))

		# dubins
		paths=[LSL,RSR,LSR,RSL] 
		minS=float("inf")
		ans=[]
		count=0
		curve=""
		ans_count=1
		for i in paths:
			path_ans=i(sx,sy,st,ex,ey,et)
			count+=1
			if path_ans:
				if minS>path_ans[3]:
					ans=path_ans
					minS=path_ans[3]
					ans_count=count
					curve=path_ans[12]

		plt.plot([ans[4],ans[6]],[ans[5],ans[7]])

		start_angle1 = np.arctan2(sy - ans[9], sx - ans[8]) * 180 / np.pi
		end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi
		
		if curve[0]=="L":
			arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1)
		else:
			arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1)
		plt.gca().add_patch(arc1)

		start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
		end_angle2 = np.arctan2(ey - ans[11], ex - ans[10]) * 180 / np.pi
		
		if curve[1]=="L":
			arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2)
		else:
			arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2)

		plt.gca().add_patch(arc2)

		plt.gca().set_aspect('equal')

		#
		sx,sy,st=ex,ey,et
	# plt.plot(x, y, 'o-')
	# plt.pause(0.05)
	plt.show()
RRT()