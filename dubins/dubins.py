import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import sys

current_x=0
current_y=0
current_th=0		#current angle in Degrees

PI=3.14
r_min=2

goal_x=4
goal_y=4
goal_th=0			#goal angle in Degrees

# fig, ax = plt.subplots()




def LSL():
	plt.ylim(-5,15)
	plt.xlim(-5,15)
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

	Ti= Ti*180/np.pi
	if Ti<0:
		Ti+=360
	Tf= Tf*180/np.pi
	if Tf<0:
		Tf+=360

	# print Ti,Tf, total_dist,S
	ans=[Ti,Tf, total_dist,S]

	start_angle1 = np.arctan2(current_y - c1_y, current_x - c1_x) * 180 / np.pi
	end_angle1 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) * 180 / np.pi

	arc1 = patches.Arc((c1_x, c1_y), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1)
	plt.gca().add_patch(arc1)

	start_angle2 = np.arctan2(tan_pt2_y - c2_y, tan_pt2_x - c2_x) * 180 / np.pi
	end_angle2 = np.arctan2(goal_y - c2_y, goal_x - c2_x) * 180 / np.pi

	arc2 = patches.Arc((c2_x, c2_y), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2)
	plt.gca().add_patch(arc2)


	plt.scatter(current_x,current_y)
	plt.scatter(goal_x,goal_y)
	plt.scatter(c1_x,c1_y)
	plt.plot([tan_pt1_x,tan_pt2_x],[tan_pt1_y,tan_pt2_y],'ro-')
	plt.scatter(c2_x,c2_y)
	plt.scatter(tan_pt1_x,tan_pt1_y)
	plt.scatter(tan_pt2_x,tan_pt2_y)

	# x = np.linspace(-10, 20, 100)
	# y = np.linspace(-10, 20, 100)
	# X, Y = np.meshgrid(x,y)
	# F = (X-c1_x)**2 + (Y-c1_y)**2 - r_min**2
	# F1 = (X-c2_x)**2 + (Y-c2_y)**2 - r_min**2
	# # plt.contour(X,Y,F,[0])
	# plt.contour(X,Y,F1,[0])
	plt.gca().set_aspect('equal')
	plt.show()
	# ax.relim()
	return ans

def RSR():
	plt.ylim(-5,15)
	plt.xlim(-5,15)
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
	if Ti<0:
		Ti+=360
	Tf= Tf*180/np.pi
	if Tf<0:
		Tf+=360

	# print Ti,Tf,total_dist,S

	start_angle1 = np.arctan2(current_y - c1_y, current_x - c1_x) * 180 / np.pi
	end_angle1 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) * 180 / np.pi

	arc1 = patches.Arc((c1_x, c1_y), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1)
	plt.gca().add_patch(arc1)

	start_angle2 = np.arctan2(tan_pt2_y - c2_y, tan_pt2_x - c2_x) * 180 / np.pi
	end_angle2 = np.arctan2(goal_y - c2_y, goal_x - c2_x) * 180 / np.pi

	arc2 = patches.Arc((c2_x, c2_y), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2)
	plt.gca().add_patch(arc2)

	print start_angle1,end_angle1,start_angle2,end_angle2

	plt.scatter(current_x,current_y)
	plt.scatter(goal_x,goal_y)
	plt.scatter(c1_x,c1_y)
	plt.scatter(c2_x,c2_y)
	plt.scatter(tan_pt1_x,tan_pt1_y)
	plt.scatter(tan_pt2_x,tan_pt2_y)
	plt.plot([tan_pt1_x,tan_pt2_x],[tan_pt1_y,tan_pt2_y],'ro-')

	# x = np.linspace(-10, 20, 100)
	# y = np.linspace(-10, 20, 100)
	# X, Y = np.meshgrid(x,y)
	# F = (X-c1_x)**2 + (Y-c1_y)**2 - r_min**2
	# F1 = (X-c2_x)**2 + (Y-c2_y)**2 - r_min**2
	# plt.contour(X,Y,F,[0])
	# plt.contour(X,Y,F1,[0])
	plt.gca().set_aspect('equal')
	plt.show()

	ans=[Ti,Tf, total_dist,S]
	return ans

def LSR():
	plt.ylim(-5,15)
	plt.xlim(-5,15)
	c1_x=current_x-r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y-r_min*math.sin(math.radians(current_th)-(PI/2))

	c2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(PI/2))	#center of second circle of min_turning radius
	c2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(PI/2))

	# if c2_x-c1_x<0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*r_min))
	# elif c2_x-c1_x>0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*r_min))

	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)

	theta =   (np.arctan2((c2_x-c1_x), c2_y-c1_y)) - np.arctan2(r_min,S/2.0)


	tan_pt1_x=c1_x+r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y-r_min*math.sin(theta)

	tan_pt2_x=c2_x-r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y+r_min*math.sin(theta)

	start_angle1 = np.arctan2(current_y - c1_y, current_x - c1_x) * 180 / np.pi
	end_angle1 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) * 180 / np.pi

	arc1 = patches.Arc((c1_x, c1_y), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1)
	plt.gca().add_patch(arc1)

	start_angle2 = np.arctan2(tan_pt2_y - c2_y, tan_pt2_x - c2_x) * 180 / np.pi
	end_angle2 = np.arctan2(goal_y - c2_y, goal_x - c2_x) * 180 / np.pi

	arc2 = patches.Arc((c2_x, c2_y), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2)
	plt.gca().add_patch(arc2)

	plt.scatter(current_x,current_y)
	plt.scatter(goal_x,goal_y)
	plt.scatter(c1_x,c1_y)
	plt.scatter(c2_x,c2_y)
	plt.scatter(tan_pt1_x,tan_pt1_y)
	plt.scatter(tan_pt2_x,tan_pt2_y)
	plt.plot([tan_pt1_x,tan_pt2_x],[tan_pt1_y,tan_pt2_y],'ro-')

	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)
	print (tan_pt1_x,tan_pt1_y)
	print (tan_pt2_x,tan_pt2_y)
	print (c1_x,c1_y,c2_x,c2_y)

	Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y)	#Final Turning angle

	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	Ti= Ti*180/np.pi
	if Ti<0:
		Ti+=360
	Tf= Tf*180/np.pi
	if Tf<0:
		Tf+=360

	print Ti,Tf,total_dist,S,"yooo"

	# x = np.linspace(-10, 20, 100)
	# y = np.linspace(-10, 20, 100)
	# X, Y = np.meshgrid(x,y)
	# F = (X-c1_x)**2 + (Y-c1_y)**2 - r_min**2
	# F1 = (X-c2_x)**2 + (Y-c2_y)**2 - r_min**2
	# plt.contour(X,Y,F,[0])
	# plt.contour(X,Y,F1,[0])
	plt.gca().set_aspect('equal')
	plt.show()

	ans=[Ti,Tf, total_dist,S]
	return ans

def RSL():
	plt.ylim(-5,15)
	plt.xlim(-5,15)
	c1_x=current_x+r_min*math.cos(math.radians(current_th)-(PI/2))	#center of first circle of min_turning radius
	c1_y=current_y+r_min*math.sin(math.radians(current_th)-(PI/2))

	c2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(PI/2))	#center of second circle of min_turning radius
	c2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(PI/2))

	# if c2_x-c1_x<0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y+2*r_min))
	# elif c2_x-c1_x>0:
	# 	theta = (np.arctan2((c2_x-c1_x), c2_y-c1_y-2*r_min))

	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5		#Distance from (c1_x,c1_y) to (c2_x,c2_y)

	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)

	theta =   + (np.arctan2((c2_x-c1_x), c2_y-c1_y)) + np.arctan2(r_min,S/2.0)


	tan_pt1_x=c1_x-r_min*math.cos(theta)	#tangent point in first circle of min_turning radius
	tan_pt1_y=c1_y+r_min*math.sin(theta)

	tan_pt2_x=c2_x+r_min*math.cos(theta)	#tangent point in second circle of min_turning radius
	tan_pt2_y=c2_y-r_min*math.sin(theta)

	plt.scatter(current_x,current_y)
	plt.scatter(goal_x,goal_y)
	plt.scatter(c1_x,c1_y)
	plt.scatter(c2_x,c2_y)
	plt.scatter(tan_pt1_x,tan_pt1_y)
	plt.scatter(tan_pt2_x,tan_pt2_y)
	plt.plot([tan_pt1_x,tan_pt2_x],[tan_pt1_y,tan_pt2_y],'ro-')

	S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5	#Distance from (c1_x,c1_y) to (c2_x,c2_y)
	print (tan_pt1_x,tan_pt1_y)
	print (tan_pt2_x,tan_pt2_y)
	print (c1_x,c1_y,c2_x,c2_y)

	S = 2*((-r_min**2 + (S1**2)/4.0 )**0.5)
	
	Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y)	#Initial Turning angle

	Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y)	#Final Turning angle

	if Ti<0:
		Ti+=2*np.pi
	if Tf<0:
		Tf+=2*np.pi

	total_dist = Ti*r_min + Tf*r_min + S 	#Total Distance from current to Goal

	Ti= Ti*180/np.pi
	if Ti<0:
		Ti+=360
	Tf= Tf*180/np.pi
	if Tf<0:
		Tf+=360

	# print Ti,Tf,total_dist,S

	start_angle1 = np.arctan2(current_y - c1_y, current_x - c1_x) * 180 / np.pi
	end_angle1 = np.arctan2(tan_pt1_y - c1_y, tan_pt1_x - c1_x) * 180 / np.pi

	arc1 = patches.Arc((c1_x, c1_y), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1)
	plt.gca().add_patch(arc1)

	start_angle2 = np.arctan2(tan_pt2_y - c2_y, tan_pt2_x - c2_x) * 180 / np.pi
	end_angle2 = np.arctan2(goal_y - c2_y, goal_x - c2_x) * 180 / np.pi

	arc2 = patches.Arc((c2_x, c2_y), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2)
	plt.gca().add_patch(arc2)

	# x = np.linspace(-10, 20, 100)
	# y = np.linspace(-10, 20, 100)
	# X, Y = np.meshgrid(x,y)
	# F = (X-c1_x)**2 + (Y-c1_y)**2 - r_min**2
	# F1 = (X-c2_x)**2 + (Y-c2_y)**2 - r_min**2
	# plt.contour(X,Y,F,[0])
	# plt.contour(X,Y,F1,[0])
	plt.gca().set_aspect('equal')
	plt.show()

	ans=[Ti,Tf, total_dist,S]
	return ans


paths=[LSL,RSR,LSR,RSL] 
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

# print "Optimal Path: " ,paths[ans_count-1], "Total distance to cover: ",distance
# LSL()
# RSR()
# LSR()
# RSL()

