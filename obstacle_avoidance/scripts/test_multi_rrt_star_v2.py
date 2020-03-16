#!/usr/bin/env python
import argparse
from threading import Thread
from time import sleep
import datetime
import random
import roslib
import tf
import sys
import rospy
import numpy as np
from numpy import nan
import copy
import mavros
import json
import os,tty,termios
import glob
from mavros.utils import *
from mavros_msgs.msg import AttitudeTarget, State, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import TransformStamped, Point, Pose, PoseStamped, TwistStamped, PolygonStamped, Polygon, Point32, Twist, Quaternion, Vector3, Twist, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32, Float64, Int8MultiArray, Header
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import GridCells, OccupancyGrid
from math import sqrt,cos,sin,atan2,asin
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import local_planning as locplan

import compute_coverage as com
import load_parameter as lp

from tf.transformations import euler_from_quaternion

import tf_conversions
import tf2_ros

NUM_OF_AGENT = 4
grid_size = 100
EPSILON = 5


def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def onSegment(p,q,r):
	if((q[0] <= max(p[0],r[0])) and (q[0] >= min(p[0],r[0])) and (q[1] <= max(p[1],r[1])) and (q[1] >= min(p[1],r[1]))):
		return True
	return False

def orientation(p,q,r):
	val = (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])
	if val == 0:
		return 0
	if val>0:
		return 1
	else:
		return 2 

def doIntersect(p1,q1,p2,q2):
	o1 = orientation(p1,q1,p2)
	o2 = orientation(p1,q1,q2)
	o3 = orientation(p2,q2,p1)
	o4 = orientation(p2,q2,q1)

	if ((o1 != o2) and (o3 != o4)):
		return True

	if ((o1 == 0) and onSegment(p1,p2,q1)):
		return True

	if ((o2 == 0) and onSegment(p1,q2,q1)):
		return True

	if ((o3 == 0) and onSegment(p2,p1,q2)):
		return True

	if ((o4 == 0) and onSegment(p2,q1,q2)):
		return True

	return False

def isInside(polygon,p):
    extreme = (999999,p[1])
    i = 0
    count = 0
    n = len(polygon)
    while(True):
        nxt = (i+1)%n  
        if(doIntersect(polygon[i],polygon[nxt],p,extreme)):
            if(orientation(polygon[i],p,polygon[nxt]) == 0):
                return onSegment(polygon[i],p,polygon[nxt])
            count=count+1
        i=nxt
        if(i==0):
            break

    return count&1

def freeObstacleWithClearance(nn,newnode,c):
    cell = 256
    current_map=np.zeros((cell,cell))
    current_map[140,140] = 100
    map_res=0.2
    map_size=256*map_res
    x0 = (nn[0]+map_size/2)/map_res
    y0 = (nn[1]+map_size/2)/map_res
    x1 = (newnode[0]+map_size/2)/map_res
    y1 = (newnode[1]+map_size/2)/map_res

    print (x0,y0),(x1,y1)
    theta = atan2(y1-y0,x1-x0)
    #A
    x2 = x0+c*sin(theta)
    y2 = y0-c*cos(theta)
    #B
    x3 = x1+c*sin(theta)
    y3 = y1-c*cos(theta)
    #D
    x4 = x0-c*sin(theta)
    y4 = y0+c*cos(theta)
    #C
    x5 = x1-c*sin(theta)
    y5 = y1+c*cos(theta)    
    max_x = int(np.ceil(max(x2,x3,x4,x5)/map_res)*map_res)
    max_y = int(np.ceil(max(y2,y3,y4,y5)/map_res)*map_res)
    min_x = int(np.floor(min(x2,x3,x4,x5)/map_res)*map_res)
    min_y = int(np.floor(min(y2,y3,y4,y5)/map_res)*map_res) 
    AB = (x3-x2,y3-y2)
    BC = (x5-x2,y5-y2)
    print "A",x2,y2,"B",x3,y3,"C",x5,y5,"D",x4,y4
    print min_x,max_x,min_y,max_y
    polygon = [(x2,y2),(x3,y3),(x5,y5),(x4,y4)]
    for i in range(min_x,max_x):
        for j in range(min_y,max_y):
            try:
                #M = (j,i)
                #AM = (j-x2,i-y2)
                #BM = (j-x3,i-y3)
                #if 0 <= np.dot(AB,AM) <= np.dot(AB,AB) and 0 <= np.dot(BC,BM) <= np.dot(BC,BC):
                if(current_map[i,j] == 100):
                    print "grid",i,j
                    if (isInside(polygon,(i,j))):
                        return False
            except:
                pass

    return True



import time
max_radius = 20    

i0 = time.time()
class DiscTemplate:
    def __init__(self, max_r):
        self.memos = []
        for k_r in range(1, max_r + 1):
            k_r_sq = k_r ** 2
            self.memos.append([])
            for x in range(-max_r, max_r + 1):
                x_sq = x ** 2
                for y in range(-max_r, max_r + 1):
                    y_sq = y ** 2
                    if x_sq + y_sq <= k_r_sq:
                        self.memos[k_r - 1].append((x,y))

        self.max_r = max_r

    def get_disc(self, r):
        return self.memos[r - 1]
'''
test = DiscTemplate(20)
i1 = time.time()

print("time to make class:", i1 - i0)

t0 = time.time()
disc = test.get_disc(19)
t1 = time.time()

print("time to produce disc:", t1 - t0)
print("Disc coordinates: \n", disc)
'''
def inflate(gridmap,distance):
    test = DiscTemplate(distance)
    g = test.get_disc(distance)
    #costmap = gridmap[:]
    retlist = set()
    for i in range(0,gridmap.shape[0]):
        for j in range(0,gridmap.shape[1]):
            #print "inflate",i,j
            if(gridmap[i][j]==100):
                #retlist.append((i,j))
                #costmap[i][j] = 100
                #pass
                
                gg = {(x+i,y+j) for (x,y) in g}
                #print "gg",gg
                retlist = retlist.union(gg)
                #gg = [(x+i,y+j) for (x,y) in g]
                '''
                for idx in gg:
                    try:
                        costmap[idx[0]][idx[1]]=100
                    except:
                        pass
                '''
    #print retlist
    return retlist
    #return costmap


def LineOfSight(walls,s1,s2):
    global map_size,free_occ,current_map,map_res
    #print "masuk los",walls
    x0=int(np.ceil(s1[0]+grid_size/2))
    y0=int(np.ceil(s1[1]+grid_size/2))
    x1=int(np.ceil(s2[0]+grid_size/2))
    y1=int(np.ceil(s2[1]+grid_size/2))
    #print (x1,y1)
    #print free_occ
    #if current_map[y1,x1] == -1:
    #    return False
    #print x0,y0,x1,y1
    dy=y1-y0
    dx=x1-x0
    f=0
    if(dy < 0):
        dy=-dy
        sy=-1
    else:
        sy=1
    if(dx < 0):
        dx=-dx
        sx=-1
    else:
        sx=1
    if(dx >= dy):
        while (x0 != x1):
            f=f+dy
            if(f >= dx):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                    return False
                y0 = y0+sy
                f = f-dx
            if (f != 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                return False
            if (dy == 0) and ((x0+((sx-1)/2),y0) in walls) and ((x0+((sx-1)/2),y0-1) in walls):
                return False
            x0=x0+sx
    else:
        while (y0 != y1):
            f=f+dx
            if(f >= dy):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                    return False
                x0 = x0+sx
                f = f-dy
            if (f != 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in walls):
                return False
            if (dx == 0) and ((x0,y0+((sy-1)/2)) in walls) and ((x0-1,y0+((sy-1)/2)) in walls):
                return False
            y0=y0+sy
	
    return True

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def reconstruct_path(came_from, start, goal_local):
    current = goal_local
    path = []
    while current != start:
        path.append(current)
        #print 'c',current
        current = came_from[current]
    path.append(start) # optional
    #path.reverse() # optional
    return path

from shapely.geometry import LineString
def is_line_intersection(line1, line2):
    line = LineString(line1)
    other = LineString(line2)
    return line.intersects(other)
    '''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    #print div
    if div < 0:
       return False
    else:
        return True
    '''

def is_line_intersect_with_lines(lines,line_in):
    if(len(lines) < 1):
        return False
    for line in lines:
        if is_line_intersection(line_in,line):
            return True
    return False

def main(args):
    rospy.init_node("test_marker")
    r=rospy.Rate(10)


    goal = PoseStamped()
    goal.pose.position.x = -49
    goal.pose.position.y = -49

    topic = 'uav_goal_marker'
    uav_goal_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
    uav_goal_marker = Marker()
    uav_goal_marker.header.frame_id = "map"
    uav_goal_marker.header.stamp=rospy.Time.now()
    uav_goal_marker.id = 0
    uav_goal_marker.type = uav_goal_marker.SPHERE
    uav_goal_marker.action = uav_goal_marker.ADD
    uav_goal_marker.scale.x = 1.0#par.ws_model['robot_radius']*2
    uav_goal_marker.scale.y = 1.0#par.ws_model['robot_radius']*2
    uav_goal_marker.scale.z = 1.0#0.005*par.RealScale
    uav_goal_marker.color.r = 0.0/255.0
    uav_goal_marker.color.g = 255.0/255.0
    uav_goal_marker.color.b = 0.0/255.0
    uav_goal_marker.color.a = 1.0
    uav_goal_marker.lifetime = rospy.Duration(0)
    uav_goal_marker.pose.orientation.x = 0.0
    uav_goal_marker.pose.orientation.y = 0.0
    uav_goal_marker.pose.orientation.z = 0.0
    uav_goal_marker.pose.orientation.w = 1.0
    uav_goal_marker.pose.position.x = goal.pose.position.x#init_pos[0]
    uav_goal_marker.pose.position.y = goal.pose.position.y#init_pos[1] 
    uav_goal_marker.pose.position.z = 0#goal.pose.position.z#init_pos[2] 
    uav_goal_marker_publisher.publish(uav_goal_marker)


    cur_pose = PoseStamped()
    cur_pose.pose.position.x = 49
    cur_pose.pose.position.y = 49

    topic = 'uav_start_marker'
    uav_start_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
    uav_start_marker = Marker()
    uav_start_marker.header.frame_id = "map"
    #uav_start_marker.header.stamp=rospy.Time.now()
    uav_start_marker.id = 0
    uav_start_marker.type = uav_start_marker.SPHERE
    uav_start_marker.action = uav_start_marker.ADD
    uav_start_marker.scale.x = 1.0#par.ws_model['robot_radius']*2
    uav_start_marker.scale.y = 1.0#par.ws_model['robot_radius']*2
    uav_start_marker.scale.z = 1.0#0.005*par.RealScale
    uav_start_marker.color.r = 255.0/255.0
    uav_start_marker.color.g = 0.0/255.0
    uav_start_marker.color.b = 0.0/255.0
    uav_start_marker.color.a = 1.0
    #uav_start_marker.lifetime = rospy.Duration(0)
    uav_start_marker.pose.orientation.x = 0.0
    uav_start_marker.pose.orientation.y = 0.0
    uav_start_marker.pose.orientation.z = 0.0
    uav_start_marker.pose.orientation.w = 1.0
    uav_start_marker.pose.position.x = cur_pose.pose.position.x#init_pos[0]
    uav_start_marker.pose.position.y = cur_pose.pose.position.y#init_pos[1] 
    uav_start_marker.pose.position.z = 0#goal.pose.position.z#init_pos[2] 
    #uav_start_marker_publisher.publish(uav_start_marker1)
    '''
    s1 = (3,-3)
    s2 = (5,-3)
    theta = atan2(s2[1]-s1[1],s2[0]-s1[0])
    d = dist(s1,s2)
    c = 2.0
    theta_ort = asin(c/d)

    theta_ort_left = theta+theta_ort # sudut RVO sebelah kiri
    bound_left = (s1[0]+d*cos(theta_ort_left), s1[1]+d*sin(theta_ort_left))
    theta_ort_right = theta-theta_ort # sudut RVO sebelah kanan
    bound_right = (s1[0]+d*cos(theta_ort_right), s1[1]+d*sin(theta_ort_right))
    '''
    (x0,y0) = (4,-2)
    (x1,y1) = (-3,0)

    c = 1.0
    theta = atan2(y1-y0,x1-x0)

    x2 = x0+c*sin(theta)
    y2 = y0-c*cos(theta)

    x3 = x1-c*sin(theta)
    y3 = y1+c*cos(theta)

    x4 = x0-c*sin(theta)
    y4 = y0+c*cos(theta)

    x5 = x1+c*sin(theta)
    y5 = y1-c*cos(theta)
    #print bound_left,bound_right
    topic = 'arrow1_marker'
    arrow1_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
    arrow1_marker = Marker()
    arrow1_marker.header.frame_id = "map"
    arrow1_marker.type = arrow1_marker.ARROW
    arrow1_marker.action = arrow1_marker.ADD
    arrow1_marker.scale.x = 0.2
    arrow1_marker.scale.y = 0.2
    arrow1_marker.scale.z = 0.2
    arrow1_marker.color.r = 255.0/255.0
    arrow1_marker.color.g = 0.0/255.0
    arrow1_marker.color.b = 255.0/255.0
    arrow1_marker.color.a = 255.0/255.0
    arrow1_marker.points = [0,0]
    p_rrt = Point()
    p_rrt.x = x0
    p_rrt.y = y0
    p_rrt.z = 0.0
    arrow1_marker.points[0] = p_rrt
    p_rrt = Point()
    p_rrt.x = x1
    p_rrt.y = y1
    p_rrt.z = 0.0
    arrow1_marker.points[1] = p_rrt   

    topic = 'arrow2_marker'
    arrow2_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
    arrow2_marker = Marker()
    arrow2_marker.header.frame_id = "map"
    arrow2_marker.type = arrow2_marker.ARROW
    arrow2_marker.action = arrow2_marker.ADD
    arrow2_marker.scale.x = 0.2
    arrow2_marker.scale.y = 0.2
    arrow2_marker.scale.z = 0.2
    arrow2_marker.color.r = 0.0/255.0
    arrow2_marker.color.g = 0.0/255.0
    arrow2_marker.color.b = 255.0/255.0
    arrow2_marker.color.a = 255.0/255.0
    arrow2_marker.points = [0,0]
    p_rrt = Point()
    p_rrt.x = x2
    p_rrt.y = y2
    p_rrt.z = 0.0
    arrow2_marker.points[0] = p_rrt
    p_rrt = Point()
    p_rrt.x = x5
    p_rrt.y = y5
    p_rrt.z = 0.0
    arrow2_marker.points[1] = p_rrt    

    topic = 'arrow3_marker'
    arrow3_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
    arrow3_marker = Marker()
    arrow3_marker.header.frame_id = "map"
    arrow3_marker.type = arrow3_marker.ARROW
    arrow3_marker.action = arrow3_marker.ADD
    arrow3_marker.scale.x = 0.2
    arrow3_marker.scale.y = 0.2
    arrow3_marker.scale.z = 0.2
    arrow3_marker.color.r = 255.0/255.0
    arrow3_marker.color.g = 0.0/255.0
    arrow3_marker.color.b = 0.0/255.0
    arrow3_marker.color.a = 255.0/255.0
    arrow3_marker.points = [0,0]
    p_rrt = Point()
    p_rrt.x = x4
    p_rrt.y = y4
    p_rrt.z = 0.0
    arrow3_marker.points[0] = p_rrt
    p_rrt = Point()
    p_rrt.x = x3
    p_rrt.y = y3
    p_rrt.z = 0.0
    arrow3_marker.points[1] = p_rrt

    #print freeObstacleWithClearance((4.0,5.3),(8.0,10.0),1.0)
    print freeObstacleWithClearance((-25.6,-25.6),(25.6,25.6),1.0)
    #print freeObstacleWithClearance((-25.6,-25.6),(-25.6,25.6),1.0)
    print isInside([(0,0),(10,0),(10,10),(0,10)],(4,2))
    print isInside([(0,0),(10,0),(10,10),(0,10)],(14,2))
    print isInside([(5,0),(6,3),(1,5),(0,2)],(6,6))
    print isInside([(5,0),(6,3),(1,5),(0,2)],(4,2))
    sizemap = 512
    og = OccupancyGrid()
    og_publisher = rospy.Publisher('/map',OccupancyGrid,queue_size=10)
    og.header.frame_id = "map"
    og.info.resolution = 1.0
    og.info.width = grid_size
    og.info.height = grid_size
    og.info.origin.position.x = -grid_size/2*og.info.resolution
    og.info.origin.position.y = -grid_size/2*og.info.resolution
    og.info.origin.position.z = 0.0
    og.info.origin.orientation.x = 0.0
    og.info.origin.orientation.y = 0.0
    og.info.origin.orientation.z = 0.0
    og.info.origin.orientation.w = 1.0
    listog = []
    gm = np.zeros((sizemap,sizemap))
    for i in range(0,sizemap):
        for j in range(0,sizemap):
            if(i == 100):
                    gm[i][j] = 100
                    listog.append(100)
            else:
                listog.append(0)

    t0 = time.time()
    setmap = inflate(gm,10)
    t1 = time.time()
    #listc = []
    #comap = np.zeros((sizemap,sizemap))
    arr = [0 for i in range(sizemap*sizemap)] #for j in range(sizemap)]
    sizemap_sizemap = sizemap*sizemap
    for i in setmap:
        idx = i[0]*sizemap+i[1]
        #if(idx < sizemap_sizemap):
        try:
            arr[idx]=100
        except:
            pass
            #comap[i[0]][i[1]]=100
    '''
    for i in range(0,sizemap):
        for j in range(0,sizemap):
            if(comap[i][j] == 100):
                listc.append(100)
    '''
    t2 = time.time()
    og.data = arr#listc#listog

    
    print "time",t2-t1
    print "time",t1-t0
    walls = set()
    for i in range(25,30):
        for j in range(0,70):
            walls.add((i,j))
    for i in range(50,100):
        for j in range(35,40):
            walls.add((i,j))

    for i in range(55,60):
        for j in range(60,100):
            walls.add((i,j))
    '''
    for i in range(70,90):
        for j in range(10,30):
            walls.add((i,j))
    '''
    listog = []
    for i in range(0,grid_size):
        for j in range(0,grid_size):
            if (j,i) in walls:
                listog.append(100)
            else:
                listog.append(0)
    og.data = listog
    rrt_star = True
    NUMNODES = 50
    map_size = grid_size

    rrt_topic = 'rrt_marker'
    rrt_marker_publisher = rospy.Publisher(rrt_topic, Marker,queue_size=10)
    rrt_marker = Marker()
    rrt_marker.header.frame_id = "map"
    #rrt_marker.header.stamp=rospy.Time.now()
    rrt_marker.id = 1
    rrt_marker.type = rrt_marker.LINE_LIST
    rrt_marker.action = rrt_marker.ADD
    rrt_marker.scale.x = 0.1
    rrt_marker.scale.y = 0.1
    #rrt_marker.scale.z = 0.005*par.RealScale
    rrt_marker.pose.orientation.w = 1.0
    #rrt_marker.pose.position.x = 0#init_pos[0]
    #rrt_marker.pose.position.y = 0#init_pos[1] 
    #rrt_marker.pose.position.z = 0#init_pos[2] 
    rrt_marker.color.r =255.0/255.0
    rrt_marker.color.g= 0.0/255.0
    rrt_marker.color.b =0.0/255.0
    rrt_marker.color.a =1.0
    #rrt_marker.lifetime = rospy.Duration(0)

    rrt_marker_list = []
    rrt_marker_publisher_list = []
    uav_start_marker_list = []
    uav_start_marker_publisher_list = []

    rrt_marker_color = [(255,0,0),(0,255,0),(0,0,255),(255,255,0)]
    uav_marker_color = [(255,255,0),(0,0,255),(0,255,0),(255,0,0)]
    #agent_pos = [(-49,-49),(-49,49),(49,-49),(49,49)]
    agent_pos = [(-4,-4),(-4,4),(4,-4),(4,4)]
    agent_tree = []
    agent_nodes = []
    agent_costs = []
    agent_node_list = []
    for a in range(0,NUM_OF_AGENT):
        agent_node_list.append(0)
        agent_nodes.append([agent_pos[a]])
        agent_tree.append({agent_pos[a]:None})
        agent_costs.append({})

        for b in range(0,NUM_OF_AGENT):
            agent_costs[a][agent_pos[b]]=0
        rrt_marker_publisher = rospy.Publisher('rrt_topic_'+repr(a), Marker,queue_size=10)
        rrt_marker = Marker()
        rrt_marker.header.frame_id = "map"
        #rrt_marker.header.stamp=rospy.Time.now()
        rrt_marker.id = 1
        rrt_marker.type = rrt_marker.LINE_LIST
        rrt_marker.action = rrt_marker.ADD
        rrt_marker.scale.x = 0.3
        rrt_marker.scale.y = 0.3
        rrt_marker.color.r = rrt_marker_color[a][0]/255.0
        rrt_marker.color.g = rrt_marker_color[a][1]/255.0
        rrt_marker.color.b = rrt_marker_color[a][2]/255.0
        rrt_marker.color.a = 1.0
        rrt_marker_list.append(rrt_marker)
        rrt_marker_publisher_list.append(rrt_marker_publisher)
        uav_start_marker_publisher = rospy.Publisher('uav_start_marker_'+repr(a), Marker,queue_size=10)
        uav_start_marker = Marker()
        uav_start_marker.header.frame_id = "map"
        #uav_start_marker.header.stamp=rospy.Time.now()
        uav_start_marker.type = uav_start_marker.SPHERE
        uav_start_marker.action = uav_start_marker.ADD
        uav_start_marker.scale.x = 1.0#par.ws_model['robot_radius']*2
        uav_start_marker.scale.y = 1.0#par.ws_model['robot_radius']*2
        uav_start_marker.scale.z = 1.0#0.005*par.RealScale
        uav_start_marker.id = a
        uav_start_marker.color.r = uav_marker_color[a][0]/255.0
        uav_start_marker.color.g = uav_marker_color[a][1]/255.0
        uav_start_marker.color.b = uav_marker_color[a][2]/255.0
        uav_start_marker.color.a = 1.0
        uav_start_marker.pose.position.x = agent_pos[a][0]
        uav_start_marker.pose.position.y = agent_pos[a][1]
        uav_start_marker_list.append(uav_start_marker)
        uav_start_marker_publisher_list.append(uav_start_marker_publisher)
    print "ok1"
    NUM_OF_NODES = 400
    for i in range(0,NUM_OF_NODES):
        node_added = False
        while(not node_added):
            rand = (random.random()-0.5)*map_size, (random.random()-0.5)*map_size
            #ra = random.randint(0,3)
            nn = agent_nodes[0][0]
            na = 0
            w = 0.8
            is_it_los = False
            for j in range(0,NUM_OF_AGENT):
                for p in agent_nodes[j]:
                    #if dist(p,rand) < dist(nn,rand):
                    if ((1-w)*dist(p,rand)+w*dist(agent_pos[j],rand) <= (1-w)*dist(nn,rand)+w*dist(agent_pos[na],rand)) and LineOfSight(walls,p,rand):
                        nn = p
                        na = j
                        agent_node_list[na] = agent_node_list[na] + 1
                        is_it_los = True

            newnode = step_from_to(nn,rand)
            near = []
            for j in range(0,NUM_OF_AGENT):
                near.append([])
                for n in agent_nodes[j]:
                    if(dist(n,newnode) < EPSILON*2):
                        near[j].append(n)

            xmin = nn
            cmin = agent_costs[na][nn]+dist(nn,newnode)
            #came_from[newnode]=nn
            for j in range(0,NUM_OF_AGENT):
                for ne in near[j]:
                    if LineOfSight(walls,ne,newnode) and agent_costs[j][nn] + dist(newnode,ne) < cmin:
                        xmin = ne
                        #cost[ne]=cost[newnode]+dist(newnode,ne)
                        #came_from[ne]=newnode
                        cmin=agent_costs[j][ne]+dist(ne,newnode)
                        na = j
            agent_tree[na][newnode] =xmin
            agent_costs[na][newnode] = cmin
            for j in range(0,NUM_OF_AGENT):
                for ne in near[j]:
                    if LineOfSight(walls,newnode,ne) and agent_costs[j][nn]+dist(newnode,ne) < agent_costs[j][ne]:
                        #xparent = came_from[ne]
                        agent_tree[j][ne] = newnode
                        agent_costs[j][ne] = agent_costs[j][newnode]+dist(newnode,ne)
            #if is_it_los == False:
            #    continue

            lines = []
            for j in range(0,NUM_OF_AGENT):
                if(j != na):
                    agent_line = []
                    for no in agent_tree[j]:
                        if agent_tree[j][no] != None:
                            agent_line.append((no,agent_tree[j][no]))
                    lines = lines+agent_line
            #print i,a
            #print lines
            if(LineOfSight(walls,nn,newnode) and (not is_line_intersect_with_lines(lines,(nn,newnode))) and is_it_los):
                agent_nodes[na].append(newnode)
                #nodes_and_parent.append((nn,newnode))
                agent_tree[na][newnode]=nn
                #came_from[newnode]=nn
                node_added = True
    print "ok2"     
    #print agent_tree       
    for a in range(0,NUM_OF_AGENT):
        rrt_marker_temp = []
        for no in agent_tree[a]:
            if agent_tree[a][no] != None:
                p_rrt = Point()
                p_rrt.x=no[0]
                p_rrt.y=no[1]
                p_rrt.z=0.0
                rrt_marker_temp.append(p_rrt)
                p_rrt = Point()
                #print agent_tree[a][no]
                p_rrt.x=agent_tree[a][no][0]
                p_rrt.y=agent_tree[a][no][1]
                p_rrt.z=0.0
                rrt_marker_temp.append(p_rrt)
        rrt_marker_list[a].points = rrt_marker_temp
    print "ok3"
    print is_line_intersection(((5,5),(0,0)),((0,5),(5,0)))
    print is_line_intersection(((5,5),(0,0)),((5,6),(0,2)))
    #for a in range(0,NUM_OF_AGENT):
    #    print uav_start_marker_list[a]
    '''
    nodes = [(goal.pose.position.x,goal.pose.position.y)]
    rrt_marker_temp = []
    #for i in range(NUMNODES):
    i=0
    print "Calculate RRT"
    path_found = False
    nodes_and_parent = []
    came_from = {}
    came_from[nodes[0]]=None
    cost = {}
    cost[nodes[0]]=0
    #list_time_los = []
    while (i<NUMNODES) and (not path_found):	
        rand = (random.random()-0.5)*map_size, (random.random()-0.5)*map_size
        nn = nodes[0]
        for p in nodes:
            if dist(p,rand) < dist(nn,rand):
                nn = p

        newnode = step_from_to(nn,rand)
        if(not rrt_star):
            #if(freeObstacle(nn,newnode)):
            #if(freeObstacleWithClearance(nn,newnode,RRT_SAFETY)):
            #start_los_time = time.time()
            is_los = LineOfSight(walls,nn,newnode)
            #list_time_los.append(time.time()-start_los_time)
            if(is_los):
                nodes.append(newnode)
                #nodes_and_parent.append((nn,newnode))
                came_from[newnode]=nn
                p_rrt = Point()
                p_rrt.x=nn[0]
                p_rrt.y=nn[1]
                p_rrt.z=0.0
                rrt_marker_temp.append(p_rrt)
                p_rrt = Point()
                p_rrt.x=newnode[0]
                p_rrt.y=newnode[1]
                p_rrt.z=0.0
                rrt_marker_temp.append(p_rrt)
                if(dist(newnode,(cur_pose.pose.position.x,cur_pose.pose.position.y))<2.0):
                    path_found = True
                    print "Path found",i
                #rrt_marker.header.frame_id=map_data.header.frame_id
                #rrt_marker.header.stamp=rospy.Time.now()
                #rrt_marker.lifetime = rospy.Duration(0)

                #pygame.draw.line(screen,white,nn,newnode)
                #pygame.display.update()
                #print nn, " ",newnode
                i=i+1
            else:
                i=i+1
        else: # RRT*
            #if(freeObstacle(nn,newnode)):
            if LineOfSight(walls,nn,newnode):
                near = []
                for n in nodes:
                    if(dist(n,newnode) < EPSILON*2):
                        near.append(n)
                #cost[newnode]=dist(nn,newnode)
                nodes.append(newnode)
                xmin = nn
                cmin = cost[nn]+dist(nn,newnode)
                #came_from[newnode]=nn
                for ne in near:
                    if LineOfSight(walls,ne,newnode) and cost[nn] + dist(newnode,ne) < cmin:
                        xmin = ne
                        #cost[ne]=cost[newnode]+dist(newnode,ne)
                        #came_from[ne]=newnode
                        cmin=cost[ne]+dist(ne,newnode)
                came_from[newnode] =xmin
                cost[newnode] = cmin
                for ne in near:
                    if LineOfSight(walls,newnode,ne) and cost[nn]+dist(newnode,ne) < cost[ne]:
                        #xparent = came_from[ne]
                        came_from[ne] = newnode
                        cost[ne] = cost[newnode]+dist(newnode,ne)
                
                #nodes_and_parent.append((nn,newnode))
                #came_from[newnode]=nn
                
                #if(dist(newnode,(cur_pose.pose.position.x,cur_pose.pose.position.y))<2.0):
                #    #path_found = True
                #    print "Path found",i
                    
                #rrt_marker.header.frame_id=map_data.header.frame_id
                #rrt_marker.header.stamp=rospy.Time.now()
                #rrt_marker.lifetime = rospy.Duration(0)

                #pygame.draw.line(screen,white,nn,newnode)
                #pygame.display.update()
                #print nn, " ",newnode
                i=i+1
            else:
                pass
                #print i,"obs det"
        if (path_found):
            replanning_rrt = False
            break
    for no in came_from:
        if(came_from[no] != None):
            p_rrt = Point()
            p_rrt.x=no[0]
            p_rrt.y=no[1]
            p_rrt.z=0.0
            rrt_marker_temp.append(p_rrt)
            p_rrt = Point()
            p_rrt.x=came_from[no][0]
            p_rrt.y=came_from[no][1]
            p_rrt.z=0.0
            rrt_marker_temp.append(p_rrt)
    rrt_marker.points = rrt_marker_temp
    if(not path_found):
        print "Path not found"
        print nodes
    if(True):
        rrt_marker.points = rrt_marker_temp
        #print "Calculate RRT Path"
        #print "came_from",came_from
        #print "nodes", (cur_pose.pose.position.x,cur_pose.pose.position.y),nodes[-1],nodes[0]
        rrt_path_temp=reconstruct_path(came_from,nodes[0],nodes[-1])
        dr = 0
        for rr in range(0,len(rrt_path_temp)-2):
            dr=dr+dist((rrt_path_temp[rr][0],rrt_path_temp[rr][1]),(rrt_path_temp[rr+1][0],rrt_path_temp[rr+1][1]))
        #if(len(rrt_path_temp) <= prev_length_rrt_path) or (goal_changed) or (not global_los):
        if(dr < 3.0*dist((cur_pose.pose.position.x,cur_pose.pose.position.y),(goal.pose.position.x,goal.pose.position.y)) or (not global_los)): #not global_los) or (len(rrt_path_temp) <= prev_length_rrt_path+5):
            print "rrt generated cz global_los"
            prev_length_rrt_path = len(rrt_path_temp)
            rrt_path = rrt_path_temp
            #rrt_marker.header.frame_id=map_data.header.frame_id
        rrt_marker_publisher.publish(rrt_marker)
        print nodes
    '''
    while(not rospy.is_shutdown()):
        
        #print "aaa"
        #uav_goal_marker_publisher.publish(uav_goal_marker)
        #uav_start_marker1_publisher.publish(uav_start_marker1)
        #rrt_marker_publisher.publish(rrt_marker)
        og_publisher.publish(og)
        for a in range(0,NUM_OF_AGENT):
            rrt_marker_publisher_list[a].publish(rrt_marker_list[a])
            uav_start_marker_publisher_list[a].publish(uav_start_marker_list[a])

        r.sleep()
        #print "bb"
    print "cc"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
	main(sys.argv)