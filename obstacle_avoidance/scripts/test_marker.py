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


def main(args):
    rospy.init_node("test_marker")
    r=rospy.Rate(10)
    topic = 'themarker'
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
    uav_goal_marker.pose.position.x = 3#init_pos[0]
    uav_goal_marker.pose.position.y = 3#init_pos[1] 
    uav_goal_marker.pose.position.z = 0#goal.pose.position.z#init_pos[2] 
    uav_goal_marker_publisher.publish(uav_goal_marker)
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
    og.info.width = sizemap
    og.info.height = sizemap
    og.info.origin.position.x = -sizemap/2*og.info.resolution
    og.info.origin.position.y = -sizemap/2*og.info.resolution
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
    while(not rospy.is_shutdown()):
        uav_goal_marker.header.stamp=rospy.Time.now()
        arrow1_marker_publisher.publish(arrow1_marker)
        arrow2_marker_publisher.publish(arrow2_marker)
        arrow3_marker_publisher.publish(arrow3_marker)
        og_publisher.publish(og)
        #print "aaa"
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