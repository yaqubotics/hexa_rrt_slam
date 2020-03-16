#!/usr/bin/env python
import argparse
from threading import Thread
from time import sleep
import datetime

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
from VelocityController import VelocityController
from nav_msgs.msg import GridCells

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import local_planning as locplan

import compute_coverage as com
import load_parameter as lp

from tf.transformations import euler_from_quaternion

from global_planning import *
from global_planning_function import *

import tf_conversions
import tf2_ros

par = lp.Parameter
listwalls = []
target = Pose()
sub_target = Pose()
sub_target_heading = 0.0

dir_path = os.path.dirname(os.path.realpath(__file__))
list_error = [[],[],[],[]]
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
xvel = 0
yvel = 0
state = 1
global_obs_detected = False
def get():
        global xvel,yvel
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                print "up"
                yvel=yvel+1
        elif k=='\x1b[B':
                print "down"
                yvel=yvel-1
        elif k=='\x1b[C':
                print "right"
                xvel=xvel+1
        elif k=='\x1b[D':
                print "left"
                xvel=xvel-1
        elif k=='s':
        		print "s"
        else:
                print "not an arrow key!"
        print "("+str(xvel)+" , "+str(yvel)+")"


def animate(i):
    global ax1,ax2,ax3,list_error
    #print "animate"
    #print i
    #ax1 = arg0
    #ax2 = arg1
    #ax3 = arg2
    
    if(len(list_error[0]) == len(list_error[1])):
    	ax1.clear()
    	ax1.plot(list_error[0], list_error[1])
    if(len(list_error[0]) == len(list_error[2])):
    	ax2.clear()
    	ax2.plot(list_error[0],list_error[2])
    if(len(list_error[0]) == len(list_error[3])):
    	ax3.clear()
    	ax3.plot(list_error[0],list_error[3])

def plot_error_live():
	global ax1,ax2,ax3
	fig = plt.figure()
	ax1 = fig.add_subplot(3,1,1)
	ax2 = fig.add_subplot(3,1,2)
	ax3 = fig.add_subplot(3,1,3)
	ani = animation.FuncAnimation(fig, animate, interval=1000)
	plt.show()

current_state = State()
current_velocity = TwistStamped()
cur_pose = PoseStamped()
current_position = [0,0,0]
list_current_position = []
list_current_velocity = []
list_velocity_angle = dict()
list_distance_obs = dict()
usingvelocitycontrol = False
usingpositioncontrol = False
start_calculate_and_publish_wall = False
list_angle_and_distance_obs = (None,[],[])
list_obstacle_vertex = (None,[],[])

header = Header()
for i in range(4):
	list_current_position.append(PoseStamped())
	list_current_velocity.append(TwistStamped())

#global variable for A star
print par.NumberOfPoints,par.NumberOfPoints
diagram = GridWithWeights(par.NumberOfPoints,par.NumberOfPoints)
diagram.walls = []

def state_cb(msg_var):
	global current_state
	if(current_state.armed != msg_var.armed):
		rospy.loginfo("armed state changed from {0} to {1}".format(
                current_state.armed, msg_var.armed))

	if(current_state.connected != msg_var.connected):
		rospy.loginfo("connected changed from {0} to {1}".format(
                current_state.connected, msg_var.connected))

	if(current_state.mode != msg_var.mode):
		rospy.loginfo("mode changed from {0} to {1}".format(
                current_state.mode, msg_var.mode))

	current_state = msg_var

def velocity_cb(msg_var,data_var): #, data_var):
	global list_current_velocity
	idx = data_var
	list_current_velocity[idx] = msg_var

def position_cb(msg_var,data_var):
	global current_position, list_current_position,cur_pose
	#print data_var, msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z
	idx = data_var[0]
	uav_idx = data_var[1]
	
	if(idx == 0):
		list_current_position[idx] = msg_var
	if(idx == 1):
		list_current_position[idx].pose.position.x = msg_var.pose.position.x+28
		list_current_position[idx].pose.position.y = msg_var.pose.position.y+28
		list_current_position[idx].pose.position.z = msg_var.pose.position.z 
	if(idx == 2):
		list_current_position[idx].pose.position.x = msg_var.pose.position.x
		list_current_position[idx].pose.position.y = msg_var.pose.position.y+17
		list_current_position[idx].pose.position.z = msg_var.pose.position.z 
	if(idx == uav_idx):
		if(idx == 0):
			cur_pose = msg_var
			current_position = (msg_var.pose.position.x,msg_var.pose.position.y,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		if(idx == 1):
			cur_pose.pose.position.x = msg_var.pose.position.x+28
			cur_pose.pose.position.y = msg_var.pose.position.y+28
			cur_pose.pose.position.z = msg_var.pose.position.z
			current_position = (msg_var.pose.position.x+28,msg_var.pose.position.y+28,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		if(idx == 2):
			cur_pose.pose.position.x = msg_var.pose.position.x
			cur_pose.pose.position.y = msg_var.pose.position.y+17
			cur_pose.pose.position.z = msg_var.pose.position.z
			current_position = (msg_var.pose.position.x,msg_var.pose.position.y+17,msg_var.pose.position.z,msg_var.header.stamp.to_sec())
		
		br = tf2_ros.TransformBroadcaster()
		t = TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "uav"+str(idx+1)+"/odom"
		t.child_frame_id = "uav"+str(idx+1)+"/base_link"
		t.transform.translation.x = cur_pose.pose.position.x
		t.transform.translation.y = cur_pose.pose.position.y
		t.transform.translation.z = cur_pose.pose.position.z
		t.transform.rotation.x = msg_var.pose.orientation.x
		t.transform.rotation.y = msg_var.pose.orientation.y
		t.transform.rotation.z = msg_var.pose.orientation.z
		t.transform.rotation.w = msg_var.pose.orientation.w

		br.sendTransform(t)
	#print list_velocity_angle,list_distance_obs

def scan_lidar_cb(msg_var,data_var):
	global list_velocity_angle,list_distance_obs,global_obs_detected,list_angle_and_distance_obs, list_obstacle_vertex
	if(msg_var.header.frame_id == "uav"+data_var+"/sonar2_link"):
		lcp = list_current_position[int(data_var)-1].pose
		obs_detected = False
		list_obs = []
		list_angle = []
		for i in range(len(msg_var.ranges)):
			if(msg_var.intensities[i] != 0.0):
				print "intens",msg_var.intensities[i] 
			if(msg_var.ranges[i] != np.inf) and (i is not len(msg_var.ranges)-1):
				if(not obs_detected):
					list_sub_obs = []
					list_sub_angle = []
					obs_detected = True
					global_obs_detected = True
				list_sub_obs.append(msg_var.ranges[i])
				list_sub_angle.append(msg_var.angle_min+i*msg_var.angle_increment)
			else:
				if(obs_detected):
					list_obs.append(list_sub_obs)
					list_angle.append(list_sub_angle)
					obs_detected = False
					global_obs_detected = False

		#print "obs rad",list_obs
		#print "angle",list_angle 
		list_all_velocity_angle = []
		list_all_distance_obs = []
		for i in range(len(list_angle)):
			#print "angle["+repr(i+1)+"]:",list_angle[i][0],list_angle[i][len(list_angle[i])-1]
			list_velocity_angle[i] = [list_angle[i][0],list_angle[i][len(list_angle[i])-1]]
			list_distance_obs[i] = [list_obs[i][0],np.min(list_obs[i]),list_obs[i][len(list_obs[i])-1]]
			list_all_velocity_angle.append(list_angle[i])
			list_all_distance_obs.append(list_obs[i])
		if(len(list_velocity_angle) > len(list_angle)):
			for i in range(len(list_velocity_angle)-len(list_angle)):
				del list_velocity_angle[len(list_angle)+i]
				del list_distance_obs[len(list_angle)+i]
		list_angle_and_distance_obs = (lcp,list_all_velocity_angle,list_all_distance_obs)
		list_obstacle_vertex = (lcp,list_velocity_angle,list_distance_obs)

def rad2deg(inp):
	return inp*180.0/np.pi
def deg2rad(inp):
	return inp/180.0*np.pi

def calculate_and_publish_wall(publisher,path_publisher,idx):
	global list_angle_and_distance_obs, diagram, list_velocity_angle, list_distance_obs, list_obstacle_vertex, target, sub_target, sub_target_heading, vController, list_current_position,usingvelocitycontrol
	last_request = rospy.Time.now()
	global_planning_method = None #"Theta_star"
	local_walls = []
	list_of_cells = []
	list_of_cell_path = []
	while (not usingvelocitycontrol):
		pass
	while(True):
		if(rospy.Time.now()-last_request > rospy.Duration(0.2)):
			last_request = rospy.Time.now()
			if(global_planning_method == "A_star") or (global_planning_method == "Theta_star"):
				lado = list_angle_and_distance_obs
				if(len(lado[1]) > 0):
					
					Astar_path = GridCells()
					Astar_path.header.frame_id = "world"
					Astar_path.header.stamp = rospy.Time.now()
					Astar_path.cell_width = 1.0
					Astar_path.cell_height = 1.0

					Astar_grid = GridCells()
					Astar_grid.header.frame_id = "world"
					Astar_grid.header.stamp = rospy.Time.now()
					Astar_grid.cell_width = 1.0
					Astar_grid.cell_height = 1.0
					
					quat_orient = lado[0]
					quat_arr = np.array([quat_orient.orientation.x,quat_orient.orientation.y,quat_orient.orientation.z,quat_orient.orientation.w])
					att = euler_from_quaternion(quat_arr,'sxyz')
					#print "pitch",att[1]*180.0/np.pi

					
					posex = lado[0].position.x
					posey = lado[0].position.y
					
					for i in range(len(lado[1])):
						for j in range(len(lado[1][i])):

							cell_float_x = posex+lado[2][i][j]*np.cos(att[2]+lado[1][i][j])*np.cos(att[1])
							cell_float_y = posey+lado[2][i][j]*np.sin(att[2]+lado[1][i][j])*np.cos(att[1])
							the_cells = Point()
							the_cells.x = cell_float_x#int(np.round(cell_float_x))
							the_cells.y = cell_float_y#int(np.round(cell_float_y))
							list_of_cells.append(the_cells) 
							if((the_cells.x,the_cells.y) not in local_walls):
								if(abs(att[0]) > deg2rad(3)) and (abs(att[1]) > deg2rad(3)):
									local_walls.append((the_cells.x,the_cells.y))
							'''
							cell_float_x = posex+(lado[2][i][j]-1)*np.cos(att[2]+lado[1][i][j])*np.cos(att[1])
							cell_float_y = posey+(lado[2][i][j]-1)*np.sin(att[2]+lado[1][i][j])*np.cos(att[1])
							the_cells = Point()
							the_cells.x = int(np.round(cell_float_x))
							the_cells.y = int(np.round(cell_float_y))
							list_of_cells.append(the_cells) 
							if((the_cells.x,the_cells.y) not in local_walls):
								local_walls.append((the_cells.x,the_cells.y))
							'''
					Astar_grid.cells = list_of_cells
					diagram.walls = local_walls
					
					if(global_planning_method == "A_star"): 
						print "walls",local_walls
						start_plan = (int(posex),int(posey))
						print "start_plan", start_plan
						goal_plan = (29,29)
						came_from, cost_so_far, priority, cost, heu = a_star_search(diagram, start_plan, goal_plan)
						path=reconstruct_path(came_from, start=start_plan, goal=goal_plan)
						print "path",path
						list_of_cell_path = []
						for i in range(len(path)):
							the_cells = Point()
							the_cells.x = path[i][0]
							the_cells.y = path[i][1]
							the_cells.z = -0.8
							list_of_cell_path.append(the_cells)
						Astar_path.cells = list_of_cell_path
						path_publisher.publish(Astar_path)
						publisher.publish(Astar_grid)
					elif(global_planning_method == "Theta_star"):
						#find the vertex
						goal_plan = (29,29)
						list_of_vertex = []
						list_of_cell_path = []
						for (x,y) in local_walls:
							neighbor_of_i = [(x-1,y-1),(x-1,y),(x-1,y+1),(x,y+1),(x,y-1),(x+1,y+1),(x+1,y),(x+1,y-1)]
							num_of_neighbor = 0
							for elem in neighbor_of_i:
								if elem in local_walls:
									num_of_neighbor+=1
							if(num_of_neighbor < 5):
								list_of_vertex.append((x,y))
								the_cells = Point()
								the_cells.x = x
								the_cells.y = y
								the_cells.z = -0.8
								list_of_cell_path.append(the_cells)
						list_of_vertex.append(goal_plan)
						the_cells = Point()
						the_cells.x = goal_plan[0]
						the_cells.y = goal_plan[1]
						the_cells.z = -0.8
						list_of_cell_path.append(the_cells)
						Astar_path.cells = list_of_cell_path
						path_publisher.publish(Astar_path)
						publisher.publish(Astar_grid)
						#print "list of vertex",list_of_vertex
						vController.setHeadingTarget(deg2rad(45.0))
						vController.setTarget(target)
						#find the list of vertex with minimum distance to goal
						
				else:
					Astar_grid = GridCells()
					Astar_grid.header.frame_id = "world"
					Astar_grid.header.stamp = rospy.Time.now()
					Astar_grid.cell_width = 1.0
					Astar_grid.cell_height = 1.0
					Astar_grid.cells = list_of_cells
					diagram.walls = []
					publisher.publish(Astar_grid)

					Astar_path = GridCells()
					Astar_path.header.frame_id = "world"
					Astar_path.header.stamp = rospy.Time.now()
					Astar_path.cell_width = 1.0
					Astar_path.cell_height = 1.0
					Astar_path.cells = list_of_cell_path
					diagram.walls = []
					path_publisher.publish(Astar_path)
			elif(global_planning_method == "Simple_VO"):
				lov = list_obstacle_vertex
				if(distance2D((target.position.x,target.position.y),(list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y)) < 2.0):
					vController.setHeadingTarget(deg2rad(45.0))
					vController.setTarget(target)
				elif(len(lov[2]) > 0):
					Thetastar_path = GridCells()
					Thetastar_path.header.frame_id = "world"
					Thetastar_path.header.stamp = rospy.Time.now()
					Thetastar_path.cell_width = 1.0
					Thetastar_path.cell_height = 1.0
					clearence = 2.0
					pos_uav = lov[0].position
					heading_to_goal_target = np.arctan2(target.position.y-pos_uav.y,target.position.x-pos_uav.x)
					list_sub_goal_target = []
					is_heading_to_target_obstructed = False
					for i in range(len(lov[2])):

						r = lov[2][i][2]
						if(abs(r) > 0.1):
							clearence_angle_left = np.arccos((2*r*r-clearence*clearence)/(2*r*r))
						else:
							clearence_angle_left = deg2rad(10.0)
						r = lov[2][i][0]
						if(abs(r) > 0.1):
							clearence_angle_right = np.arccos((2*r*r-clearence*clearence)/(2*r*r))
						else:
							clearence_angle_right = deg2rad(10.0)
						quat_orient = lov[0]
						
						quat_arr = np.array([quat_orient.orientation.x,quat_orient.orientation.y,quat_orient.orientation.z,quat_orient.orientation.w])
						att = euler_from_quaternion(quat_arr,'sxyz')
						theta_ort_left = att[2]+lov[1][i][1]+clearence_angle_left
						theta_ort_right = att[2]+lov[1][i][0]-clearence_angle_right
						#print "clearence",rad2deg(clearence_angle_left),rad2deg(clearence_angle_right)
						print "compare heading", rad2deg(theta_ort_left),rad2deg(heading_to_goal_target),rad2deg(theta_ort_right)
						if locplan.in_between(theta_ort_right,heading_to_goal_target,theta_ort_left):
							is_heading_to_target_obstructed = True
						list_sub_goal_target.append((pos_uav.x+r*np.cos(theta_ort_left),pos_uav.y+r*np.sin(theta_ort_left)))
						list_sub_goal_target.append((pos_uav.x+r*np.cos(theta_ort_right),pos_uav.y+r*np.sin(theta_ort_right)))

						if(is_heading_to_target_obstructed): #direct heading to goal obstruct by obstacle
							print "create sub goal Theta*"
							list_distance_sub_goal_to_the_goal = []
							for i in range(len(list_sub_goal_target)):
								list_distance_sub_goal_to_the_goal.append(distance2D(list_sub_goal_target[i],(target.position.x,target.position.y)))
							min_idx = np.argmin(list_distance_sub_goal_to_the_goal)
							sub_target = Pose()
							sub_target.position.x = list_sub_goal_target[min_idx][0]
							sub_target.position.y = list_sub_goal_target[min_idx][1]
							sub_target.position.z = target.position.z
							print "sub_target",sub_target.position.x,sub_target.position.y
							sub_target_heading = np.arctan2(list_sub_goal_target[min_idx][1]-pos_uav.y,list_sub_goal_target[min_idx][0]-pos_uav.x)
							vController.setHeadingTarget(sub_target_heading)
							vController.setTarget(sub_target)
						else: # go directly to the target
							#print "goes to the goal target directly"
							sub_target = Pose()
							sub_target.position.x = target.position.x
							sub_target.position.y = target.position.y
							sub_target.position.z = target.position.z
							sub_target_heading = heading_to_goal_target
							vController.setHeadingTarget(sub_target_heading)
							vController.setTarget(sub_target)
						the_cells = Point()
						the_cells.x = sub_target.position.x
						the_cells.y = sub_target.position.y
						Thetastar_path.cells = [the_cells]
						path_publisher.publish(Thetastar_path)
				else:
					Thetastar_path = GridCells()
					Thetastar_path.header.frame_id = "world"
					Thetastar_path.header.stamp = rospy.Time.now()
					Thetastar_path.cell_width = 1.0
					Thetastar_path.cell_height = 1.0
					Thetastar_path.cells = []
					path_publisher.publish(Thetastar_path)

def update_uav_marker(marker,pos):
  temp_marker = marker
  temp_marker.pose.orientation.w = pos[3]
  temp_marker.pose.position.x = pos[0]
  temp_marker.pose.position.y = pos[1]
  temp_marker.pose.position.z = pos[2]

  return temp_marker

def update_uav_goal_marker(marker,pos):
  temp_marker = marker
  temp_marker.pose.orientation.w = pos[3]
  temp_marker.pose.position.x = pos[0]
  temp_marker.pose.position.y = pos[1]
  temp_marker.pose.position.z = pos[2]

  return temp_marker

def key_function():
	global usingvelocitycontrol,xvel,yvel
	while(True):
		if(usingvelocitycontrol):
			get()

def distance3D(a,b):
	#return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
	a = (a.position.x,a.position.y,a.position.z)
	b = (b.position.x,b.position.y,b.position.z)
	return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

def distance2D(a,b):
	#return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2++(a[2]-b[2])**2)
	a = (a.position.x,a.position.y)
	b = (b.position.x,b.position.y)
	return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def PID_update(target,state,time):
	error = target-state

def vController_update(goal,curr):
	time = curr[3]
	outpur = TwistStamped()
	linear = Vector3()
	linear.x = PID_update(goal[0],curr[0],time)
	linear.x = PID_update(goal[1],curr[1],time)
	linear.x = PID_update(goal[2],curr[2],time)
	output.twist = Twist()
	output.twist.linear = linear
	return 

def publish_uav_position_rviz(br,x,y,z,uav_idx):
	br.sendTransform((x,y,z),
	              (0.0, 0.0, 0.0, 1.0),
	              rospy.Time.now(),
	              "uav_tf_"+uav_idx,
	              "world")

def main(args):
	global list_current_position, list_current_velocity, current_position, usingvelocitycontrol,usingpositioncontrol,xvel,yvel,state,cur_pose,list_error,target
	global global_obs_detected, list_velocity_angle, vController
	
	par.CurrentIteration = 1
	if(len(args) > 1):
		uav_ID = str(args[1])
	else:
		uav_ID = "1"
	idx = int(uav_ID)-1
	rospy.init_node("control_uav_"+uav_ID)
	print "UAV"+uav_ID
	vController = VelocityController()
	try:
		rospy.wait_for_service("/uav"+uav_ID+"/mavros/cmd/arming")
		rospy.wait_for_service("/uav"+uav_ID+"/mavros/set_mode")
	except rospy.ROSException:
		fail("failed to connect to service")
	state_sub=rospy.Subscriber("/uav"+uav_ID+"/mavros/state", State,queue_size=10,callback=state_cb)
	local_vel_pub=rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
	local_pos_pub=rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_position/local",PoseStamped,queue_size=10)
	local_pos_target=rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_raw/local",PositionTarget,queue_size=10)
	atittude_pub = rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=10)
	thr_pub = rospy.Publisher("/uav"+uav_ID+"/mavros/setpoint_attitude/att_throttle",Float64,queue_size=10)
	Astar_grid_pub = rospy.Publisher("/uav"+uav_ID+"/Astar_grid",GridCells,queue_size=10)
	Astar_path_pub = rospy.Publisher("/uav"+uav_ID+"/Astar_path",GridCells,queue_size=10)
	arming_client = rospy.ServiceProxy("/uav"+uav_ID+"/mavros/cmd/arming",CommandBool)
	set_mode_client = rospy.ServiceProxy("/uav"+uav_ID+"/mavros/set_mode",SetMode)
	for i in range(4):
		#velocity_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/velocity",TwistStamped,queue_size = 10,callback=velocity_cb, callback_args=i)
		position_sub = rospy.Subscriber("/uav"+repr(i+1)+"/mavros/local_position/pose",PoseStamped,queue_size = 10,callback=position_cb,callback_args=(i,int(uav_ID)-1))
	
	scan_lidar_sub = rospy.Subscriber("/scan",LaserScan,queue_size=10,callback=scan_lidar_cb,callback_args=uav_ID)
	br = tf.TransformBroadcaster()
	r=rospy.Rate(10)
	print "TRY TO CONNECT"
	while ((not rospy.is_shutdown()) and (not current_state.connected)):
		#rospy.spinOnce()
		r.sleep()
	#print(current_state.connected.__class__)
	rospy.loginfo("CURRENT STATE CONNECTED")
	poses = PoseStamped()
	#poses.pose = Pose()
	#poses.pose.position = Point()
	target = Pose()
	if(idx == 0):
		target.position.x = 0
		target.position.y = 0
	if(idx==1):
		target.position.x = 0
		target.position.y = 0
	if(idx==2):
		target.position.x = 0
		target.position.y = 0
	if(idx==3):
		target.position.x = 4
		target.position.y = 4
	target.position.z = 10
	poses.pose.position.x = target.position.x
	poses.pose.position.y = target.position.y
	poses.pose.position.z = target.position.z
	q=quaternion_from_euler(0,0,45*np.pi/180.0)
	poses.pose.orientation.x = q[0]
	poses.pose.orientation.y = q[1]
	poses.pose.orientation.z = q[2]
	poses.pose.orientation.w = q[3]
	i = 100
	#while((not rospy.is_shutdown()) and (i>0)):
	#	local_pos_pub.publish(poses)
	#	i = i-1
	rviz_visualization_start = False
	last_request = rospy.Time.now()
	count = 0
	if(idx==1):
		target.position.x = 28
		target.position.y = 28
	if(idx==2):
		target.position.x = 0
		target.position.y = 17
	#thread1 = Thread(target = key_function)
  	#thread1.start()

	style.use('fivethirtyeight')
  	thread2 = Thread(target = plot_error_live)
  	thread2.start()
  	thread3 = Thread(target = calculate_and_publish_wall, args = (Astar_grid_pub,Astar_path_pub,idx))
  	thread3.start()
  	#file_error_pos_x = open(dir_path+'/txt/error_pos_x.txt','w')
  	#file_error_pos_y = open(dir_path+'/txt/error_pos_y.txt','w')
  	#file_error_pos_z = open(dir_path+'/txt/error_pos_z.txt','w')
  	error_time = 0
  	print poses
  	uav_color = [255,0,0,255]
	topic = 'uav_marker_'+uav_ID
	uav_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	uav_marker = Marker()
	uav_marker.header.frame_id = "world"
	uav_marker.type = uav_marker.SPHERE
	uav_marker.action = uav_marker.ADD
	uav_marker.scale.x = par.ws_model['robot_radius']*3
	uav_marker.scale.y = par.ws_model['robot_radius']*3
	uav_marker.scale.z = 0.005*par.RealScale
	uav_marker.color.r = float(uav_color[0])/255
	uav_marker.color.g = float(uav_color[1])/255
	uav_marker.color.b = float(uav_color[2])/255
	uav_marker.color.a = float(uav_color[3])/255
	uav_marker.pose.orientation.w = 1.0
	uav_marker.pose.position.x = 0#init_pos[0]
	uav_marker.pose.position.y = 0#init_pos[1] 
	uav_marker.pose.position.z = 0#init_pos[2] 
	uav_marker_publisher.publish(uav_marker)

	uav_color = [255,255,255,255]
	topic = 'uav_goal_marker_'+uav_ID
	uav_goal_marker_publisher = rospy.Publisher(topic, Marker,queue_size=10)
	uav_goal_marker = Marker()
	uav_goal_marker.header.frame_id = "world"
	uav_goal_marker.type = uav_goal_marker.SPHERE
	uav_goal_marker.action = uav_goal_marker.ADD
	uav_goal_marker.scale.x = par.ws_model['robot_radius']*2
	uav_goal_marker.scale.y = par.ws_model['robot_radius']*2
	uav_goal_marker.scale.z = 0.005*par.RealScale
	uav_goal_marker.color.r = float(uav_color[0])/255
	uav_goal_marker.color.g = float(uav_color[1])/255
	uav_goal_marker.color.b = float(uav_color[2])/255
	uav_goal_marker.color.a = float(uav_color[3])/255
	uav_goal_marker.pose.orientation.w = 1.0
	uav_goal_marker.pose.position.x = 0#init_pos[0]
	uav_goal_marker.pose.position.y = 0#init_pos[1] 
	uav_goal_marker.pose.position.z = 0#init_pos[2] 
	uav_goal_marker_publisher.publish(uav_goal_marker)
	uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
	uav_goal_marker_publisher.publish(uav_goal_marker)
	last_request_rviz = rospy.Time.now()
	last_HRVO_request=rospy.Time.now()
	while(not rospy.is_shutdown()):
		if(rviz_visualization_start and (rospy.Time.now()-last_request_rviz > rospy.Duration(0.2))):
			publish_uav_position_rviz(br,list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z,uav_ID)
			uav_marker=update_uav_marker(uav_marker,(list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z,1.0))
			uav_marker_publisher.publish(uav_marker)
			last_request_rviz = rospy.Time.now()
		if((not rviz_visualization_start) and (current_state.mode != 'OFFBOARD') and (rospy.Time.now()-last_request > rospy.Duration(1.0))):
			offb_set_mode = set_mode_client(0,'OFFBOARD')
			if(offb_set_mode.mode_sent):
				rospy.loginfo("OFFBOARD ENABLED")
			last_request = rospy.Time.now()
		else:
			if((not current_state.armed) and (rospy.Time.now()-last_request > rospy.Duration(1.0))):
				arm_cmd = arming_client(True)
				if(arm_cmd.success):
					rospy.loginfo("VEHICLE ARMED")
					rviz_visualization_start = True
				last_request=rospy.Time.now()
		#print distance3D(cur_pose.pose,target)
		
		if(distance3D(cur_pose.pose,target) < 0.5) and (not usingvelocitycontrol):
			print "sampai"
			usingvelocitycontrol = True
			usingpositioncontrol = False
			target = Pose()
			if(idx == 0):
				target.position.x = 28
				target.position.y = 28
			if(idx == 1):
				target.position.x = 0
				target.position.y = 0
			if(idx == 2):
				target.position.x = 21
				target.position.y = 14
			if(idx == 3):
				target.position.x = 0
				target.position.y = 0
			
			target.position.z = 10
			print target
			psi_target = 45*np.pi/180.0 #np.pi/180.0 #np.arctan((self.target.position.y-self.prev_target.position.y)/(self.target.position.x-self.prev_target.position.x))#(linear.y,linear.x)
			
			diagram4 = GridWithWeights(par.NumberOfPoints,par.NumberOfPoints)
			diagram4.walls = []
			for i in range(par.NumberOfPoints): # berikan wall pada algoritma pathfinding A*
				for j in range(par.NumberOfPoints):
					if (par.ObstacleRegion[j,i]==0):
						diagram4.walls.append((i,j))
			for i in range(par.NumberOfPoints):
				for j in range(par.NumberOfPoints):
					if(par.ObstacleRegionWithClearence[j][i] == 0):
						diagram4.weights.update({(i,j): par.UniversalCost})
					
			goal_pos = (target.position.x/par.RealScale,target.position.y/par.RealScale)
			start_pos = (cur_pose.pose.position.x/par.RealScale,cur_pose.pose.position.y/par.RealScale)
			pathx,pathy=Astar_version1(par,start_pos,goal_pos,diagram4)
			print pathx
			print pathy
			vController.setHeadingTarget(deg2rad(90.0)) #45.0))
			target.position.x = pathx[0]*par.RealScale
			target.position.y = pathy[0]*par.RealScale
			uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
			uav_goal_marker_publisher.publish(uav_goal_marker)
			vController.setTarget(target)
			
			vController.setPIDGainX(1,0.0,0.0)
			vController.setPIDGainY(1,0.0,0.0)
			vController.setPIDGainZ(1,0,0)
			vController.setPIDGainPHI(1,0.0,0.0)
			vController.setPIDGainTHETA(1,0.0,0.0)
			vController.setPIDGainPSI(1,0.0,0.0)

		if(usingpositioncontrol):
			print "kontrol posisi"
			poses.pose.position.x = 29
			poses.pose.position.y = 29
			poses.pose.position.z = 10
			local_pos_pub.publish(poses)
		elif(usingvelocitycontrol):
			if(distance2D(cur_pose.pose,target) < 1.5):
				if(len(pathx) > 1):
					del pathx[0]
					del pathy[0]
					target.position.x = pathx[0]*par.RealScale
					target.position.y = pathy[0]*par.RealScale
					uav_goal_marker=update_uav_marker(uav_goal_marker,(target.position.x,target.position.y,target.position.z,1.0))
					uav_goal_marker_publisher.publish(uav_goal_marker)
					print target
					vController.setTarget(target)

			if(rospy.Time.now()-last_HRVO_request > rospy.Duration(0.05)):
				last_HRVO_request=rospy.Time.now()
				header.stamp = rospy.Time.now()
				
				des_vel = vController.update(cur_pose)
				current_agent_pose = (list_current_position[idx].pose.position.x,list_current_position[idx].pose.position.y,list_current_position[idx].pose.position.z)
				current_agent_vel = (list_current_velocity[idx].twist.linear.x,list_current_velocity[idx].twist.linear.y)
				current_neighbor_pose = []
				for i in range(par.NumberOfAgents):
					if(i != idx):
						current_neighbor_pose.append((list_current_position[i].pose.position.x,list_current_position[i].pose.position.y))
				current_neighbor_vel = []
				for i in range(par.NumberOfAgents):
					if(i != idx):
						current_neighbor_vel.append((list_current_velocity[i].twist.linear.x,list_current_velocity[i].twist.linear.y))
				V_des = (des_vel.twist.linear.x,des_vel.twist.linear.y)
				quat_arr = np.array([list_current_position[idx].pose.orientation.x,list_current_position[idx].pose.orientation.y,list_current_position[idx].pose.orientation.z,list_current_position[idx].pose.orientation.w])
				att = euler_from_quaternion(quat_arr,'sxyz')
				#V = locplan.RVO_update_single(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par,list_velocity_angle,list_distance_obs,att[2])
				V,RVO_BA_all = locplan.RVO_update_single_static(current_agent_pose, current_neighbor_pose, current_agent_vel, current_neighbor_vel, V_des, par)
	        
				V_msg = des_vel
				V_msg.twist.linear.x = V[0]
				V_msg.twist.linear.y = V[1]
				local_vel_pub.publish(V_msg)
				
				goal = (target.position.x,target.position.y,target.position.z)
				x = current_position
				list_error[0].append(error_time)
				list_error[1].append(goal[0]-x[0])
				list_error[2].append(goal[1]-x[1])
				list_error[3].append(goal[2]-x[2])
				error_time = error_time + 1
				k=0.1
				vx = k*(goal[0]-x[0])
				vy = k*(goal[1]-x[1])
				vz = k*(goal[2]-x[2])
				postar = PositionTarget()
				postar.header = header
				postar.coordinate_frame = PositionTarget().FRAME_LOCAL_NED
				p = PositionTarget().IGNORE_PX | PositionTarget().IGNORE_PY | PositionTarget().IGNORE_PZ 
				a = PositionTarget().IGNORE_AFX | PositionTarget().IGNORE_AFY | PositionTarget().IGNORE_AFZ	
				v = PositionTarget().IGNORE_VX | PositionTarget().IGNORE_VY | PositionTarget().IGNORE_VZ	
				y = PositionTarget().IGNORE_YAW | PositionTarget().IGNORE_YAW_RATE
				f = PositionTarget().FORCE
				postar.type_mask = a | y | p | f #| PositionTarget().IGNORE_YAW | PositionTarget().IGNORE_YAW_RATE | PositionTarget().FORCE
				postar.velocity.x = vx
				postar.velocity.y = vy
				postar.velocity.z = vz
				postar.position.x = goal[0]
				postar.position.y = goal[1]
				postar.position.z = goal[2]
				
				vel_msg = TwistStamped()
				vel_msg.header = header
				vel_msg.twist.linear.x = xvel
				vel_msg.twist.linear.y = yvel
				vel_msg.twist.linear.z = 0.0
				vel_msg.twist.angular.x = 0.0
				vel_msg.twist.angular.y = 0.0
				vel_msg.twist.angular.z = 0.0
				
				q=quaternion_from_euler(0,0,0.2)
				att = PoseStamped()
				att.pose.orientation.x = q[0]
				att.pose.orientation.y = q[1]
				att.pose.orientation.z = q[2]
				att.pose.orientation.w = q[3]
				att.pose.position.x = 0.0
				att.pose.position.y = 0.0
				att.pose.position.z = 2.0

				cmd_thr = Float64()
				cmd_thr.data = 0.3

				att_raw = AttitudeTarget()
				att_raw.header = Header()
				att_raw.body_rate = Vector3()

				att_raw.thrust = 0.7 #Float64()
				att_raw.header.stamp = rospy.Time.now()
				att_raw.header.frame_id = "fcu"
				att_raw.type_mask = 71
				att_raw.orientation = Quaternion(*quaternion_from_euler(0,0,0.2))

		else:
			local_pos_pub.publish(poses)
		count=count+1
		r.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)