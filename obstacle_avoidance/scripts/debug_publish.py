#!/usr/bin/env python
import argparse
import roslib
import rospy
import sys
import numpy as numpy
from std_msgs.msg import Bool,Float32MultiArray,MultiArrayDimension

list_boolean = []
curr_boolean = False
def boolean_cb(msg_var,data_var):
	
	curr_boolean=msg_var.data
	print "data_var" ,data_var,msg_var.data

def main(args):
	rospy.init_node("float32ma")
	pub = rospy.Publisher("/float32ma",Float32MultiArray,queue_size=10)
	i = 0.0
	j = 0.0
	dat = Float32MultiArray()
  	dat.layout.dim.append(MultiArrayDimension())
  	dat.layout.dim[0].label = "length"
  	dat.layout.dim[0].size = 2
  	dat.layout.dim[0].stride = 2
  	dat.layout.data_offset = 0
  	dat.data = [i,j]
	#sub = rospy.Subscriber("/bool"+args[1],Bool,queue_size=10,callback=boolean_cb,callback_args=args[1])
	#sub = rospy.Subscriber("/bool"+repr(3),Bool,queue_size=10,callback=boolean_cb,callback_args=repr(3))
	#sub = rospy.Subscriber("/bool"+repr(4),Bool,queue_size=10,callback=boolean_cb,callback_args=repr(4))
	while not rospy.is_shutdown():
		pub.publish(dat)
		#print curr_boolean
		rospy.sleep(1)
		i=i+1
		j=j-2
		dat.data = [i,j]
		#pub.publish(False)
		#print curr_boolean
		#rospy.sleep(1)

	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
  	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)