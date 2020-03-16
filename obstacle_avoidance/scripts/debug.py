#!/usr/bin/env python
import argparse
import roslib
import rospy
import sys
import numpy as numpy
from std_msgs.msg import Bool

list_boolean = []
curr_boolean = False
def boolean_cb(msg_var,data_var):
	
	curr_boolean=msg_var.data
	print "data_var" ,data_var,msg_var.data

def main(args):
	rospy.init_node("bool"+args[2])
	pub = rospy.Publisher("/bool"+args[1],Bool,queue_size=10)
	#sub = rospy.Subscriber("/bool"+args[1],Bool,queue_size=10,callback=boolean_cb,callback_args=args[1])
	sub = rospy.Subscriber("/bool"+repr(3),Bool,queue_size=10,callback=boolean_cb,callback_args=repr(3))
	sub = rospy.Subscriber("/bool"+repr(4),Bool,queue_size=10,callback=boolean_cb,callback_args=repr(4))
	
	while not rospy.is_shutdown():
		pub.publish(True)
		print curr_boolean
		rospy.sleep(1)
		pub.publish(False)
		print curr_boolean
		rospy.sleep(1)

	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
  	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)