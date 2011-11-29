#!/usr/bin/python

# print out Nao odometry data to screen and file "odom.txt"
# e.g. for quick Gnuplot plotting

import roslib
roslib.load_manifest('nao_remote')
import rospy

from nao_msgs.msg import TorsoOdometry
from nav_msgs.msg import Odometry

def handleTorsoOdom(data):
	
	print("%f %f %f %f %f %f" % (data.x, data.y,data.z, data.wx,data.wy, data.wz))
	global file
	file.write("%f %f %f %f %f %f\n" % (data.x, data.y,data.z, data.wx,data.wy, data.wz))
	
	

if __name__ == '__main__':
	
	rospy.init_node('torso2odom')
	odomPub = rospy.Publisher("odometry", Odometry)
	rospy.Subscriber("torso_odometry", TorsoOdometry, handleTorsoOdom)
	
	file = open('odom.txt', 'w')
	
	rospy.spin()

