#!/usr/bin/env python

#!/usr/bin/env python

# SVN $HeadURL: https://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/scripts/plan_footsteps.py $
# SVN $Id: plan_footsteps.py 1862 2011-08-29 16:29:14Z hornunga@informatik.uni-freiburg.de $

#
# A simple service client calling a running footstep_planner node.
# Using this program you can request footstep plans from the command line.
#
# Author: Armin Hornung, University of Freiburg 
#
# This is program is part of the ROS footstep planner:
# http://www.ros.org/wiki/footstep_planner
# License: GPL 3
#

import roslib
roslib.load_manifest('nao_remote')
import rospy
import sys
import actionlib

from nao_msgs.msg import BodyPoseAction, BodyPoseGoal

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('\nUSAGE: %s pose_name\n\n' %  sys.argv[0])
    
    rospy.init_node('execute_pose')
    
    poseClient = actionlib.SimpleActionClient("body_pose", BodyPoseAction)
    if not poseClient.wait_for_server(rospy.Duration(3.0)):
        rospy.logfatal("Could not connect to required \"body_pose\" action server, is the pose_manager node running?");
        rospy.signal_shutdown();	
    
    goal = BodyPoseGoal
    goal.pose_name = str(sys.argv[1])
    rospy.loginfo("Calling pose_manager for pose %s...", goal.pose_name)
    
    
    poseClient.send_goal_and_wait(goal, rospy.Duration(5.0))
    #TODO: check for errors    
    exit(0)