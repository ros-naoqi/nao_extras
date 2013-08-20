#!/usr/bin/env python
#
# ROS node to from nao's IMU message to a standard sensor_msgs IMU
#
# Copyright 2013 Armin Hornung, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('nao_remote')
import rospy

from sensor_msgs.msg import Imu

from nao_msgs.msg import TorsoIMU
from tf import transformations as tf

pub = None

def handleIMU(data):
  rospy.logdebug("TorsoIMU received for conversion: %f %f", data.angleX, data.angleY)
  imu_msg = Imu()
  q = tf.quaternion_from_euler(data.angleX, data.angleY, 0.0)
  imu_msg.header = data.header
  imu_msg.orientation.x = q[0]
  imu_msg.orientation.y = q[1]
  imu_msg.orientation.z = q[2]
  imu_msg.orientation.w = q[3]
  
  pub.publish(imu_msg)

  

if __name__ == '__main__':

  rospy.init_node('nao_imu_conversion')
  pub = rospy.Publisher('imu', Imu)
  rospy.Subscriber("torso_imu", TorsoIMU, handleIMU, queue_size=50)
  
  
  rospy.spin()
  exit(0)

