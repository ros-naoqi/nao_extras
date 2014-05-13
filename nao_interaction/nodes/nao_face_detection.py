#!/usr/bin/env python

#
# ROS node to serve NAO's ALFaceDetectionProxy's functionalities
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2014 Manos Tsardoulias, CERTH/ITI
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
#    # Neither the name of the CERTH/ITI nor the names of its
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

import rospy

import naoqi
from std_msgs.msg import String
from nao_extras_msgs.msg import FaceDetected
from naoqi import ( ALModule, ALBroker, ALProxy )

#
# Notes:
# - A port number > 0 for the module must be specified.
#   If port 0 is used, a free port will be assigned automatically,
#   but naoqi is unable to pick up the assigned port number, leaving
#   the module unable to communicate with naoqi (1.10.52).
# 
# - Callback functions _must_ have a docstring, otherwise they won't get bound.
# 
# - Shutting down the broker manually will result in a deadlock,
#   not shutting down the broker will sometimes result in an exception 
#   when the script ends (1.10.52).
#

class NaoFaceDetection(ALModule):
    "Sends callbacks for NAO's face detection to ROS"
    def __init__(self, moduleName):
        # get connection from command line:
        from optparse import OptionParser

        parser = OptionParser()
        parser.add_option("--ip", dest="ip", default="",
                          help="IP/hostname of broker. Default is system's default IP address.", metavar="IP")
        parser.add_option("--port", dest="port", default=10512,
                          help="IP/hostname of broker. Default is 10511.", metavar="PORT")
        parser.add_option("--pip", dest="pip", default="127.0.0.1",
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_option("--pport", dest="pport", default=9559,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        (options, args) = parser.parse_args()
        self.ip = options.ip
        self.port = int(options.port)
        self.pip = options.pip
        self.pport = int(options.pport)
        self.moduleName = moduleName
        
        self.init_almodule()
        
        # ROS initialization:
        rospy.init_node('nao_face_detection')
             
        # init. messages:
        self.faces = FaceDetected()              
        self.facesPub = rospy.Publisher("faces_detected", FaceDetected)
        
        self.subscribe()
        
        rospy.loginfo("nao_face_detection initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        # TODO: check self.memProxy.version() for > 1.6
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)


    def shutdown(self): 
        self.unsubscribe()
        # Shutting down broker seems to be not necessary any more
        # try:
        #     self.broker.shutdown()
        # except RuntimeError,e:
        #     rospy.logwarn("Could not shut down Python Broker: %s", e)

    def subscribe(self):
        # Subscription to the FaceDetected event
        self.memProxy.subscribeToEvent("FaceDetected", self.moduleName, "onFaceDetected")

    def unsubscribe(self):
        self.memProxy.unsubscribeToEvent("FaceDetected", self.moduleName)

    def onFaceDetected(self, strVarName, value, strMessage):
        "Called when a face was detected"
        
        if len(value) == 0:
          return
        
        # For the specific fields in the value variable check here:
        #https://community.aldebaran-robotics.com/doc/1-14/naoqi/vision/alfacedetection.html
        
        #~ Camera id
        self.faces.camera_id.data = int(value[4]);
        
        self.faces.camera_0_pose.position.x = float(value[2][0])
        self.faces.camera_0_pose.position.y = float(value[2][1])
        self.faces.camera_0_pose.position.z = float(value[2][2])
        self.faces.camera_0_pose.orientation.x = float(value[2][3])
        self.faces.camera_0_pose.orientation.y = float(value[2][4])
        self.faces.camera_0_pose.orientation.z = float(value[2][5])
        
        self.faces.camera_1_pose.position.x = float(value[3][0])
        self.faces.camera_1_pose.position.y = float(value[3][1])
        self.faces.camera_1_pose.position.z = float(value[3][2])
        self.faces.camera_1_pose.orientation.x = float(value[3][3])
        self.faces.camera_1_pose.orientation.y = float(value[3][4])
        self.faces.camera_1_pose.orientation.z = float(value[3][5])
        
        #~ value[1][0][0] : Face info
        #~ value[1][0][1] : Extra info
        #~ Shape information
        self.faces.shape_alpha.data = float(value[1][0][0][1])
        self.faces.shape_beta.data = float(value[1][0][0][2])
        self.faces.shape_sizeX.data = float(value[1][0][0][3])
        self.faces.shape_sizeY.data = float(value[1][0][0][4])
        
        self.faces.face_id.data = float(value[1][0][1][0])
        self.faces.score_reco.data = float(value[1][0][1][1])
        self.faces.face_label.data = str(value[1][0][1][2])
        
        self.faces.left_eye_eyeCenter_x.data = float(value[1][0][1][3][0])
        self.faces.left_eye_eyeCenter_y.data = float(value[1][0][1][3][1])
        self.faces.left_eye_noseSideLimit_x.data = float(value[1][0][1][3][2])
        self.faces.left_eye_noseSideLimit_y.data = float(value[1][0][1][3][3])
        self.faces.left_eye_earSideLimit_x.data = float(value[1][0][1][3][4])
        self.faces.left_eye_earSideLimit_y.data = float(value[1][0][1][3][5])
        self.faces.left_eye_topLimit_x.data = float(value[1][0][1][3][6])
        self.faces.left_eye_topLimit_y.data = float(value[1][0][1][3][7])
        self.faces.left_eye_bottomLimit_x.data = float(value[1][0][1][3][8])
        self.faces.left_eye_bottomLimit_y.data = float(value[1][0][1][3][9])
        self.faces.left_eye_midTopEarLimit_x.data = float(value[1][0][1][3][10])
        self.faces.left_eye_midTopEarLimit_y.data = float(value[1][0][1][3][11])
        self.faces.left_eye_midTopNoseLimit_x.data = float(value[1][0][1][3][12])
        self.faces.left_eye_midTopNoseLimit_y.data = float(value[1][0][1][3][13])
        
        self.faces.right_eye_eyeCenter_x.data = float(value[1][0][1][4][0])
        self.faces.right_eye_eyeCenter_y.data = float(value[1][0][1][4][1])
        self.faces.right_eye_noseSideLimit_x.data = float(value[1][0][1][4][2])
        self.faces.right_eye_noseSideLimit_y.data = float(value[1][0][1][4][3])
        self.faces.right_eye_earSideLimit_x.data = float(value[1][0][1][4][4])
        self.faces.right_eye_earSideLimit_y.data = float(value[1][0][1][4][5])
        self.faces.right_eye_topLimit_x.data = float(value[1][0][1][4][6])
        self.faces.right_eye_topLimit_y.data = float(value[1][0][1][4][7])
        self.faces.right_eye_bottomLimit_x.data = float(value[1][0][1][4][8])
        self.faces.right_eye_bottomLimit_y.data = float(value[1][0][1][4][9])
        self.faces.right_eye_midTopEarLimit_x.data = float(value[1][0][1][4][10])
        self.faces.right_eye_midTopEarLimit_y.data = float(value[1][0][1][4][11])
        self.faces.right_eye_midTopNoseLimit_x.data = float(value[1][0][1][4][12])
        self.faces.right_eye_midTopNoseLimit_y.data = float(value[1][0][1][4][13])
        
        self.faces.left_eyebrow_noseSideLimit_x.data = float(value[1][0][1][5][0])
        self.faces.left_eyebrow_noseSideLimit_y.data = float(value[1][0][1][5][1])
        self.faces.left_eyebrow_center_x.data = float(value[1][0][1][5][2])
        self.faces.left_eyebrow_center_y.data = float(value[1][0][1][5][3])
        self.faces.left_eyebrow_earSideLimit_x.data = float(value[1][0][1][5][4])
        self.faces.left_eyebrow_earSideLimit_y.data = float(value[1][0][1][5][5])
        
        self.faces.right_eyebrow_noseSideLimit_x.data = float(value[1][0][1][6][0])
        self.faces.right_eyebrow_noseSideLimit_y.data = float(value[1][0][1][6][1])
        self.faces.right_eyebrow_center_x.data = float(value[1][0][1][6][2])
        self.faces.right_eyebrow_center_y.data = float(value[1][0][1][6][3])
        self.faces.right_eyebrow_earSideLimit_x.data = float(value[1][0][1][6][4])
        self.faces.right_eyebrow_earSideLimit_y.data = float(value[1][0][1][6][5])
        
        self.faces.nose_bottomCenterLimit_x.data = float(value[1][0][1][7][0])
        self.faces.nose_bottomCenterLimit_y.data = float(value[1][0][1][7][1])
        self.faces.nose_bottomLeftLimit_x.data = float(value[1][0][1][7][2])
        self.faces.nose_bottomLeftLimit_y.data = float(value[1][0][1][7][3])
        self.faces.nose_bottomRightLimit_x.data = float(value[1][0][1][7][4])
        self.faces.nose_bottomRightLimit_y.data = float(value[1][0][1][7][5])
        
        self.faces.mouth_leftLimit_x.data = float(value[1][0][1][8][0])
        self.faces.mouth_leftLimit_y.data = float(value[1][0][1][8][1])
        self.faces.mouth_rightLimit_x.data = float(value[1][0][1][8][2])
        self.faces.mouth_rightLimit_y.data = float(value[1][0][1][8][3])
        self.faces.mouth_topLimit_x.data = float(value[1][0][1][8][4])
        self.faces.mouth_topLimit_y.data = float(value[1][0][1][8][5])
        self.faces.mouth_bottomLimit_x.data = float(value[1][0][1][8][6])
        self.faces.mouth_bottomLimit_y.data = float(value[1][0][1][8][7])
        self.faces.mouth_midTopLeftLimit_x.data = float(value[1][0][1][8][8])
        self.faces.mouth_midTopLeftLimit_y.data = float(value[1][0][1][8][9])
        self.faces.mouth_midTopRightLimit_x.data = float(value[1][0][1][8][10])
        self.faces.mouth_midTopRightLimit_y.data = float(value[1][0][1][8][11])
        self.faces.mouth_midBottomRightLimit_x.data = float(value[1][0][1][8][12])
        self.faces.mouth_midBottomRightLimit_y.data = float(value[1][0][1][8][13])
        self.faces.mouth_midBottomLeftLimit_x.data = float(value[1][0][1][8][14])
        self.faces.mouth_midBottomLeftLimit_y.data = float(value[1][0][1][8][15])

        self.facesPub.publish(self.faces)

if __name__ == '__main__':
    ROSNaoFaceDetectionModule = NaoFaceDetection("ROSNaoFaceDetectionModule")

    rospy.spin()
    
    rospy.loginfo("Stopping nao_face_detection ...")
    ROSNaoFaceDetectionModule.shutdown();        
    rospy.loginfo("nao_face_detection stopped.")
    exit(0)
