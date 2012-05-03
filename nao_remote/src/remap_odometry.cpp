// SVN $HeadURL$
// SVN $Id$

/*
 * Nao Odometry Remapping in ROS (based on odometry_publisher_tutorial)
 *
 * Copyright 2009-2011 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/nao
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <nao_msgs/TorsoOdometry.h>
#include <nao_msgs/TorsoIMU.h>
#include <nao_remote/SetTransform.h>
#include <cmath>

class OdometryRemap
{
public:
	OdometryRemap();
	~OdometryRemap();
	bool pauseOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool resumeOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool odomOffsetCallback(nao_remote::SetTransform::Request& req, nao_remote::SetTransform::Response& res);
	bool setOdomPoseCallback(nao_remote::SetTransform::Request& req, nao_remote::SetTransform::Response& res);


private:
	void jsCallback(const sensor_msgs::JointState::ConstPtr & msg);
	void torsoOdomCallback(const nao_msgs::TorsoOdometryConstPtr& odom, const nao_msgs::TorsoIMUConstPtr& imu);

	ros::Publisher m_odomPub;
	tf::TransformBroadcaster m_transformBroadcaster;
	message_filters::Subscriber<nao_msgs::TorsoOdometry>* m_torsoOdomSub;
	message_filters::Subscriber<nao_msgs::TorsoIMU>* m_torsoIMUSub;
	message_filters::TimeSynchronizer<nao_msgs::TorsoOdometry, nao_msgs::TorsoIMU>* m_synchronizer;
	ros::ServiceServer m_pauseOdomSrv;
	ros::ServiceServer m_resumeOdomSrv;
	ros::ServiceServer m_odomOffsetSrv;
	ros::ServiceServer m_setOdomPoseSrv;
	ros::NodeHandle m_nh;
	ros::NodeHandle m_privateNh;

	//for base_footprint_frame: broadcasts frame between right and left foot, coinciding with the ground
	message_filters::Subscriber<sensor_msgs::JointState> * m_jsSub;
	tf::MessageFilter<sensor_msgs::JointState> * m_jsFilter;
	tf::TransformBroadcaster m_brBaseFootPrint;
	tf::TransformListener m_listener;

	geometry_msgs::TransformStamped m_odomTransformMsg;
	tf::Pose m_odomPose; // current "real" odometry pose in original (Nao) odom frame
	tf::Transform m_odomOffset; // offset on odometry origin
	nav_msgs::Odometry m_odom;

	std::string m_odomFrameId;
	std::string m_baseFrameId;
	std::string m_lfootFrameId;
	std::string m_rfootFrameId;
	std::string m_imuTopic;
	std::string m_baseFootPrintID;
	bool m_useIMUAngles;
	bool m_paused;
	double m_lastOdomTime;

	double m_lastWx;
	double m_lastWy;
	double m_lastWz;

	tf::Pose m_targetPose;
	bool m_mustUpdateOffset;
	bool m_initializeFromIMU;
	bool m_initializeFromOdometry;
	bool m_isInitialized;
};

OdometryRemap::OdometryRemap()
  : m_privateNh("~"), m_odomFrameId("odom"), m_baseFrameId("base_link"),
    m_lfootFrameId("l_sole"), m_rfootFrameId("r_sole"),
    m_imuTopic("torso_imu"), m_baseFootPrintID("base_footprint"),
    m_useIMUAngles(false), m_paused(false),
    m_lastOdomTime(0.0), m_lastWx(0.0), m_lastWy(0.0), m_lastWz(0.0),
    m_mustUpdateOffset(false), m_initializeFromIMU(false),
    m_initializeFromOdometry(false), m_isInitialized(false)
{
    // Read parameters
	m_privateNh.param("odom_frame_id", m_odomFrameId, m_odomFrameId);
	m_privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	m_privateNh.param("use_imu_angles", m_useIMUAngles, m_useIMUAngles);
	m_privateNh.param("init_from_imu", m_initializeFromIMU, m_initializeFromIMU);
	m_privateNh.param("init_from_odometry", m_initializeFromOdometry, m_initializeFromOdometry);
	m_privateNh.param("imu_topic", m_imuTopic, m_imuTopic);

	// Resolve TF frames using ~tf_prefix parameter
	m_odomFrameId = m_listener.resolve(m_odomFrameId);
	m_baseFrameId = m_listener.resolve(m_baseFrameId);
	m_baseFootPrintID = m_listener.resolve(m_baseFootPrintID);
	m_lfootFrameId = m_listener.resolve(m_lfootFrameId);
	m_rfootFrameId = m_listener.resolve(m_rfootFrameId);

	m_odomPub = m_nh.advertise<nav_msgs::Odometry>("odom", 50);

	m_pauseOdomSrv = m_nh.advertiseService("pause_odometry", &OdometryRemap::pauseOdomCallback, this);
	m_resumeOdomSrv = m_nh.advertiseService("resume_odometry", &OdometryRemap::resumeOdomCallback, this);
	m_odomOffsetSrv = m_nh.advertiseService("odometry_offset", &OdometryRemap::odomOffsetCallback, this);
	m_setOdomPoseSrv = m_nh.advertiseService("set_odometry_pose", &OdometryRemap::setOdomPoseCallback, this);

	m_torsoOdomSub = new message_filters::Subscriber<nao_msgs::TorsoOdometry>(m_nh, "torso_odometry", 10);
	m_torsoIMUSub = new message_filters::Subscriber<nao_msgs::TorsoIMU>(m_nh, m_imuTopic, 10);
	m_synchronizer = new message_filters::TimeSynchronizer<nao_msgs::TorsoOdometry, nao_msgs::TorsoIMU>(*m_torsoOdomSub, *m_torsoIMUSub, 10);
	m_synchronizer->registerCallback(boost::bind(&OdometryRemap::torsoOdomCallback, this, _1, _2));

	m_jsSub = new message_filters::Subscriber<sensor_msgs::JointState>(m_nh, "joint_states", 50);
	m_jsFilter = new tf::MessageFilter<sensor_msgs::JointState>(*m_jsSub, m_listener, m_rfootFrameId, 50);
	std::vector<std::string> frames;
	frames.push_back(m_rfootFrameId);
	frames.push_back(m_odomFrameId);
	m_jsFilter->setTargetFrames(frames);
	m_jsFilter->registerCallback(boost::bind(&OdometryRemap::jsCallback, this, _1));

	if (m_useIMUAngles)
		ROS_INFO("Using IMU for odometry roll & pitch");

	// default values of transforms:
	m_odomTransformMsg.header.frame_id = m_odomFrameId;
	m_odomTransformMsg.child_frame_id = m_baseFrameId;

	m_odomOffset = tf::Transform(tf::createIdentityQuaternion());
	m_odomPose = tf::Transform(tf::createIdentityQuaternion());

	if(m_initializeFromIMU || m_initializeFromOdometry) {
	    m_mustUpdateOffset = true;
	} else {
	    m_mustUpdateOffset = false;
	    m_isInitialized = true;
	}
}

OdometryRemap::~OdometryRemap(){
	delete m_synchronizer;
	delete m_torsoOdomSub;
	delete m_torsoIMUSub;
        delete m_jsFilter;
        delete m_jsSub;
}

void OdometryRemap::torsoOdomCallback(const nao_msgs::TorsoOdometryConstPtr& msgOdom, const nao_msgs::TorsoIMUConstPtr& msgIMU){
	if (m_paused){
		ROS_DEBUG("Skipping odometry callback, paused");
		return;
	}

	double odomTime = (msgOdom->header.stamp).toSec();
	ROS_DEBUG("Received [%f %f %f %f (%f/%f) (%f/%f) %f]", odomTime, msgOdom->x, msgOdom->y, msgOdom->z, msgOdom->wx, msgIMU->angleX, msgOdom->wy, msgIMU->angleY, msgOdom->wz);

	double dt = (odomTime - m_lastOdomTime);


	tf::Quaternion q;
	// roll and pitch from IMU, yaw from odometry:
	if (m_useIMUAngles)
		q.setRPY(msgIMU->angleX, msgIMU->angleY, msgOdom->wz);
	else
		q.setRPY(msgOdom->wx, msgOdom->wy, msgOdom->wz);


	m_odomPose.setOrigin(tf::Vector3(msgOdom->x, msgOdom->y, msgOdom->z));
	m_odomPose.setRotation(q);

	// apply offset transformation:
	tf::Pose transformedPose;

	if(m_mustUpdateOffset) {
        if(!m_isInitialized) {
            if(m_initializeFromIMU) {
                // Initialization from IMU: Take x, y, z, yaw from odometry, roll and pitch from IMU
                m_targetPose.setOrigin(m_odomPose.getOrigin());
                m_targetPose.setRotation(tf::createQuaternionFromRPY(msgIMU->angleX, msgIMU->angleY, msgOdom->wz));
            } else if(m_initializeFromOdometry) {
                m_targetPose.setOrigin(m_odomPose.getOrigin());
                m_targetPose.setRotation(tf::createQuaternionFromRPY(msgOdom->wx, msgOdom->wy, msgOdom->wz));
            }
            m_isInitialized = true;
        } else {
            // Overwrite target pitch and roll angles with IMU data
            const double target_yaw = tf::getYaw(m_targetPose.getRotation());
            if(m_initializeFromIMU) {
                m_targetPose.setRotation(tf::createQuaternionFromRPY(msgIMU->angleX, msgIMU->angleY, target_yaw));
            } else if(m_initializeFromOdometry){
                m_targetPose.setRotation(tf::createQuaternionFromRPY(msgOdom->wx, msgOdom->wy, target_yaw));
            }
        }
 		m_odomOffset = m_targetPose * m_odomPose.inverse();
		transformedPose = m_targetPose;
		m_mustUpdateOffset = false;
	} else {
	    transformedPose = m_odomOffset * m_odomPose;
	}

	// publish the transform over tf first:
	m_odomTransformMsg.header.stamp = msgOdom->header.stamp;
	tf::transformTFToMsg(transformedPose, m_odomTransformMsg.transform);
	m_transformBroadcaster.sendTransform(m_odomTransformMsg);

	//next, publish the actual odometry message:
	m_odom.header.stamp = msgOdom->header.stamp;
	m_odom.header.frame_id = m_odomFrameId;


	//set the velocity first (old values still valid)
	m_odom.twist.twist.linear.x = (msgOdom->x - m_odom.pose.pose.position.x) / dt;
	m_odom.twist.twist.linear.y = (msgOdom->y - m_odom.pose.pose.position.y) / dt;
	m_odom.twist.twist.linear.z = (msgOdom->z - m_odom.pose.pose.position.z) / dt;
	// TODO: calc angular velocity!
//	m_odom.twist.twist.angular.z = vth; ??

	//set the position from the above calculated transform
	m_odom.pose.pose.orientation = m_odomTransformMsg.transform.rotation;
	m_odom.pose.pose.position.x = m_odomTransformMsg.transform.translation.x;
	m_odom.pose.pose.position.y = m_odomTransformMsg.transform.translation.y;
	m_odom.pose.pose.position.z = m_odomTransformMsg.transform.translation.z;

	//publish the message
	m_odomPub.publish(m_odom);

	m_lastOdomTime = odomTime;

	// TODO: not used
//	m_lastWx = msgOdom->wx;
//	m_lastWy = msgOdom->wy;
//	m_lastWz = msgOdom->wz;
}

bool OdometryRemap::pauseOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	if (m_paused){
		ROS_WARN("Odometry pause requested, but is already paused");
		return false;
	} else{
		ROS_INFO("Odometry paused");
		m_paused = true;
		m_targetPose = m_odomOffset * m_odomPose;
		m_mustUpdateOffset = true;
		return true;
	}
}

bool OdometryRemap::resumeOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	if (m_paused){
		ROS_INFO("Odometry resumed");
		m_paused = false;
		return true;
	} else{
		ROS_WARN("Odometry resume requested, but is not paused");
		return false;
	}
}

bool OdometryRemap::odomOffsetCallback(nao_remote::SetTransform::Request& req, nao_remote::SetTransform::Response& res){
	ROS_INFO("New odometry offset received");
	tf::Transform newOffset;
	tf::transformMsgToTF(req.offset, newOffset);

	// add new offset to current (transformed) odom pose:
	if(!m_mustUpdateOffset) {
	    m_mustUpdateOffset = true;
	    m_targetPose = m_odomOffset * m_odomPose * newOffset;
	} else {
	    m_targetPose = m_targetPose * newOffset;
	}

    return true;
}

bool OdometryRemap::setOdomPoseCallback(nao_remote::SetTransform::Request& req, nao_remote::SetTransform::Response& res){
	ROS_INFO("New target for current odometry pose received");
	tf::Transform targetPose;
	tf::transformMsgToTF(req.offset, targetPose);

	m_odomOffset = targetPose * m_odomPose.inverse();

	return true;
}
void OdometryRemap::jsCallback(const sensor_msgs::JointState::ConstPtr & ptr)
{
   ros::Time time = ptr->header.stamp;
   tf::StampedTransform tf_odom_to_base, tf_odom_to_left_foot, tf_odom_to_right_foot;

   try {
      m_listener.lookupTransform(m_odomFrameId, m_lfootFrameId, time, tf_odom_to_left_foot);
      m_listener.lookupTransform(m_odomFrameId, m_rfootFrameId, time, tf_odom_to_right_foot);
      m_listener.lookupTransform(m_odomFrameId, m_baseFrameId,  time, tf_odom_to_base);
   } catch (const tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return ;
   }

   tf::Vector3 new_origin = (tf_odom_to_right_foot.getOrigin() + tf_odom_to_left_foot.getOrigin())/2.0; // middle of both feet
   double height = std::min(tf_odom_to_left_foot.getOrigin().getZ(), tf_odom_to_right_foot.getOrigin().getZ()); // fix to lowest foot
   new_origin.setZ(height);

   // adjust yaw according to torso orientation, all other angles 0 (= in z-plane)
   double roll, pitch, yaw;
   tf_odom_to_base.getBasis().getRPY(roll, pitch, yaw);

   tf::Transform tf_odom_to_footprint(tf::createQuaternionFromYaw(yaw), new_origin);
   tf::Transform tf_base_to_footprint = tf_odom_to_base.inverse() * tf_odom_to_footprint;

   // publish transform with parent m_baseFrameId and new child m_baseFootPrintID
   // i.e. transform from m_baseFrameId to m_baseFootPrintID
   m_brBaseFootPrint.sendTransform(tf::StampedTransform(tf_base_to_footprint, time, m_baseFrameId, m_baseFootPrintID));

   return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "remap_odometry");
  OdometryRemap odomRemap;
  ros::spin();

  return 0;
}


