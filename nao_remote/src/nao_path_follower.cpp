// SVN $HeadURL$
// SVN $Id$

/*
 *
 * Copyright 2011 Armin Hornung & Stefan Osswald, University of Freiburg
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
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>

class PathFollower
{
public:
	PathFollower();
	~PathFollower();
	void goalActionCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void goalCB(const geometry_msgs::PoseStampedConstPtr& goal);

private:
	void stopWalk();
	void setVelocity(const tf::Transform& relTarget);
	void publishTargetPoseVis(const tf::Stamped<tf::Pose>& targetPose);
	void footContactCallback(const std_msgs::BoolConstPtr& contact);

	ros::NodeHandle nh;
	ros::NodeHandle privateNh;
	ros::Subscriber m_simpleGoalSub;
	ros::Publisher m_targetPub, m_velPub, m_actionGoalPub, m_visPub;
	tf::TransformListener m_tfListener;
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> m_walkGoalServer;
	ros::ServiceClient m_inhibitWalkClient, m_uninhibitWalkClient;
	ros::Subscriber m_footContactSub;

	std::string m_baseFrameId;     ///< Base frame of the robot.
	double m_controllerFreq;       ///< Desired frequency for controller loop.
	double m_targetDistThres;      ///< Maximum linear deviation from target point.
	double m_targetAngThres;       ///< Maximum angular deviation from target orientation.
	bool m_isJoystickInhibited;    ///< True = joystick walk commands are inhibited.
	bool m_useFootContactProtection;  ///< True = abort walk when robot is lifted off the ground
	bool m_footContact;            ///< True = robot currently has foot contact with the ground.
	
	// Parameters for the velocity controller
	bool m_useVelocityController;  ///< True = use simple velocity controller, false = use Aldebaran's walkTo controller.
	double m_maxVelFractionX;      ///< Maximum fraction of forward velocity for velocity controller, between 0.0 and 1.0.
	double m_maxVelFractionY;      ///< Maximum fraction of sideways velocity for velocity controller, between 0.0 and 1.0.
	double m_maxVelFractionYaw;    ///< Maximum fraction of rotational velocity for velocity controller, between 0.0 and 1.0.
	double m_stepFreq;             ///< Step frequency (should be set to the same value as in nao_walker).
	double m_stepFactor;           ///< Fraction of the maximum step frequency.
	const double m_maxVelXY;       ///< Maximum translational velocity (0.0952 m/s according to Nao documentation).
	const double m_maxVelYaw;      ///< Maximum rotational velocity (47.6 deg/s according to Nao documentation).
	const double m_minStepFreq;    ///< Minimum step frequency (1.67 Hz according to Nao documentation).
	const double m_maxStepFreq;    ///< Maximum step frequency (2.38 Hz according to Nao documentation).
	double m_thresholdFar;         ///< @see PathFollower::setVelocity()
	double m_thresholdRotate;      ///< @see PathFollower::setVelocity()
	double m_thresholdDampXY;      ///< Slow down when distance to goal is smaller than this value [in meters].
	double m_thresholdDampYaw;     ///< Slow down rotation when angular deviation from goal pose is smaller than this value [in radians].
};

PathFollower::PathFollower()
  : privateNh("~"),
    m_walkGoalServer(nh, "walk_target", boost::bind(&PathFollower::goalActionCB, this, _1), false),
    m_baseFrameId("base_footprint"), m_controllerFreq(10),
    m_targetDistThres(0.05), m_targetAngThres(angles::from_degrees(5.)),
    m_isJoystickInhibited(false), 
    m_useFootContactProtection(true), m_footContact(true),
    m_useVelocityController(false),
    m_maxVelFractionX(0.7), m_maxVelFractionY(0.7), m_maxVelFractionYaw(0.7), m_stepFreq(0.5),
    m_maxVelXY(0.0952), m_maxVelYaw(angles::from_degrees(47.6)), m_minStepFreq(1.67), m_maxStepFreq(2.38),    
    m_thresholdFar(0.20), m_thresholdRotate(angles::from_degrees(45.)),
    m_thresholdDampXY(0.20), m_thresholdDampYaw(angles::from_degrees(30.))
{

	privateNh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	m_baseFrameId = m_tfListener.resolve(m_baseFrameId);
	privateNh.param("controller_frequency", m_controllerFreq, m_controllerFreq);
	privateNh.param("target_distance_threshold", m_targetDistThres, m_targetDistThres);
	privateNh.param("target_angle_threshold", m_targetAngThres, m_targetAngThres);
	privateNh.param("max_vel_x", m_maxVelFractionX, m_maxVelFractionX);
	privateNh.param("max_vel_y", m_maxVelFractionY, m_maxVelFractionY);
	privateNh.param("max_vel_yaw", m_maxVelFractionYaw, m_maxVelFractionYaw);
	privateNh.param("step_freq", m_stepFreq, m_stepFreq);
	privateNh.param("use_vel_ctrl", m_useVelocityController, m_useVelocityController);
	privateNh.param("use_foot_contact_protection", m_useFootContactProtection, m_useFootContactProtection);
        ROS_INFO("Param controller_frequency: %f",m_controllerFreq);
        ROS_INFO("Param target_distance_threshold: %f",m_targetDistThres);
        ROS_INFO("Param target_angle_threshold: %f",m_targetAngThres);
        ROS_INFO("Param max_vel_x: %f, max_vel_y: %f, max_vel_yaw: %f",m_maxVelFractionX,m_maxVelFractionY,m_maxVelFractionYaw);
        ROS_INFO("Param step_freq: %f",m_stepFreq);
        ROS_INFO("Param use_vel_ctrl: %d",m_useVelocityController);
        ROS_INFO("Param use_foot_contact_protection: %d",m_useFootContactProtection);

	m_stepFactor = ((m_maxStepFreq - m_minStepFreq) * m_stepFreq + m_minStepFreq) / m_maxStepFreq;
        ROS_INFO("Using step factor of %4.2f",m_stepFactor);

	if(m_useVelocityController) {
	    ROS_WARN("Velocity controller is not working yet!");
	    ROS_INFO("Using velocity controller");
	    m_velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	} else {
	    ROS_INFO("Using target pose controller");
	    m_targetPub = nh.advertise<geometry_msgs::Pose2D>("cmd_pose", 10);
	}

	m_actionGoalPub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("walk_target/goal", 1);
	//we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
	//they won't get any useful information back about its status, but this is useful for tools
	//like nav_view and rviz
	m_simpleGoalSub = nh.subscribe<geometry_msgs::PoseStamped>("walk_target_simple/goal", 1, boost::bind(&PathFollower::goalCB, this, _1));
	
	if(m_useFootContactProtection) {
	    m_footContactSub = nh.subscribe<std_msgs::Bool>("foot_contact", 1, &PathFollower::footContactCallback, this);
	}

	m_inhibitWalkClient = nh.serviceClient<std_srvs::Empty>("inhibit_walk");
	m_uninhibitWalkClient = nh.serviceClient<std_srvs::Empty>("uninhibit_walk");

	m_visPub = privateNh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true);

	// start up action server now:
	m_walkGoalServer.start();
}

PathFollower::~PathFollower(){

}

/**
 * @brief Publishes the target pose as a geometry_msgs/Pose for visualization.
 * @param targetPose Target pose to publish.
 */
void PathFollower::publishTargetPoseVis(const tf::Stamped<tf::Pose>& targetPose) {
    geometry_msgs::PoseStamped targetPoseMsg;
    tf::poseStampedTFToMsg(targetPose, targetPoseMsg);
    m_visPub.publish(targetPoseMsg);
}

void PathFollower::goalCB(const geometry_msgs::PoseStampedConstPtr& goal) {
	ROS_DEBUG("In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
	move_base_msgs::MoveBaseActionGoal action_goal;
	action_goal.header.stamp = ros::Time::now();
	action_goal.goal.target_pose = *goal;
	m_actionGoalPub.publish(action_goal);
}

void PathFollower::footContactCallback(const std_msgs::BoolConstPtr& contact) {
    m_footContact = contact->data;
}

void PathFollower::goalActionCB(const move_base_msgs::MoveBaseGoalConstPtr &goal){
	tf::Stamped<tf::Transform> targetPose;
	tf::poseStampedMsgToTF(goal->target_pose, targetPose);

	publishTargetPoseVis(targetPose);

	ros::Rate r(m_controllerFreq);

    if(!m_isJoystickInhibited) {
        std_srvs::Empty e;
        if(m_inhibitWalkClient.call(e)) {
            ROS_INFO("Joystick inhibited");
            m_isJoystickInhibited = true;
        } else {
            ROS_WARN("Could not inhibit joystick walk");
        }
    }


	while(nh.ok()){
		if (m_walkGoalServer.isPreemptRequested()){
			if(m_walkGoalServer.isNewGoalAvailable()){
				ROS_INFO("walk_target ActionServer: new goal available");
				move_base_msgs::MoveBaseGoal newGoal = *m_walkGoalServer.acceptNewGoal();
				tf::poseStampedMsgToTF(newGoal.target_pose, targetPose);
				publishTargetPoseVis(targetPose);
				if(!m_isJoystickInhibited) {
				    std_srvs::Empty e;
					if(m_inhibitWalkClient.call(e)) {
					    ROS_INFO("Joystick inhibited");
						m_isJoystickInhibited = true;
					} else {
						ROS_WARN("Could not inhibit joystick walk");
					}
				}
			} else {
				ROS_INFO("walk_target ActionServer preempted");
				// anything to cleanup?e)
				stopWalk();

				m_walkGoalServer.setPreempted();
				return;
			}
		}

		if(m_useFootContactProtection && !m_footContact) {
		    ROS_WARN("Lost foot contact, abort walk");
		    stopWalk();
		    m_walkGoalServer.setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the robot lost foot contact with the ground");
		    return;
		}


		//tf::Stamped<tf::Transform> odomToTarget;
		tf::StampedTransform globalToBase;
		try
		{
			// this causes a segfault when shutting down, move_base also doesn't have it?
//			m_tfListener.waitForTransform(targetPose.frame_id_, m_baseFrameId, ros::Time(), ros::Duration(0.1));
			m_tfListener.lookupTransform(targetPose.frame_id_, m_baseFrameId, ros::Time(), globalToBase);
		}
		catch(const tf::TransformException& e)
		{
			ROS_WARN("Failed to obtain tf to base (%s)", e.what());
			r.sleep();
			continue;
		}
		//make sure base footprint is on the floor:
		globalToBase.getOrigin().setZ(0.0);
		double roll, pitch, yaw;
		globalToBase.getBasis().getRPY(roll, pitch, yaw);
		globalToBase.setRotation(tf::createQuaternionFromYaw(yaw));

		move_base_msgs::MoveBaseFeedback feedback;
		feedback.base_position.header.stamp = globalToBase.stamp_;
		feedback.base_position.header.frame_id = globalToBase.frame_id_;
		tf::poseTFToMsg(globalToBase, feedback.base_position.pose);
		m_walkGoalServer.publishFeedback(feedback);


		tf::Transform relTarget = globalToBase.inverseTimes(targetPose);
		relTarget.getBasis().getRPY(roll, pitch, yaw);

        // target reached:
        if (relTarget.getOrigin().length()< m_targetDistThres && std::abs(yaw) < m_targetAngThres){
            ROS_INFO("Target (%f %f %F) reached", targetPose.getOrigin().x(), targetPose.getOrigin().y(),
                    tf::getYaw(targetPose.getRotation()));

            stopWalk();

            m_walkGoalServer.setSucceeded();
            return;
        } else {
            if(m_useVelocityController) {
                setVelocity(relTarget);
            } else {
                geometry_msgs::Pose2D target;
                // workaround for NaoQI API (stay on the straight line connection facing towards goal)
                if (relTarget.getOrigin().length() > 0.2){
                    relTarget.setOrigin(0.2* relTarget.getOrigin().normalized());
                    yaw = atan2(relTarget.getOrigin().y(), relTarget.getOrigin().x());
                }
                target.x = relTarget.getOrigin().x();
                target.y = relTarget.getOrigin().y();
                target.theta = yaw;

                m_targetPub.publish(target);
            }
        }

	    r.sleep();
	    //make sure to sleep for the remainder of our cycle time
	    if(r.cycleTime() > ros::Duration(1 / m_controllerFreq))
	      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", m_controllerFreq, r.cycleTime().toSec());

	}

	//if the node is killed then we'll abort and return
    stopWalk();
	m_walkGoalServer.setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");

}

/**
 * @brief Stop walk and uninhibit joystick.
 */
void PathFollower::stopWalk(){
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	if (!ros::service::call("stop_walk_srv", req, resp)){
		ROS_WARN("Call to stop_walk_srv Service failed, falling back to target=0,0,0");
		geometry_msgs::Pose2D target;
		target.x = 0.0;
		target.y = 0.0;
		target.theta = 0.0;
		m_targetPub.publish(target);
	}
	if(m_isJoystickInhibited) {
	    std_srvs::Empty e;
		if(m_uninhibitWalkClient.call(e)) {
		    ROS_INFO("Joystick uninhibited");
			m_isJoystickInhibited = false;
		} else {
			ROS_WARN("Could not uninhibit joystick walk");
		}
	}
}

/**
 * @brief Velocity controller.
 * @param relTarget Target pose relative to the current robot pose.
 *
 * Strategy:
 * @verbatim
distance            | angle               | strategy
-----------------------------------------------------------------
> m_thresholdFar    | > m_thresholdRotate | rotate on the spot towards the target
> m_thresholdFar    | < m_thresholdRotate | walk towards the target, orient towards the target
> m_targetDistThres |                     | walk towards the target, orient towards final yaw angle
< m_targetDistThres | > m_targetAngThres  | rotate on the spot towards final yaw angle
< m_targetDistThres | < m_targetAngThres  | stop
@endverbatim
 */

void PathFollower::setVelocity(const tf::Transform& relTarget) {
	double dx, dy, yaw;
	bool rotateOnSpot;
    const double dist = relTarget.getOrigin().length();
	if(dist > m_thresholdFar) {
	    // Far away: Orient towards the target point
	    yaw = atan2(relTarget.getOrigin().y(), relTarget.getOrigin().x());
	    rotateOnSpot = (std::abs(yaw) > m_thresholdRotate);
	} else {
	    // Near: Orient towards the final angle
	    yaw = tf::getYaw(relTarget.getRotation());
        rotateOnSpot = (dist < m_targetDistThres);
        if(dist < m_targetDistThres && std::abs(yaw) < m_targetAngThres) {
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;
            m_velPub.publish(twist);
            return;
        }
	}

	// Normalize angle between -180° and +180°
	yaw = angles::normalize_angle(yaw);

	// Reduce velocity near target
	const double dampXY = std::min(dist / m_thresholdDampXY, 1.0);
	const double dampYaw = std::min(std::abs(yaw) / m_thresholdDampYaw, 1.0);

	if(rotateOnSpot) {
	    dx = 0.0;
	    dy = 0.0;
	} else {
	    dx = relTarget.getOrigin().x();
	    dy = relTarget.getOrigin().y();
	}

    const double times[] = {
        std::abs(dx)  / ((dampXY * m_maxVelFractionX)    * m_maxVelXY  * m_stepFactor),
        std::abs(dy)  / ((dampXY * m_maxVelFractionY)    * m_maxVelXY  * m_stepFactor),
        std::abs(yaw) / ((dampYaw * m_maxVelFractionYaw) * m_maxVelYaw * m_stepFactor),
        1. / m_controllerFreq
    };
    const double maxtime = *std::max_element(times, times + 4);

    geometry_msgs::Twist twist;
    twist.linear.x  = dx  / (maxtime * m_maxVelXY  * m_stepFactor);
    twist.linear.y  = dy  / (maxtime * m_maxVelXY  * m_stepFactor);
    twist.angular.z = yaw / (maxtime * m_maxVelYaw * m_stepFactor);

	ROS_DEBUG("setVelocity: target %f %f %f --> velocity %f %f %f",
	         relTarget.getOrigin().x(), relTarget.getOrigin().y(), yaw,
	         twist.linear.x, twist.linear.y, twist.angular.z);
	m_velPub.publish(twist);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "nao_path_follower");
  PathFollower follower;

  ros::spin();

  return 0;
}


