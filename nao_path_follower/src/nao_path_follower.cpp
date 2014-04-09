/*
 *
 * Copyright 2011-2014 Armin Hornung, Stefan Osswald, Daniel Maier
 * University of Freiburg
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nao_msgs/FollowPathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>
#include <assert.h>

class PathFollower
{
public:
	PathFollower();
	~PathFollower();
	void goalActionCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
	void pathActionCB(const nao_msgs::FollowPathGoalConstPtr &goal);
	void goalCB(const geometry_msgs::PoseStampedConstPtr& goal);
	void pathCB(const nav_msgs::PathConstPtr& goal);
        bool getRobotPose( tf::StampedTransform & globalToBase, const std::string & global_frame_id);

private:
        bool getNextTarget(const nav_msgs::Path & path, const tf::Pose & robotPose,  const std::vector<geometry_msgs::PoseStamped>::const_iterator & currentPathPoseIt, tf::Stamped<tf::Transform> & targetPose, std::vector< geometry_msgs::PoseStamped>::const_iterator & targetPathPoseIt);
	void stopWalk();
	void setVelocity(const tf::Transform& relTarget);
	void publishTargetPoseVis(const tf::Stamped<tf::Pose>& targetPose);
	void footContactCallback(const std_msgs::BoolConstPtr& contact);

	ros::NodeHandle nh;
	ros::NodeHandle privateNh;
	ros::Subscriber m_simpleGoalSub, m_simplePathSub;
	ros::Publisher m_targetPub, m_velPub, m_actionGoalPub, m_visPub, m_actionPathPub;
	tf::TransformListener m_tfListener;
	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> m_walkGoalServer;
	actionlib::SimpleActionServer<nao_msgs::FollowPathAction> m_walkPathServer;
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
        // for path following
	double m_pathNextTargetDistance;     ///< Cumulative Distance to next Velocity Target from current robot pose along path
};

PathFollower::PathFollower()
  : privateNh("~"),
    m_walkGoalServer(nh, "walk_target", boost::bind(&PathFollower::goalActionCB, this, _1), false),
    m_walkPathServer(nh, "walk_path", boost::bind(&PathFollower::pathActionCB, this, _1), false),
    m_baseFrameId("base_footprint"), m_controllerFreq(10),
    m_targetDistThres(0.05), m_targetAngThres(angles::from_degrees(5.)),
    m_isJoystickInhibited(false), 
    m_useFootContactProtection(true), m_footContact(true),
    m_useVelocityController(false),
    m_maxVelFractionX(0.7), m_maxVelFractionY(0.7), m_maxVelFractionYaw(0.7), m_stepFreq(0.5),
    m_maxVelXY(0.0952), m_maxVelYaw(angles::from_degrees(47.6)), m_minStepFreq(1.667), m_maxStepFreq(2.381),    
    m_thresholdFar(0.20), m_thresholdRotate(angles::from_degrees(45.)),
    m_thresholdDampXY(0.20), m_thresholdDampYaw(angles::from_degrees(30.)), m_pathNextTargetDistance(0.1)
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
	privateNh.param("path_next_target_distance", m_pathNextTargetDistance, m_pathNextTargetDistance);
	privateNh.param("threshold_damp_xy", m_thresholdDampXY, m_thresholdDampXY);

	m_stepFactor = ((m_maxStepFreq - m_minStepFreq) * m_stepFreq + m_minStepFreq) / m_maxStepFreq;
        ROS_INFO("Using step factor of %4.2f",m_stepFactor);

	if(m_useVelocityController) {
	    ROS_INFO("Using velocity controller");
	    m_velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	} else {
	    ROS_INFO("Using target pose controller");
	    m_targetPub = nh.advertise<geometry_msgs::Pose2D>("cmd_pose", 10);
	}

	m_actionGoalPub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("walk_target/goal", 1);
	m_actionPathPub = nh.advertise<nao_msgs::FollowPathActionGoal>("walk_path/goal", 1);
	//we'll provide a mechanism for some people to send goals or paths as PoseStamped messages over a topic
	//they won't get any useful information back about its status, but this is useful for tools
	//like nav_view and rviz
	m_simpleGoalSub = nh.subscribe<geometry_msgs::PoseStamped>("walk_target_simple/goal", 1, boost::bind(&PathFollower::goalCB, this, _1));
	m_simplePathSub = nh.subscribe<nav_msgs::Path>("walk_path_simple/goal", 1, boost::bind(&PathFollower::pathCB, this, _1));
	
	if(m_useFootContactProtection) {
	    m_footContactSub = nh.subscribe<std_msgs::Bool>("foot_contact", 1, &PathFollower::footContactCallback, this);
	}

	m_inhibitWalkClient = nh.serviceClient<std_srvs::Empty>("inhibit_walk");
	m_uninhibitWalkClient = nh.serviceClient<std_srvs::Empty>("uninhibit_walk");

	m_visPub = privateNh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true);

	// start up action server now:
	m_walkGoalServer.start();
	m_walkPathServer.start();
}

PathFollower::~PathFollower(){
   std::cerr << "Quitting PathFollower.." << std::endl;
   // TODO: stopWalk is not completed if ctrl+c is hit FIX THIS!
   stopWalk();
   std::cout << ".. stopWalk() called before quitting" << std::endl;

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

void PathFollower::pathCB(const nav_msgs::PathConstPtr& goal) {
   ROS_INFO("Simple Path callback");
	ROS_DEBUG("In ROS path callback, wrapping the Path in the action message and re-sending to the server.");
        nao_msgs::FollowPathActionGoal action_goal;
        action_goal.header = goal->header;
        action_goal.goal.path = *goal;
	m_actionPathPub.publish(action_goal);
}

void PathFollower::footContactCallback(const std_msgs::BoolConstPtr& contact) {
    m_footContact = contact->data;
}


bool hasOrientation(const geometry_msgs::Pose & pose)
{
   double eps = 1e-5;
   if ( fabs(pose.orientation.x) < eps && fabs(pose.orientation.y) < eps && fabs(pose.orientation.z) < eps && fabs(pose.orientation.w) < eps )
      return false;
   geometry_msgs::Quaternion q = pose.orientation;
   if ( fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w -1 ) > eps)
         return false;
   return true;
         
}

bool PathFollower::getRobotPose( tf::StampedTransform & globalToBase, const std::string & global_frame_id)
{
   try
   {
      // this causes a segfault when shutting down, move_base also doesn't have it?
      //			m_tfListener.waitForTransform(targetPose.frame_id_, m_baseFrameId, ros::Time(), ros::Duration(0.1));
      m_tfListener.lookupTransform(global_frame_id, m_baseFrameId, ros::Time(), globalToBase);
      //ROS_INFO("ros::Time() is %f, ros::Time::now() is %f, delta is %f", globalToBase.stamp_.toSec(), ros::Time::now().toSec(), ros::Duration(ros::Time::now() - globalToBase.stamp_).toSec());
   }
   catch(const tf::TransformException& e)
   {
      ROS_WARN("Failed to obtain tf to base (%s)", e.what());
      return false;
   }
   //make sure base footprint is on the floor:
   globalToBase.getOrigin().setZ(0.0);
   double roll, pitch, yaw;
   globalToBase.getBasis().getRPY(roll, pitch, yaw);
   globalToBase.setRotation(tf::createQuaternionFromYaw(yaw));
   return true;
}

double distance (const tf::Pose & p1, const tf::Pose & p2)
{
   return (p1.getOrigin() - p2.getOrigin()).length();
}

void print_transform(const tf::Transform & t)
{
   geometry_msgs::Pose m;
   tf::poseTFToMsg(t, m);
   std::cout << m;
}
double getYawBetween(const tf::Transform & p1, const tf::Transform & p2)
{
   tf::Vector3 o1 = p1.getOrigin();
   tf::Vector3 o2 = p2.getOrigin();
   return  std::atan2( o2.getY() - o1.getY(), o2.getX() - o1.getX());
}
tf::Quaternion getOrientationBetween(const tf::Transform & p1, const tf::Transform & p2)
{
   /*
   tf::Quaternion q;
   tf::Transform t1 = p1;
   t1.setRotation(tf::createQuaternionFromYaw(0.0));
   tf::Transform t2 = p2;
   t2.setRotation(tf::createQuaternionFromYaw(0.0));
   print_transform(t1);
   print_transform(t2);
   std::cout << std::endl;

   tf::Transform relTarget = t1.inverseTimes(t2);
   print_transform(relTarget);
   std::cout << std::endl;
   relTarget.getBasis().getRPY(roll, pitch, yaw);
   */
   //ROS_INFO("Computed yaw %f ",  yaw);
   double yaw = getYawBetween(p1, p2);
   return (tf::createQuaternionFromYaw(yaw));
}


typedef std::vector< geometry_msgs::PoseStamped>::const_iterator PathIterator;

double moveAlongPathByDistance( const PathIterator & start, PathIterator & end, double targetDistance)
{
   PathIterator iter = start;
   tf::Stamped<tf::Transform> tfPose, tfPose2;
   double dist = 0.0;
   while(iter+1 != end )
   {
      tf::poseStampedMsgToTF(*(iter), tfPose);
      tf::poseStampedMsgToTF(*(iter+1), tfPose2);
      double d = distance( tfPose, tfPose2);
      if (dist + d > targetDistance )
         break;
      dist += d;
      ++iter;
   }
   end = iter;
   return dist;
}

// this function assumes that robotPose is close (< robot_start_path_threshold) to *currentPathPoseIt 
bool PathFollower::getNextTarget(const nav_msgs::Path & path, const tf::Pose & robotPose,  const std::vector<geometry_msgs::PoseStamped>::const_iterator & currentPathPoseIt, tf::Stamped<tf::Transform> & targetPose, std::vector< geometry_msgs::PoseStamped>::const_iterator & targetPathPoseIt)
{  

   // current state in  path unclear: cannot go further, end of path reached
   if( path.poses.empty() || currentPathPoseIt == path.poses.end() ) // past-the-end of path
   {
      ROS_ERROR("Path does not have successors for currentPathPoseIt (i.e. currentPathPoseIt pointing to past-the-end or is empty)");
      return true;
      // TODO: Throw something
   }

   
   // 0 successors possible from currentPathPoseIt
   if( currentPathPoseIt+1 == path.poses.end() )
   {
      // this can happen, if we are close to the final goal or someone send us a single goal wrapped as path (not allowed)
      // under first assumption it we just keep the current targetPose
      ROS_INFO("Goal almost reached! Keeping targetPose. ");
      return true;
   }
   // from here, currentPathPoseIt has at least one successor

   // rewrite
   std::vector< geometry_msgs::PoseStamped>::const_iterator iter = currentPathPoseIt+1;
   tf::Stamped<tf::Transform> tfPose, tfPose2;
   tf::poseStampedMsgToTF( *iter, tfPose);
   double dist = distance( robotPose, tfPose);
   bool targetIsEndOfPath = true;
   double targetDistance = m_pathNextTargetDistance; // target at most x cm ahead of robot
   while(iter+1 != path.poses.end() )
   {
      tf::poseStampedMsgToTF(*iter, tfPose);
      tf::poseStampedMsgToTF(*(iter+1), tfPose2);
      double d = distance( tfPose, tfPose2);
      if (dist + d > targetDistance )
      {
         targetIsEndOfPath = false;
         break;
      }
      dist += d;
      ++iter;
   }

   targetPathPoseIt = iter;
   tf::poseStampedMsgToTF(*iter, targetPose);
   

   // check if poses have orientation
   if ( !hasOrientation (iter->pose) )
   {
      //ROS_WARN("Target has no orientation. Computing orientation from other waypoints.");

      tf::Quaternion orientation;
      if( iter==currentPathPoseIt ) // path has only one element so get orientation from robot to poses[0]
      {
         ROS_ERROR("This case should never happend (anymore)");
         orientation = getOrientationBetween( robotPose, targetPose);
         assert(0); // produce seg fault // TODO: To remove this later
      }
      else if (targetIsEndOfPath) // path length >= 2 and iter points to end of path
      {
         tf::Stamped<tf::Transform> prevPose;
         tf::poseStampedMsgToTF( *(iter-1), prevPose );
         orientation = getOrientationBetween( prevPose, targetPose);
      }
      else // iter has at least one succesor
      {
         // set Orientation so that successor of targetPose are faced
         /*
         ++iter;
         tf::poseStampedMsgToTF( *iter, tfPose );
         dist = distance( targetPose, tfPose );
         while(iter+1 != path.poses.end() )
         {
            tf::poseStampedMsgToTF(*(iter), tfPose);
            tf::poseStampedMsgToTF(*(iter+1), tfPose2);
            double d = distance( tfPose, tfPose2);
            if (dist + d > targetDistance )
               break;
            dist += d;
            ++iter;
         }
         ROS_INFO("Found successor at %4.2f, %4.2f", tfPose.getOrigin().x(), tfPose.getOrigin().y());
         */
         //std::vector< tf::Stamped<tf::Transform> > poses;
         /*
         iter = targetPathPoseIt;
         double cumDist = 0.0;
         double sinSum = 0.0;
         double cosSum = 0.0;
         // first add robot orientation
         double yaw = getYawBetween( robotPose, targetPose); 
         sinSum += std::sin(yaw);
         cosSum += std::cos(yaw);
         ROS_INFO("Found target at %4.2f, %4.2f, %4.2f", targetPose.getOrigin().x(), targetPose.getOrigin().y(), yaw*180.0/M_PI);
         for(int i =0; i < 2 ; ++i )
         {
            PathIterator end = path.poses.end();
            cumDist += moveAlongPathByDistance( iter, end, targetDistance);
            tf::poseStampedMsgToTF(*(iter), tfPose2);
            tf::poseStampedMsgToTF(*(end), tfPose);
            double yaw = getYawBetween( tfPose2, tfPose); 
            sinSum += std::sin(yaw);
            cosSum += std::cos(yaw);
            //poses.push_back(tfPose);
            iter = end;
            ROS_INFO("Found successor at %4.2f, %4.2f, %4.2f", tfPose.getOrigin().x(), tfPose.getOrigin().y(), yaw*180.0/M_PI);
            if( iter+1 == path.poses.end() )
               break;
         }
         yaw = std::atan2( sinSum, cosSum );
         */
         tf::poseStampedMsgToTF(*(targetPathPoseIt+1), tfPose2 );
         double oldyaw = getYawBetween( targetPose, tfPose2);
         double yaw = oldyaw;
         if (targetPathPoseIt-1 != path.poses.begin() && targetPathPoseIt + 2 != path.poses.end() )
         {

            tf::poseStampedMsgToTF(*(targetPathPoseIt-2), tfPose);
            tf::poseStampedMsgToTF(*(targetPathPoseIt+2), tfPose2 );
            double yaw1 = getYawBetween(tfPose, targetPose);
            double yaw2 = getYawBetween(targetPose, tfPose2);
            yaw = atan2( sin(yaw1)+sin(yaw2), cos(yaw1) + cos(yaw2) );
         }
         ROS_INFO("Using yaw %4.2f instead of %4.2f (old method)", yaw*180.0/M_PI, oldyaw*180.0/M_PI);
         orientation = tf::createQuaternionFromYaw(yaw);


      }
      targetPose.setRotation(orientation);

      //ROS_INFO_STREAM("..computed orientation" << basis << std::endl);
   }

   return targetIsEndOfPath;

}


void PathFollower::pathActionCB(const nao_msgs::FollowPathGoalConstPtr &goal){
   ROS_INFO("path action starting...");

   nav_msgs::Path path = goal->path;
   nao_msgs::FollowPathFeedback feedback;
   if( path.poses.empty() )
   {
      ROS_INFO("Stop requested by sending empty path.");
      stopWalk();
      m_walkPathServer.setSucceeded(nao_msgs::FollowPathResult(),"Stop succeeed");
      return;
   }
   if(path.poses.size()==1)
   {
      ROS_INFO("Sending a single goal is not supported (use walk_target instead).");
      stopWalk();
      m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Single goal stop");
      return;
   }
   ros::Rate r(m_controllerFreq);

   ros::Time lastTfSuccessTime = ros::Time::now();
   tf::StampedTransform globalToBase;
   double tfLookupTimeThresh = 1.0; // TODO: param
   std::string global_frame_id = path.header.frame_id;
   while (! getRobotPose(globalToBase, global_frame_id) )
   {
      if ( (ros::Time::now() - lastTfSuccessTime ).toSec() > tfLookupTimeThresh )
      {
         ROS_ERROR("Could not get robot pose. Stopping robot");
         stopWalk();
         m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Aborted");
         return;
      }
      else
      {
         r.sleep();
         continue;
      }
   }
   //lastTfSuccessTime = globalToBase.stamp_;
   lastTfSuccessTime = ros::Time::now();

   // Check if start of path is consistent with current robot pose, otherwise abort
   std::vector< geometry_msgs::PoseStamped>::const_iterator currentPathPoseIt, targetPathPoseIt;
   currentPathPoseIt = path.poses.begin();
   tf::Stamped<tf::Transform> tmp;
   tf::poseStampedMsgToTF( *currentPathPoseIt, tmp);
   double robot_start_path_threshold = 0.05; // TODO: Param
   if (distance(tmp, globalToBase)> robot_start_path_threshold )
   {
      ROS_ERROR("Robot is too far away from start of plan. aborting");
      stopWalk();
      m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Aborted");
      return;
   }


   // This is where Nao is currently walking to (current sub goal)
   tf::Stamped<tf::Transform> targetPose;
   bool targetIsEndOfPath = getNextTarget(path, globalToBase, currentPathPoseIt, targetPose, targetPathPoseIt );
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
   
   while(ros::ok()){
      if (! getRobotPose(globalToBase, global_frame_id) )
      {
         if ( (ros::Time::now() - lastTfSuccessTime ).toSec() > tfLookupTimeThresh )
         {
            ROS_ERROR("Could not get robot pose. Stopping robot");
            stopWalk();
            m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Aborted");
            return;
         }
         else
         {
            r.sleep();
            continue;
         }
      }
      //lastTfSuccessTime = globalToBase.stamp_;
      lastTfSuccessTime = ros::Time::now();
      double dt = (lastTfSuccessTime - globalToBase.stamp_).toSec();
      if ( dt > tfLookupTimeThresh )
      {
         ROS_WARN("TF transforms are behind current clock by %4.2f", dt);
         //TODO do something here..
      }

      if (m_walkPathServer.isPreemptRequested()){
         if(m_walkPathServer.isNewGoalAvailable()){
            ROS_INFO("walk_path ActionServer: new goal available");
            nao_msgs::FollowPathGoal newGoal = *m_walkPathServer.acceptNewGoal();
            path = newGoal.path;
            // Check if path is not empty, otherwise abort
            if( path.poses.empty() )
            {
               ROS_INFO("Stop requested by sending empty path.");
               stopWalk();
               m_walkPathServer.setSucceeded(nao_msgs::FollowPathResult(),"Stop succeeed");
               return;
            }
            /*
            if(path.poses.size()==1)
            {
               ROS_INFO("Sending a single goal is not supported (use walk_target instead).");
               stopWalk();
               m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Single goal stop");
               return;
            }
            */

            // Check if start of path is consistent with current robot pose, otherwise abort
            currentPathPoseIt = path.poses.begin();
            tf::Stamped<tf::Transform> tmp;
            tf::poseStampedMsgToTF( *currentPathPoseIt, tmp);
            if (distance(tmp, globalToBase)> robot_start_path_threshold )
            {
               ROS_ERROR("Robot is too far away from start of plan. aborting");
               stopWalk();
               m_walkPathServer.setAborted(nao_msgs::FollowPathResult(),"Aborted");
               return;
            }

            // set targetPose, this will not change until we reached it
            targetIsEndOfPath = getNextTarget(path, globalToBase, currentPathPoseIt, targetPose, targetPathPoseIt );
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
            ROS_INFO("walk_path ActionServer preempted");
            // anything to cleanup?e)
            stopWalk();

            m_walkPathServer.setPreempted();
            return;
         }
      }

      if(m_useFootContactProtection && !m_footContact) {
         ROS_WARN("Lost foot contact, abort walk");
         stopWalk();
         m_walkPathServer.setAborted(nao_msgs::FollowPathResult(), "Aborting on the goal because the robot lost foot contact with the ground");
         return;
      }


      /*
         move_base_msgs::MoveBaseFeedback feedback;
         feedback.base_position.header.stamp = globalToBase.stamp_;
         feedback.base_position.header.frame_id = globalToBase.frame_id_;
         tf::poseTFToMsg(globalToBase, feedback.base_position.pose);
         m_walkGoalServer.publishFeedback(feedback);
         */


      double roll, pitch, yaw;
      tf::Transform relTarget = globalToBase.inverseTimes(targetPose);
      relTarget.getBasis().getRPY(roll, pitch, yaw);


      // treat last targetPose different from other waypoints on path
      if (targetIsEndOfPath && relTarget.getOrigin().length()< m_targetDistThres && std::abs(yaw) < m_targetAngThres)
      {
         ROS_INFO("Target (%f %f %F) reached", targetPose.getOrigin().x(), targetPose.getOrigin().y(),
               tf::getYaw(targetPose.getRotation()));

         stopWalk();

         m_walkPathServer.setSucceeded(nao_msgs::FollowPathResult(), "Target reached");
         return;
      }
      // TODO: Use other thresholds for target != endOfPath
      else if ( relTarget.getOrigin().length()< robot_start_path_threshold && std::abs(yaw) < 45*M_PI/180.0)
      {
         // update currentPathPoseIt to point to waypoint corresponding to  targetPose
         currentPathPoseIt = targetPathPoseIt;
         // update targetPose, targetPathPoseIt
         targetIsEndOfPath = getNextTarget(path, globalToBase, currentPathPoseIt, targetPose, targetPathPoseIt );
         publishTargetPoseVis(targetPose);
         // update relTarget
         relTarget = globalToBase.inverseTimes(targetPose);
      }

      // walk to targetPose (relTarget)
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


      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / m_controllerFreq))
         ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", m_controllerFreq, r.cycleTime().toSec());

   }

   //if the node is killed then we'll abort and return

   // TODO: stopWalk is not completed if ctrl+c is hit FIX THIS!
   stopWalk();
   m_walkPathServer.setAborted(nao_msgs::FollowPathResult(), "Aborting on the goal because the node has been killed");



   std::cout << ("... path action ending") << std::endl;
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

            m_walkGoalServer.setSucceeded(move_base_msgs::MoveBaseResult(), "Target reached");
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
           if(m_useVelocityController) {
              ROS_WARN("Call to stop_walk_srv Service failed, falling back to velocity=0,0,0");
              geometry_msgs::Twist twist;
              twist.linear.x = 0.0;
              twist.linear.y = 0.0;
              twist.angular.z = 0.0;
              m_velPub.publish(twist);
           }
           else
           {
              ROS_WARN("Call to stop_walk_srv Service failed, falling back to target=0,0,0");
              geometry_msgs::Pose2D target;
              target.x = 0.0;
              target.y = 0.0;
              target.theta = 0.0;
              m_targetPub.publish(target);
           }
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


