/*
 * Nao Joystick / Gamepad teleoperation
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/nao_teleop
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


#ifndef NAO_TELEOP_TELEOP_NAO_JOY_
#define NAO_TELEOP_TELEOP_NAO_JOY_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <nao_msgs/JointTrajectoryAction.h>
#include <nao_msgs/BodyPoseAction.h>
#include <nao_msgs/BodyPoseActionGoal.h>
#include <nao_msgs/CmdVelService.h>
#include <actionlib/client/simple_action_client.h>

// switch between diamondback /electric:
#if ROS_VERSION_MINIMUM(1,6,0)
#include <sensor_msgs/Joy.h>
using sensor_msgs::Joy;
#else
#include <joy/Joy.h>
using joy::Joy;
#endif

namespace nao_teleop{
/**
 * \brief Nao Teleoperation via Joystick. Ct
 *
 * Subscribes to "joy" messages, and remaps them to Nao teleoperation commands.
 */
class TeleopNaoJoy
{
public:
  TeleopNaoJoy();
  void pubMsg();
  ros::NodeHandle nh;
  ros::NodeHandle privateNh;

  /**
   * \brief Subscribe to joystick using default hander (TeleopNaoJoy::joyCallback).
   */
  ros::Subscriber subscribeToJoystick();

  /**
   * \brief calls m_bodyPoseClient on the poseName, to execute a body pose
   * @return success of actionlib call
   */
  bool callBodyPoseClient(const std::string& poseName);


protected:
  void joyCallback(const Joy::ConstPtr& joy);
  bool buttonPressed(int button, const Joy::ConstPtr& joy) const;
  bool buttonTriggered(int button, const Joy::ConstPtr& joy) const;
  bool buttonChanged(int button, const Joy::ConstPtr& joy, const Joy::ConstPtr& prevJoy) const;
  void initializePreviousJoystick(const Joy::ConstPtr& joy);
  void setPreviousJoystick(const Joy::ConstPtr& joy) {
    m_previousJoystick = joy;
  }

  bool m_enabled;



  /**
   * \brief Subscribe to joystick using this callback function.
   *
   */
  template<class M, class T>
  ros::Subscriber subscribeToJoystick(void(T::*fp)(M), T* obj) {
    m_joySub = nh.subscribe<Joy>("joy", 3, fp,obj);
    return m_joySub;
  }

  bool inhibitWalk(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool uninhibitWalk(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool axisValid(int axis, const Joy::ConstPtr& joy) const;

  int m_xAxis;
  int m_yAxis;
  int m_turnAxis;
  int m_headYawAxis;
  int m_headPitchAxis;
  int m_crouchBtn;
  int m_initPoseBtn;
  int m_enableBtn;
  int m_modifyHeadBtn;
  int m_startScanBtn;
  int m_stopScanBtn;
  double m_maxVx;
  double m_maxVy;
  double m_maxVw;
  double m_maxHeadYaw;
  double m_maxHeadPitch;
  ros::Duration m_bodyPoseTimeOut;
  int m_inhibitCounter;

  bool m_previousJoystick_initialized;
  Joy::ConstPtr m_previousJoystick;

  ros::Publisher m_movePub;
  ros::Publisher m_moveBtnPub;
  ros::Publisher m_headPub;
  ros::Subscriber m_joySub;
  ros::Publisher m_speechPub;
  ros::ServiceServer m_inhibitWalkSrv;
  ros::ServiceServer m_uninhibitWalkSrv;
  ros::ServiceClient m_cmdVelClient;
  ros::ServiceClient m_stiffnessDisableClient;
  ros::ServiceClient m_stiffnessEnableClient;
  actionlib::SimpleActionClient<nao_msgs::BodyPoseAction> m_bodyPoseClient;
  geometry_msgs::Twist m_motion;
  nao_msgs::JointAnglesWithSpeed m_headAngles;
};
}

#endif
