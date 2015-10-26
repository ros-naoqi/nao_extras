/*
 * Nao Joystick / Gamepad teleoperation
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
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

#include <nao_teleop/teleop_nao_joy.h>
#include <naoqi_bridge_msgs/BodyPoseActionGoal.h>
#include <naoqi_bridge_msgs/CmdVelService.h>
#include <naoqi_bridge_msgs/JointTrajectoryAction.h>

using sensor_msgs::Joy;

namespace nao_teleop{
TeleopNaoJoy::TeleopNaoJoy()
: privateNh("~"), m_enabled(false),
  m_xAxis(3), m_yAxis(2), m_turnAxis(0), m_headYawAxis(4),	m_headPitchAxis(5),
  m_crouchBtn(8), m_initPoseBtn(0), m_enableBtn(9), m_modifyHeadBtn(5),
  m_maxVx(1.0), m_maxVy(1.0), m_maxVw(0.5),
  m_maxHeadYaw(2.0943), m_maxHeadPitch(0.7853),
  m_bodyPoseTimeOut(5.0),
  m_inhibitCounter(0), m_previousJoystick_initialized(false),
  m_bodyPoseClient("body_pose", true),
  m_prevMotionZero(true), m_prevHeadZero(true)
{
  privateNh.param("axis_x", m_xAxis, m_xAxis);
  privateNh.param("axis_y", m_yAxis, m_yAxis);
  privateNh.param("axis_turn", m_turnAxis, m_turnAxis);
  privateNh.param("axis_head_yaw", m_headYawAxis, m_headYawAxis);
  privateNh.param("axis_head_pitch", m_headPitchAxis, m_headPitchAxis);
  privateNh.param("btn_crouch", m_crouchBtn, m_crouchBtn);
  privateNh.param("btn_init_pose", m_initPoseBtn, m_initPoseBtn);
  privateNh.param("btn_enable_control", m_enableBtn, m_enableBtn);
  privateNh.param("btn_head_mod", m_modifyHeadBtn, m_modifyHeadBtn);
  privateNh.param("max_vx", m_maxVx, m_maxVx);
  privateNh.param("max_vy", m_maxVy, m_maxVy);
  privateNh.param("max_vw", m_maxVw, m_maxVw);

  privateNh.param("max_head_yaw", m_maxHeadYaw, m_maxHeadYaw);
  privateNh.param("max_head_pitch", m_maxHeadPitch, m_maxHeadPitch);

  m_motion.linear.x = m_motion.linear.y = m_motion.angular.z = 0.0;
  m_headAngles.joint_names.push_back("HeadYaw");
  m_headAngles.joint_names.push_back("HeadPitch");
  m_headAngles.joint_angles.resize(2, 0.0f);
  m_headAngles.relative = 0;
  m_headAngles.speed = 0.2; // TODO: param

  m_movePub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  m_headPub = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 1);
  m_speechPub = nh.advertise<std_msgs::String>("speech", 1);
  m_inhibitWalkSrv = nh.advertiseService("inhibit_walk", &TeleopNaoJoy::inhibitWalk, this);
  m_uninhibitWalkSrv = nh.advertiseService("uninhibit_walk", &TeleopNaoJoy::uninhibitWalk, this);
  m_cmdVelClient = nh.serviceClient<naoqi_bridge_msgs::CmdVelService>("cmd_vel_srv");
  m_stiffnessDisableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/disable");
  m_stiffnessEnableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/enable");


  if (!m_bodyPoseClient.waitForServer(ros::Duration(3.0))){
    ROS_WARN_STREAM("Could not connect to \"body_pose\" action server, "
        << "there will be no body poses available on button presses.\n"
        << "Is the pose_manager node running?");
  }

  std::cout << "starting button is set to " << m_enableBtn << std::endl;
}


ros::Subscriber TeleopNaoJoy::subscribeToJoystick() {
  return subscribeToJoystick(&TeleopNaoJoy::joyCallback, this);
}

bool TeleopNaoJoy::callBodyPoseClient(const std::string& poseName){
  if (!m_bodyPoseClient.isServerConnected()){
    return false;
  }

  naoqi_bridge_msgs::BodyPoseGoal goal;
  goal.pose_name = poseName;
  m_bodyPoseClient.sendGoalAndWait(goal, m_bodyPoseTimeOut);
  actionlib::SimpleClientGoalState state = m_bodyPoseClient.getState();
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Pose action \"%s\" did not succeed (%s): %s", goal.pose_name.c_str(), state.toString().c_str(), state.text_.c_str());
    return false;
  } else{
    ROS_INFO("Pose action \"%s\" succeeded", goal.pose_name.c_str());
    return true;
  }

}


void TeleopNaoJoy::initializePreviousJoystick(const Joy::ConstPtr& joy) {
  if(!m_previousJoystick_initialized) {
    // if no previous joystick message has been received
    // assume all buttons and axes have been zero
    //
    Joy::Ptr pJoy(new Joy());
    pJoy->buttons.resize( joy->buttons.size(), 0);
    pJoy->axes.resize( joy->axes.size(), 0.0);
    m_previousJoystick = pJoy;
    m_previousJoystick_initialized = true;
  }

}


/**
 * \brief Callback for joystick messages
 *
 * @param joy
 */
void TeleopNaoJoy::joyCallback(const Joy::ConstPtr& joy){
  initializePreviousJoystick(joy);

  // Buttons:
  // TODO: make buttons generally configurable by mapping btn_id => pose_string

  if (m_enabled && buttonTriggered(m_crouchBtn, joy) && m_bodyPoseClient.isServerConnected()){
    if (callBodyPoseClient("crouch")){
      std_srvs::Empty e;
      m_stiffnessDisableClient.call(e);
    }
  }

  if (m_enabled && buttonTriggered(m_initPoseBtn, joy) && m_bodyPoseClient.isServerConnected()){
    callBodyPoseClient("init");
  }

  if (buttonTriggered(m_enableBtn, joy)){
    std_msgs::String string;
    if (m_enabled){
      m_enabled = false;
      string.data = "Gamepad control disabled";

    } else{
      m_enabled = true;
      string.data = "Gamepad control enabled";
      std_srvs::Empty e;
      m_stiffnessEnableClient.call(e);
    }
    m_speechPub.publish(string);
    ROS_INFO("%s", (string.data).c_str());

  }


  // directional commands
  // walking velocities and head movements
  if (!axisValid(m_xAxis, joy) ||  !axisValid(m_yAxis, joy) || !axisValid(m_turnAxis, joy)){
    m_motion.linear.x = m_motion.linear.y = m_motion.angular.z = 0.0f;
    m_headAngles.joint_angles[0] = m_headAngles.joint_angles[1] = 0.0f;
    ROS_WARN("Joystick message too short for Move or Turn axis!\n");
  } else{
    if (buttonPressed(m_modifyHeadBtn, joy)){
      // move head
      m_headAngles.header.stamp = ros::Time::now();
      m_headAngles.relative = 1;
      m_headAngles.joint_angles[0] = joy->axes[m_turnAxis];
      m_headAngles.joint_angles[1] = joy->axes[m_xAxis];

    } else {
      // stop head:
      m_headAngles.joint_angles[0] = m_headAngles.joint_angles[1] = 0.0f;
      // move base:
      m_motion.linear.x = m_maxVx * std::max(std::min(joy->axes[m_xAxis], 1.0f), -1.0f);
      m_motion.linear.y = m_maxVy * std::max(std::min(joy->axes[m_yAxis], 1.0f), -1.0f);
      m_motion.angular.z = m_maxVw * std::max(std::min(joy->axes[m_turnAxis], 1.0f), -1.0f);

    }
  }

  /*
      // Head pos:
      if (!axisValid(m_headPitchAxis, joy) || !axisValid(m_headYawAxis, joy)){
      m_headAngles.absolute = 0;
      m_headAngles.yaw = 0.0;
      m_headAngles.pitch = 0.0;
      } else {
      m_headAngles.absolute = 0;
      m_headAngles.yaw = joy->axes[m_headYawAxis];
      m_headAngles.pitch = joy->axes[m_headPitchAxis];
      }
   */

  setPreviousJoystick(joy);

}

/**
 * \brief Returns true when a valid button is pressed
 *
 * @param button
 * @param joy
 * @return
 */
bool TeleopNaoJoy::buttonPressed(int button, const Joy::ConstPtr& joy) const{
  return (button >= 0 && unsigned(button) < joy->buttons.size() && joy->buttons[button] == 1);
}

/**
 * \brief Returns true when a valid button is triggered
 *
 * @param button
 * @param joy
 * @return
 */
bool TeleopNaoJoy::buttonTriggered(int button, const Joy::ConstPtr& joy) const{
  return (buttonPressed(button, joy) && buttonChanged(button, joy, m_previousJoystick));
}



/**
 * \brief Returns true when a button has changed compared to previous joystick state
 *
 * @param button
 * @param joy
 * @param prevJoy
 * @return
 */

bool TeleopNaoJoy::buttonChanged(int button, const Joy::ConstPtr& joy, const Joy::ConstPtr& prevJoy) const{
  return (unsigned(button) < joy->buttons.size() && joy->buttons[button] != prevJoy->buttons[button] );
}

/**
 * \brief Check whether an axis id is valid
 *
 * @param axis
 * @param joy
 * @return
 */
bool TeleopNaoJoy::axisValid(int axis, const Joy::ConstPtr& joy) const{
  return (axis >= 0 && unsigned(axis) < joy->buttons.size());
}

/**
 * \brief Publish motion message to Nao
 */
void TeleopNaoJoy::pubMsg(){
  if (m_enabled && m_inhibitCounter == 0)
  {
    const bool headZero = m_headAngles.joint_angles[0] == 0.0f && m_headAngles.joint_angles[1] == 0.0f;
    // Send head angle only if it is non-zero or the previous angle was non-zero.
    // This avoids sending zero angles repeatedly, which would interfere with other
    // modules sending motion commands (e.g., planners) when the joystick is not in use.
    if (!headZero || !m_prevHeadZero)
    {
      m_headPub.publish(m_headAngles);
      std::cout << "going to publish head angles" << std::endl;
    }
    m_prevHeadZero = headZero;

    const bool motionZero = m_motion.linear.x == 0.0f && m_motion.linear.y == 0.0f && m_motion.angular.z == 0.0f;
    if (!motionZero || !m_prevMotionZero)
    {
      m_movePub.publish(m_motion);
      std::cout << "going to publish motion commands" << std::endl;
    }
    m_prevMotionZero = motionZero;

  }
}

/**
 * \brief Inhibit joystick from sending walk commands.
 *
 * The teleop_nao node sends walk commands to the robot continuously,
 * even when the joystick is in its zero position. This can
 * disturb motions initiated by other nodes. To stop the teleop_nao
 * node from publishing walk commands, call this service before you
 * execute your own motion commands.
 *
 * Call the /uninhibit_walk service to enable sending walk commands again.
 *
 */
bool TeleopNaoJoy::inhibitWalk(std_srvs::EmptyRequest& /*req*/, std_srvs::EmptyResponse& res) {
  if(m_inhibitCounter == 0) {
    // not yet inhibited: publish zero walk command before inhibiting joystick
    m_motion.linear.x = 0.0;
    m_motion.linear.y = 0.0;
    m_motion.linear.z = 0.0;
    m_motion.angular.x = 0.0;
    m_motion.angular.y = 0.0;
    m_motion.angular.z = 0.0;
    naoqi_bridge_msgs::CmdVelService service_call;
    service_call.request.twist = m_motion;
    m_cmdVelClient.call(service_call);
  }
  m_inhibitCounter++;
  ROS_DEBUG("Inhibit counter: %d", m_inhibitCounter);
  return true;
}

/**
 * \brief Uninhibit joystick from sending walk commands.
 *
 * Call the /uninhibit_walk service to enable sending walk
 * commands again.
 *
 * If /inhibit_walk was called more than once, the walk commands
 * will only be enabled after /uninhibit_walk was called for
 * the same number of times.
 */
bool TeleopNaoJoy::uninhibitWalk(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse& res) {
  if(m_inhibitCounter > 0) {
    m_inhibitCounter--;
    ROS_DEBUG("Inhibit counter: %d", m_inhibitCounter);
  } else {
    m_inhibitCounter = 0;
    ROS_WARN("/uninhibit_walk called more times than /inhibit_walk - ignoring");
  }
  return true;
}
}

