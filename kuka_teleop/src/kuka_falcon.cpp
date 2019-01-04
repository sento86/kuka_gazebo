/*
 * baxter_teleop
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

///\author Vicent Girbés Juan
///\brief Converts joystick commands on /joy to twist commands on /cmd_vel.

//***********************************************

#define DEFAULT_MAX_LEVEL           1.0
#define NUM_AXIS                    4
#define NUM_BUTTONS                 4
#define JOYSTICK_HZ                 100.0   //Desired frequency
#define JOYSTICK_TIMER              -1.0    //Time (secs) max. without reading (Timeout for joystick reset (<0 no timeout))

//***********************************************

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include <control_toolbox/pid.h>

//***********************************************

#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/WrenchStamped.h"
#include "baxter_core_msgs/EndpointState.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Header.h"

//***********************************************

using namespace std;

//***********************************************

class TeleopJoy
{
  public:
  geometry_msgs::TwistStamped cmd_vel;
  geometry_msgs::PoseStamped cmd_pose;
  sensor_msgs::JointState cmd_joint;
  std_msgs::Header cmd_gripper;
  //joy::Joy joy;
  geometry_msgs::Quaternion req_q;
  double req_vx, req_vy, req_vz, req_wx, req_wy, req_wz;
  double sign_vx, sign_vy, sign_vz, sign_wx, sign_wy, sign_wz;
  double max_vx, max_vy, max_vz, max_wx, max_wy, max_wz;
  double max_dx, max_dy, max_dz, max_dR, max_dP, max_dY;
  int axis_vx, axis_vy, axis_vz, axis_wx, axis_wy, axis_wz;
  double req_q0, req_q1, req_q2, req_q3, req_q4, req_q5, req_q6;
  int pos_button, ang_button, deadman_button, gripper_button;
  int mode, num_modes, num_joints;
  bool pos_, ang_;
  bool lock_, sim_, feedback_, gripper_, last_gripper_;
  bool deadman_no_publish_, deadman_, last_deadman_;
  unsigned int req_gripper;
  std::string limb_, control_;
  std::string last_selected_topic_;
  std::string cmd_vel_topic_, cmd_pose_topic_, cmd_joint_topic_, cmd_gripper_topic_;
  std::string target_topic_, haptic_topic_, mode_topic_;
  geometry_msgs::Vector3 haptic_position, target_position, target_orientation;
  geometry_msgs::Vector3 haptic_feedback;
  std_msgs::UInt8 haptic_mode;
  sensor_msgs::JointState joint_state;
  geometry_msgs::WrenchStamped wrench;
  baxter_core_msgs::EndpointState endpoint;

  tf::Quaternion q;
  tf::Matrix3x3 Ree; // Rotation matrix of End Effector
  double sign_limb;
  double offset_x, offset_y, offset_z;

  unsigned int state;
  unsigned int ix, iy, iz, i0;
  double K;
  double req_vx0, req_vy0, req_vz0;
  double req_wx0, req_wy0, req_wz0;
  double joy_wx_old, joy_wy_old, joy_wz_old;
  std::vector<unsigned int> list;

  control_toolbox::Pid pid_vx, pid_vy, pid_vz;
  control_toolbox::Pid pid_wx, pid_wy, pid_wz;

  // Current and last time
  ros::Time ros_time_now, ros_time_last;
  // Delay of time since last control action
  ros::Duration delay_time;

  sensor_msgs::Joy last_processed_joy_message_;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_, pose_pub_, joint_pub_, gripper_pub_;
  ros::Publisher target_pub_, haptic_pub_, mode_pub_;
  ros::Subscriber joy_sub_, joint_sub_, wrench_sub_, endpoint_sub_, position_sub_;

  TeleopJoy(bool deadman_no_publish = false) :
	sign_vx(1.0), sign_vy(1.0), sign_vz(1.0),
	sign_wx(1.0), sign_wy(1.0), sign_wz(1.0),
    max_vx(0.1), max_vy(0.1), max_vz(0.1),
    max_wx(0.5), max_wy(0.5), max_wz(0.5),
    max_dx(0.1), max_dy(0.1), max_dz(0.1),
    max_dR(0.5), max_dP(0.5), max_dY(0.5),
    mode(0), num_modes(2), num_joints(7),
	deadman_no_publish_(deadman_no_publish),
    deadman_(false), last_deadman_(false),
    pos_(true), ang_(false),
    lock_(false), sim_(false),
	gripper_(false), last_gripper_(false),
	n_private_("~")
  { }

  void init()
  {
	cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.linear.z = 0.0;
	cmd_vel.twist.angular.x = cmd_vel.twist.angular.y = cmd_vel.twist.angular.z = 0;
	cmd_pose.pose.position.x = cmd_pose.pose.position.y = cmd_pose.pose.position.z = 0.0;
	cmd_pose.pose.orientation.w = cmd_pose.pose.orientation.x = cmd_pose.pose.orientation.y = cmd_pose.pose.orientation.z = 0.0;

	// Set messages topics
	n_private_.param("cmd_vel_topic", cmd_vel_topic_, std::string("cmd_vel"));
	n_private_.param("cmd_pose_topic", cmd_pose_topic_, std::string("cmd_pose"));
    n_private_.param("cmd_joint_topic", cmd_joint_topic_, std::string("cmd_joint/states"));
	n_private_.param("cmd_gripper_topic", cmd_gripper_topic_, std::string("cmd_gripper"));
	n_private_.param("target_topic", target_topic_, std::string("follower/target"));
	n_private_.param("haptic_topic", haptic_topic_, std::string("follower/force"));
	n_private_.param("mode_topic", mode_topic_, std::string("follower/mode"));
	n_private_.param("deadman_no_publish", deadman_no_publish_, deadman_no_publish_);
	n_private_.param("limb", limb_, std::string("left"));
	n_private_.param("sim", sim_, false);
	n_private_.param("control", control_, std::string("velocity"));
	n_private_.param("feedback", feedback_, false);

	// Set max speed (cartesian)
	n_private_.param("max_vx", max_vx, max_vx);
	n_private_.param("max_vy", max_vy, max_vy);
	n_private_.param("max_vz", max_vz, max_vz);
	n_private_.param("max_wx", max_wx, max_wx);
	n_private_.param("max_wy", max_wy, max_wy);
	n_private_.param("max_wz", max_wz, max_wz);

	// Set max position (cartesian)
	n_private_.param("max_dx", max_dx, max_dx);
	n_private_.param("max_dy", max_dy, max_dy);
	n_private_.param("max_dz", max_dz, max_dz);
	n_private_.param("max_dR", max_dR, max_dR);
	n_private_.param("max_dP", max_dP, max_dP);
	n_private_.param("max_dY", max_dY, max_dY);

	// Set signs
	n_private_.param("sign_vx", sign_vx, 1.0);
	n_private_.param("sign_vy", sign_vy, 1.0);
	n_private_.param("sign_vz", sign_vz, 1.0);
	n_private_.param("sign_wx", sign_wx, 1.0);
	n_private_.param("sign_wy", sign_wy, 1.0);
	n_private_.param("sign_wz", sign_wz, 1.0);

	// Set axis
	n_private_.param("axis_vx", axis_vx, 3);
	n_private_.param("axis_vy", axis_vy, 0);
	n_private_.param("axis_vz", axis_vz, 4);
	n_private_.param("axis_wx", axis_wx, 0);
	n_private_.param("axis_wy", axis_wy, 3);
	n_private_.param("axis_wz", axis_wz, 1);

	// Set buttons
	n_private_.param("pos_button", pos_button, 0);
	n_private_.param("ang_button", ang_button, 3);
	n_private_.param("deadman_button", deadman_button, 2);
	n_private_.param("gripper_button", gripper_button, 1);

    n_private_.param("num_modes", num_modes, num_modes);
    n_private_.param("num_joints", num_joints, num_joints);

    cmd_joint.name.resize(num_joints);
    cmd_joint.position.resize(num_joints);
    //cmd_joint.velocity.resize(num_joints);
    //cmd_joint.effort.resize(num_joints);
    for(int i=0; i<num_joints; i++){
        std::string joint_name = "q"+std::to_string(i);
        cmd_joint.name[i]=joint_name;
        cmd_joint.position[i]=0.0;
    }

	double joy_msg_timeout;
    n_private_.param("joy_msg_timeout", joy_msg_timeout, JOYSTICK_TIMER); //default to JOYSTICK_TIMER=1.0 seconds timeout
	if (joy_msg_timeout <= 0)
	{
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	}
	else
	{
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	}

	// Set max speed (cartesian)
	ROS_DEBUG("max_vx: %.3f m/s", max_vx);
	ROS_DEBUG("max_vy: %.3f m/s", max_vy);
	ROS_DEBUG("max_vz: %.3f m/s", max_vz);
	ROS_DEBUG("max_wx: %.3f deg/s", max_wx*180.0/M_PI);
	ROS_DEBUG("max_wy: %.3f deg/s", max_wy*180.0/M_PI);
	ROS_DEBUG("max_wz: %.3f deg/s", max_wz*180.0/M_PI);

	// Set max distance (cartesian)
	ROS_DEBUG("max_dx: %.3f m", max_dx);
	ROS_DEBUG("max_dy: %.3f m", max_dy);
	ROS_DEBUG("max_dz: %.3f m", max_dz);
	ROS_DEBUG("max_dR: %.3f deg", max_dR*180.0/M_PI);
	ROS_DEBUG("max_dP: %.3f deg", max_dP*180.0/M_PI);
	ROS_DEBUG("max_dY: %.3f deg", max_dY*180.0/M_PI);

	// Set signs
	ROS_DEBUG("sign_vx: %f", sign_vx);
	ROS_DEBUG("sign_vy: %f", sign_vy);
	ROS_DEBUG("sign_vz: %f", sign_vz);
	ROS_DEBUG("sign_wx: %f", sign_wx);
	ROS_DEBUG("sign_wy: %f", sign_wy);
	ROS_DEBUG("sign_wz: %f", sign_wz);

	// Set axis
	ROS_DEBUG("axis_vx: %d", axis_vx);
	ROS_DEBUG("axis_vy: %d", axis_vy);
	ROS_DEBUG("axis_vz: %d", axis_vz);
	ROS_DEBUG("axis_wx: %d", axis_wx);
	ROS_DEBUG("axis_wy: %d", axis_wy);
	ROS_DEBUG("axis_wz: %d", axis_wz);

	// Set buttons
	ROS_DEBUG("pos_button: %d", pos_button);
	ROS_DEBUG("ang_button: %d", ang_button);
	ROS_DEBUG("deadman_button: %d", deadman_button);
	ROS_DEBUG("gripper_button: %d", gripper_button);
	ROS_DEBUG("joy_msg_timeout: %f", joy_msg_timeout);

	vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic_, 1);
	pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>(cmd_pose_topic_, 1);
    joint_pub_ = n_.advertise<sensor_msgs::JointState>(cmd_joint_topic_, 1);
	gripper_pub_ = n_.advertise<std_msgs::Header>(cmd_gripper_topic_, 1);

	target_pub_ = n_.advertise<geometry_msgs::Vector3>(target_topic_, 1);
	haptic_pub_ = n_.advertise<geometry_msgs::Vector3>(haptic_topic_, 1);
	mode_pub_ = n_.advertise<std_msgs::UInt8>(mode_topic_, 1);

	joy_sub_ = n_.subscribe("follower/joystick", 10, &TeleopJoy::joy_cb, this);
	position_sub_ = n_.subscribe("follower/position", 10, &TeleopJoy::position_cb, this);
	wrench_sub_ = n_.subscribe(limb_+"/ft_sensor", 10, &TeleopJoy::wrench_cb, this);
	endpoint_sub_ = n_.subscribe("robot/limb/"+limb_+"/endpoint_state", 10, &TeleopJoy::endpoint_cb, this);
    joint_sub_ = n_.subscribe("robot/joint_states", 10, &TeleopJoy::joint_cb, this);


    if((control_=="joint")||(control_=="position")){
		if(limb_=="right")
			sign_limb = 1.0;
		else if(limb_=="left")
			sign_limb = -1.0;
		else
			sign_limb = 0.0;
    }
	else{
		sign_limb = 0.0;
	}
	target_position.x = 0.05*sign_limb; // Sign depends on the arm to be controlled (left -, right +)
	target_position.y = 0.0;
	target_position.z = 0.125;
	target_orientation.x = 0.0;
	target_orientation.y = 0.0;
	target_orientation.z = 0.125;

	req_vx = req_vy = req_vz = req_wx = req_wy = req_wz = 0.0;
	req_q0 = req_q1 = req_q2 = req_q3 = req_q4 = req_q5 = req_q6 = 0.0;

	// Initial position of End-Effector
	offset_x =  0.5;
	offset_y = -0.2*sign_limb;
	offset_z =  0.0;

	// Initial rotation of End-Effector
	Ree.setRPY(M_PI, 0.0, M_PI);
	Ree.getRotation(q);
	tf::quaternionTFToMsg(q,req_q);

	state = 0;
	ix = iy = iz = i0 = 0;
  	req_vx0 = req_vy0 = req_vz0 = 0.0;
  	req_wx0 = req_wy0 = req_wz0 = 0.0;
  	joy_wx_old = joy_wy_old = joy_wz_old = 0.0;
  	list.assign(50, 0);

    // Set parameters of PID controllers
    double Kp_v=10.0,Ki_v=0.0,Kd_v=0.0;
    pid_vx.initPid(Kp_v, Ki_v, Kd_v, 1.0, -1.0);
    pid_vy.initPid(Kp_v, Ki_v, Kd_v, 1.0, -1.0);
    pid_vz.initPid(Kp_v, Ki_v, Kd_v, 1.0, -1.0);

    double Kp_w=15.0,Ki_w=0.0,Kd_w=0.0;
    pid_wx.initPid(Kp_w, Ki_w, Kd_w, 1.0, -1.0);
    pid_wy.initPid(Kp_w, Ki_w, Kd_w, 1.0, -1.0);
    pid_wz.initPid(Kp_w, Ki_w, Kd_w, 1.0, -1.0);

  }

  ~TeleopJoy() { }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
	// Do not process the same message twice.
	if(joy_msg->header.stamp == last_processed_joy_message_.header.stamp) {
		// notify the user only if the problem persists
		if(ros::Time::now() - joy_msg->header.stamp > ros::Duration(5.0/JOYSTICK_HZ))
			ROS_WARN_THROTTLE(1.0, "Received Joy message with same timestamp multiple times. Ignoring subsequent messages.");
		deadman_ = false;
		return;
	}
	last_processed_joy_message_ = *joy_msg;

    //Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();

    if(((unsigned int)pos_button < joy_msg->buttons.size()) && joy_msg->buttons[pos_button]){
    	pos_ = true;
    	ang_ = false;
    }
    if(((unsigned int)ang_button < joy_msg->buttons.size()) && joy_msg->buttons[ang_button]){
    	ang_ = true;
    	pos_ = false;
    }
    if(((unsigned int)gripper_button < joy_msg->buttons.size()) && joy_msg->buttons[gripper_button])
    	gripper_ = true;
    else
    	gripper_ = false;

	if(gripper_&&(gripper_ != last_gripper_)){
		if(req_gripper==0)
			req_gripper = 1;
		else
			req_gripper = 0;
	}
	last_gripper_ = gripper_;

	if(req_gripper != cmd_gripper.seq){
		cmd_gripper.stamp = ros::Time::now();
		cmd_gripper.seq = req_gripper;
		cmd_gripper.frame_id = "base";
		gripper_pub_.publish(cmd_gripper);
	}

    deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);

//    if (!deadman_)
//      return;

    if(deadman_){

		// Update axis values
		if(control_=="joint"){
			if(pos_){
				if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()))
					req_vx = joy_msg->axes[axis_vx] * max_dx * sign_vx;
				else
					req_vx = 0.0;
				if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()))
					req_vy = joy_msg->axes[axis_vy] * max_dy * sign_vy;
				else
					req_vy = 0.0;
				if((axis_vz >= 0) && (((unsigned int)axis_vz) < joy_msg->axes.size()))
					req_vz = joy_msg->axes[axis_vz] * max_dz * sign_vz;
				else
					req_vz = 0.0;

				// Enforce max/mins for velocity
				// Joystick should be [-1, 1], but it might not be
				req_vx = max(min(req_vx, max_dx), -max_dx);
				req_vy = max(min(req_vy, max_dy), -max_dy);
				req_vz = max(min(req_vz, max_dz), -max_dz);

				// Neutral position offset
				req_vx += offset_x;
				req_vy += offset_y;
				req_vz += offset_z;

			}else if(ang_){
				if((axis_wx >= 0) && (((unsigned int)axis_wx) < joy_msg->axes.size()))
					req_wx = joy_msg->axes[axis_wx] * max_dR * sign_wx;
				else
					req_wx = 0.0;
				if((axis_wy >= 0) && (((unsigned int)axis_wy) < joy_msg->axes.size()))
					req_wy = joy_msg->axes[axis_wy] * max_dP * sign_wy;
				else
					req_wy = 0.0;
				if((axis_wz >= 0) && (((unsigned int)axis_wz) < joy_msg->axes.size()))
					req_wz = joy_msg->axes[axis_wz] * max_dY * sign_wz;
				else
					req_wz = 0.0;

				// Enforce max/mins for velocity
				// Joystick should be [-1, 1], but it might not be
				req_wx = max(min(req_wx, max_dR), -max_dR);
				req_wy = max(min(req_wy, max_dP), -max_dP);
				req_wz = max(min(req_wz, max_dY), -max_dY);
			}

			// Set a deadzone to get rid of noise
			double deadzone = 0.02;
			if(fabs(req_vx)<deadzone)
				req_vx = 0.0;
			if(fabs(req_vy)<deadzone)
				req_vy = 0.0;
			if(fabs(req_vz)<deadzone)
				req_vz = 0.0;
			if(fabs(req_wx)<deadzone)
				req_wx = 0.0;
			if(fabs(req_wy)<deadzone)
				req_wy = 0.0;
			if(fabs(req_wz)<deadzone)
				req_wz = 0.0;

		}else if(control_=="position"){
			if(pos_){
				if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()))
					req_vx = joy_msg->axes[axis_vx] * max_dx * sign_vx;
				else
					req_vx = 0.0;
				if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()))
					req_vy = joy_msg->axes[axis_vy] * max_dy * sign_vy;
				else
					req_vy = 0.0;
				if((axis_vz >= 0) && (((unsigned int)axis_vz) < joy_msg->axes.size()))
					req_vz = joy_msg->axes[axis_vz] * max_dz * sign_vz;
				else
					req_vz = 0.0;

				// Enforce max/mins for velocity
				// Joystick should be [-1, 1], but it might not be
				req_vx = max(min(req_vx, max_dx), -max_dx);
				req_vy = max(min(req_vy, max_dy), -max_dy);
				req_vz = max(min(req_vz, max_dz), -max_dz);

				// Neutral position offset
				req_vx += offset_x;
				req_vy += offset_y;
				req_vz += offset_z;

			}else if(ang_){
				if((axis_wx >= 0) && (((unsigned int)axis_wx) < joy_msg->axes.size()))
					req_wx = joy_msg->axes[axis_wx] * max_dR * sign_wx;
				else
					req_wx = 0.0;
				if((axis_wy >= 0) && (((unsigned int)axis_wy) < joy_msg->axes.size()))
					req_wy = joy_msg->axes[axis_wy] * max_dP * sign_wy;
				else
					req_wy = 0.0;
				if((axis_wz >= 0) && (((unsigned int)axis_wz) < joy_msg->axes.size()))
					req_wz = joy_msg->axes[axis_wz] * max_dY * sign_wz;
				else
					req_wz = 0.0;

				// Enforce max/mins for velocity
				// Joystick should be [-1, 1], but it might not be
				req_wx = max(min(req_wx, max_dR), -max_dR);
				req_wy = max(min(req_wy, max_dP), -max_dP);
				req_wz = max(min(req_wz, max_dY), -max_dY);
			}

			// Set a deadzone to get rid of noise
			double deadzone = 0.02;
			if(fabs(req_vx)<deadzone)
				req_vx = 0.0;
			if(fabs(req_vy)<deadzone)
				req_vy = 0.0;
			if(fabs(req_vz)<deadzone)
				req_vz = 0.0;
			if(fabs(req_wx)<deadzone)
				req_wx = 0.0;
			if(fabs(req_wy)<deadzone)
				req_wy = 0.0;
			if(fabs(req_wz)<deadzone)
				req_wz = 0.0;

			tf::Matrix3x3 RM;
			RM.setRPY(req_wx, req_wy, req_wz);
			Ree.setRPY(M_PI, 0.0, M_PI);// Initial rotation of End-Effector
			RM = RM*Ree;
			RM.getRotation(q);
			tf::quaternionTFToMsg(q,req_q);

		}else if(control_=="velocity"){

			if(pos_){
				req_wx = 0.0;
				req_wy = 0.0;
				req_wz = 0.0;
				if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()))
					req_vx = joy_msg->axes[axis_vx] * max_vx * sign_vx;
				else
					req_vx = 0.0;
				if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()))
					req_vy = joy_msg->axes[axis_vy] * max_vy * sign_vy;
				else
					req_vy = 0.0;
				if((axis_vz >= 0) && (((unsigned int)axis_vz) < joy_msg->axes.size()))
					req_vz = joy_msg->axes[axis_vz] * max_vz * sign_vz;
				else
					req_vz = 0.0;

			}else if(ang_){
				req_vx = 0.0;
				req_vy = 0.0;
				req_vz = 0.0;
				if((axis_wx >= 0) && (((unsigned int)axis_wx) < joy_msg->axes.size()))
					req_wx = joy_msg->axes[axis_wx] * max_wx * sign_wx;
				else
					req_wx = 0.0;
				if((axis_wy >= 0) && (((unsigned int)axis_wy) < joy_msg->axes.size()))
					req_wy = joy_msg->axes[axis_wy] * max_wy * sign_wy;
				else
					req_wy = 0.0;
				if((axis_wz >= 0) && (((unsigned int)axis_wz) < joy_msg->axes.size()))
					req_wz = joy_msg->axes[axis_wz] * max_wz * sign_wz;
				else
					req_wz = 0.0;
			}

			// Enforce max/mins for velocity
			// Joystick should be [-1, 1], but it might not be
			req_vx = max(min(req_vx, max_vx), -max_vx);
			req_vy = max(min(req_vy, max_vy), -max_vy);
			req_vz = max(min(req_vz, max_vz), -max_vz);
			req_wx = max(min(req_wx, max_wx), -max_wx);
			req_wy = max(min(req_wy, max_wy), -max_wy);
			req_wz = max(min(req_wz, max_wz), -max_wz);

			// Set a deadzone to get rid of noise
			double deadzone = 0.05;
			if(fabs(req_vx)<deadzone)
				req_vx = 0.0;
			if(fabs(req_vy)<deadzone)
				req_vy = 0.0;
			if(fabs(req_vz)<deadzone)
				req_vz = 0.0;
			if(fabs(req_wx)<deadzone)
				req_wx = 0.0;
			if(fabs(req_wy)<deadzone)
				req_wy = 0.0;
			if(fabs(req_wz)<deadzone)
				req_wz = 0.0;

		}else{

			// Enforce max/mins for velocity
			// Joystick should be [-1, 1], but it might not be
			req_vx = 0.0;
			req_vy = 0.0;
			req_vz = 0.0;
			req_wx = 0.0;
			req_wy = 0.0;
			req_wz = 0.0;
		}

		// Unlock device when deadman button pressed (white)
	    if(control_=="joint"){
			// Save current position to lock haptic device
			target_position.z = (endpoint.pose.position.x - offset_x)/(max_dx * sign_vx);
			target_position.x = (endpoint.pose.position.y - offset_y)/(max_dy * sign_vy);
			target_position.y = (endpoint.pose.position.z - offset_z)/(max_dz * sign_vz);

			// Normalization
			target_position.x = max(min(target_position.x,1.0),-1.0);
			target_position.y = max(min(target_position.y,1.0),-1.0);
			target_position.z = max(min(target_position.z,1.0),-1.0);

			// Scale to falcon device's workspace
			target_position.x = target_position.x/20.0;
			target_position.y = target_position.y/20.0;
			target_position.z = target_position.z/20.0+0.125;

			// Save current orientation to lock haptic device
			target_orientation.x = (req_q4)/(max_dR * sign_wx);
			target_orientation.z = (req_q5)/(max_dP * sign_wy);
			target_orientation.y = (req_q6)/(max_dY * sign_wz);

			// Normalization
			target_orientation.x = max(min(target_orientation.x,1.0),-1.0);
			target_orientation.y = max(min(target_orientation.y,1.0),-1.0);
			target_orientation.z = max(min(target_orientation.z,1.0),-1.0);

			// Scale to falcon device's workspace
			target_orientation.x = target_orientation.x/20.0;
			target_orientation.y = target_orientation.y/20.0;
			target_orientation.z = target_orientation.z/20.0+0.125;

    	}else if(control_=="position"){
//			// Save current position to lock haptic device
//			if(pos_){
//				// Save current position to lock haptic device
//				target_position = haptic_position;
//
//			}else if(ang_){
//				// Save current orientation to lock haptic device
//				target_orientation = haptic_position;
//			}
    		if(pos_){
			// Save current position to lock haptic device
			target_position.z = (endpoint.pose.position.x - offset_x)/(max_dx * sign_vx);
			target_position.x = (endpoint.pose.position.y - offset_y)/(max_dy * sign_vy);
			target_position.y = (endpoint.pose.position.z - offset_z)/(max_dz * sign_vz);

			// Normalization
			target_position.x = max(min(target_position.x,1.0),-1.0);
			target_position.y = max(min(target_position.y,1.0),-1.0);
			target_position.z = max(min(target_position.z,1.0),-1.0);

			// Scale to falcon device's workspace
			target_position.x = target_position.x/20.0;
			target_position.y = target_position.y/20.0;
			target_position.z = target_position.z/20.0+0.125;
    		}else if(ang_){
			// Save current orientation to lock haptic device
//			target_orientation.x = (req_q4)/(max_dR * sign_wx);
//			target_orientation.z = (req_q5)/(max_dP * sign_wy);
//			target_orientation.y = (req_q6)/(max_dY * sign_wz);

			//RM.getRotation(q);
			tf::quaternionMsgToTF(endpoint.pose.orientation, q);
			tf::Matrix3x3 RM(q);
			Ree.setRPY(M_PI, 0.0, M_PI);// Initial rotation of End-Effector
			RM = RM*Ree.inverse();
			RM.getRPY(req_wx, req_wy, req_wz);

			target_orientation.x = (req_wx)/(max_dR * sign_wx);
			target_orientation.z = (req_wy)/(max_dP * sign_wy);
			target_orientation.y = (req_wz)/(max_dY * sign_wz);

			// Normalization
			target_orientation.x = max(min(target_orientation.x,1.0),-1.0);
			target_orientation.y = max(min(target_orientation.y,1.0),-1.0);
			target_orientation.z = max(min(target_orientation.z,1.0),-1.0);

			// Scale to falcon device's workspace
			target_orientation.x = target_orientation.x/20.0;
			target_orientation.y = target_orientation.y/20.0;
			target_orientation.z = target_orientation.z/20.0+0.125;

    		}
    	}

	}else{

		// Lock device when deadman button not pressed (white)
		if(pos_)
			target_pub_.publish(target_position);
		else if(ang_)
			target_pub_.publish(target_orientation);

	}

	if(deadman_ != last_deadman_){
		if((control_=="joint")||(control_=="position")){
			if(deadman_){ // Unlock device when deadman button pressed (white)
				haptic_mode.data = 1;
				mode_pub_.publish(haptic_mode);
			}else{ // Lock device when deadman button not pressed (white)
				haptic_mode.data = 0;
				mode_pub_.publish(haptic_mode);
			}
		}
	}
    last_deadman_ = deadman_;

  }

  /** Callback for phantom omni joint states topic **/
  void joint_cb(const sensor_msgs::JointState::ConstPtr& joint_msg)
  {
	  joint_state = *joint_msg;

	  if(control_=="joint"){
		//if(pos_){
			// Update joint references
			for(int i=0; i<joint_state.position.size(); i++){
				// Joint space control enabled
				if(joint_state.name[i]==std::string(limb_+"_s0"))
					req_q0 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_s1"))
					req_q1 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_e0"))
					req_q2 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_e1"))
					req_q3 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_w0"))
					req_q4 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_w1"))
					req_q5 = joint_state.position[i];
				else if(joint_state.name[i]==std::string(limb_+"_w2"))
					req_q6 = joint_state.position[i];
			}
		//}
	  }
  }

  /** Callback for Falcon haptic pose topic **/
  void position_cb(const geometry_msgs::Vector3::ConstPtr& position_msg)
  {
	  haptic_position = *position_msg;
  }

  /** Callback for Baxter's endpoint topic **/
  void endpoint_cb(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
  {
	  endpoint = *endpoint_msg;

	  if(control_=="joint"){
		  if(ang_)// Update orientation reference
			req_q = endpoint.pose.orientation;
	  }
  }

  /** Callback for force/torque sensor topic **/
  void wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
  {
	wrench = *wrench_msg;

	if(deadman_==true){

        double scale;
        tf::Vector3 force_global;
        if(sim_){
            scale = 0.1; // Sim
            force_global.setValue(-wrench.wrench.force.y*scale, -wrench.wrench.force.z*scale, wrench.wrench.force.x*scale); // Sim
        }else{
        	scale = 1.0; // Real
        	force_global.setValue(-wrench.wrench.force.y*scale, wrench.wrench.force.z*scale, -wrench.wrench.force.x*scale); // Real
        }

        double dt = 0.01; // Sampling time
        double RC = 0.04; // Time constant (Real)
        if(sim_)
            RC = 0.1; // Time constant (Sim)
		double alpha = dt/(RC+dt);
		haptic_feedback.x = alpha*force_global.x() + (1.0-alpha)*haptic_feedback.x;
		haptic_feedback.y = alpha*force_global.y() + (1.0-alpha)*haptic_feedback.y;
		haptic_feedback.z = alpha*force_global.z() + (1.0-alpha)*haptic_feedback.z;

		// Dead zone correction
		double dead_zone = 0.5;
		if(fabs(haptic_feedback.x)<dead_zone)
			haptic_feedback.x=0.0;
		else if(haptic_feedback.x>=dead_zone)
			haptic_feedback.x=haptic_feedback.x-dead_zone;
		else if(haptic_feedback.x<=-dead_zone)
			haptic_feedback.x=haptic_feedback.x+dead_zone;
		if(fabs(haptic_feedback.y)<dead_zone)
			haptic_feedback.y=0.0;
		else if(haptic_feedback.y>=dead_zone)
			haptic_feedback.y=haptic_feedback.y-dead_zone;
		else if(haptic_feedback.y<=-dead_zone)
			haptic_feedback.y=haptic_feedback.y+dead_zone;
		if(fabs(haptic_feedback.z)<dead_zone)
			haptic_feedback.z=0.0;
		else if(haptic_feedback.z>=dead_zone)
			haptic_feedback.z=haptic_feedback.z-dead_zone;
		else if(haptic_feedback.z<=-dead_zone)
			haptic_feedback.z=haptic_feedback.z+dead_zone;
		// Saturation
		double max_force = 20.0;
		if(haptic_feedback.x>max_force)
			haptic_feedback.x=max_force;
		else if(haptic_feedback.x<-max_force)
			haptic_feedback.x=-max_force;
		if(haptic_feedback.y>max_force)
			haptic_feedback.y=max_force;
		else if(haptic_feedback.y<-max_force)
			haptic_feedback.y=-max_force;
		if(haptic_feedback.z>max_force)
			haptic_feedback.z=max_force;
		else if(haptic_feedback.z<-max_force)
			haptic_feedback.z=-max_force;
		ROS_DEBUG("feedback: %f %f %f", haptic_feedback.x, haptic_feedback.y, haptic_feedback.z);
	}
  }

  double angle_diff(double  angle1, double  angle2){

       // Rotate angle1 with angle2 so that the sought after
       // angle is between the resulting angle and the x-axis
       double angle = angle1 - angle2;

       // "Normalize" angle to range [-180,180[
       if (angle < -M_PI)
           angle = angle+M_PI*2;
       else if (angle > M_PI)
           angle = angle-M_PI*2;

       return angle;
  }

  void send_cmd()
  {
    if(deadman_ && last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
		// Save current time
		ros_time_now = ros::Time::now();
		// Delay of time since last control action
		delay_time = ros_time_now - ros_time_last;
		// Update last time for next control step
		ros_time_last = ros_time_now;

    	if(pos_){
//			haptic_feedback.x = 0.0;
//			haptic_feedback.y = 0.0;
//			haptic_feedback.z = 0.0;

			haptic_feedback.x =  pid_vy.computeCommand(req_vy-endpoint.pose.position.y, delay_time);
			haptic_feedback.y = -pid_vz.computeCommand(req_vz-endpoint.pose.position.z, delay_time);
			haptic_feedback.z =  pid_vx.computeCommand(req_vx-endpoint.pose.position.x, delay_time);

    	}else if(ang_){
			// Process force-feedback
			double dx = last_processed_joy_message_.axes[axis_wx] - joy_wx_old;
			double dy = last_processed_joy_message_.axes[axis_wy] - joy_wy_old;
			double dz = last_processed_joy_message_.axes[axis_wz] - joy_wz_old;
			joy_wx_old = last_processed_joy_message_.axes[axis_wx];
			joy_wy_old = last_processed_joy_message_.axes[axis_wy];
			joy_wz_old = last_processed_joy_message_.axes[axis_wz];

			double dmin = 1e-6;
			if((abs(dx)>dmin)&&((abs(dx)>abs(dy))&&(abs(dx)>abs(dz)))){
				list.push_back(1);
				list.erase(list.begin());
			}else if((abs(dy)>dmin)&&(abs(dy)>abs(dz))){
				list.push_back(2);
				list.erase(list.begin());
			}else if(abs(dz)>dmin){
				list.push_back(3);
				list.erase(list.begin());
			}else{
				list.push_back(0);
				list.erase(list.begin());
			}

			ix=iy=iz=i0=0;
			for (int i=0; i<list.size(); i++){

				switch(list.at(i)) {
					case 0: i0++;
							if(i0>list.size())
								i0=list.size();
							break;
					case 1: ix++;
							if(ix>list.size())
								ix=list.size();
							break;
					case 2: iy++;
							if(iy>list.size())
								iy=list.size();
							break;
					case 3: iz++;
							if(iz>list.size())
								iz=list.size();
							break;
				}
			}

//			double cx, cy, cz, cmin = 0.3, cmax = 0.7;
			double cx, cy, cz, cmin = 0.5;
			cx = double(ix)/double(list.size());
			cy = double(iy)/double(list.size());
			cz = double(iz)/double(list.size());

			if((cx>cmin)&&(cx>cy)&&(cx>cz)){
				req_wx0 = last_processed_joy_message_.axes[axis_wx];
				if(state!=1){
					state = 1;
					//req_wx0 = last_processed_joy_message_.axes[axis_wx];
					req_wy0 = last_processed_joy_message_.axes[axis_wy];
					req_wz0 = last_processed_joy_message_.axes[axis_wz];
				}
//				else if(cx>cmax){
//					cx=1.0;
//					cy=0.0;
//					cz=0.0;
//				}
			}else if((cy>cmin)&&(cy>cx)&&(cy>cz)){
				req_wy0 = last_processed_joy_message_.axes[axis_wy];
				if(state!=2){
					state = 2;
					req_wx0 = last_processed_joy_message_.axes[axis_wx];
					//req_wy0 = last_processed_joy_message_.axes[axis_wy];
					req_wz0 = last_processed_joy_message_.axes[axis_wz];
				}
//				else if(cy>cmax){
//					cx=0.0;
//					cy=1.0;
//					cz=0.0;
//				}
			}else if((cz>cmin)&&(cz>cx)&&(cz>cy)){
				req_wz0 = last_processed_joy_message_.axes[axis_wz];
				if(state!=3){
					state = 3;
					req_wx0 = last_processed_joy_message_.axes[axis_wx];
					req_wy0 = last_processed_joy_message_.axes[axis_wy];
					//req_wz0 = last_processed_joy_message_.axes[axis_wz];
				}
//				else if(cz>cmax){
//					cx=0.0;
//					cy=0.0;
//					cz=1.0;
//				}
			}else{
				state = 0;
				req_wx0 = last_processed_joy_message_.axes[axis_wx];
				req_wy0 = last_processed_joy_message_.axes[axis_wy];
				req_wz0 = last_processed_joy_message_.axes[axis_wz];
//				cx=0.0;
//				cy=0.0;
//				cz=0.0;
			}

			double ux, uy, uz;
			ux = pid_wx.computeCommand(req_wx0-last_processed_joy_message_.axes[axis_wx], delay_time);
			uy = pid_wz.computeCommand(req_wz0-last_processed_joy_message_.axes[axis_wz], delay_time);
			uz = pid_wy.computeCommand(req_wy0-last_processed_joy_message_.axes[axis_wy], delay_time);

			haptic_feedback.x = (1.0-cx)*ux;
			haptic_feedback.y = (1.0-cy)*uy;
			haptic_feedback.z = (1.0-cz)*uz;
    	}

        //Copy linear and angular velocities to twist message
    	if(control_=="joint"){
		  if(pos_){
			  // Copy position and orientation to pose message
			  cmd_pose.pose.position.x = req_vx;
			  cmd_pose.pose.position.y = req_vy;
			  cmd_pose.pose.position.z = req_vz;
			  cmd_pose.pose.orientation = req_q;
			  cmd_pose.header.stamp=ros::Time::now();
			  // Publish cmd_pose
			  pose_pub_.publish(cmd_pose);
			  // Publish haptic feedback
			  if(feedback_==false){
			        haptic_feedback.x = 0.0;
			        haptic_feedback.y = 0.0;
			        haptic_feedback.z = 0.0;
			  }
			  haptic_pub_.publish(haptic_feedback);
			  ROS_DEBUG("baxter_teleop::falcon (joint position mode): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);

		  }else if(ang_){
			  // Copy angles to joint message
			  cmd_joint.position[0] = req_q0;
			  cmd_joint.position[1] = req_q1;
			  cmd_joint.position[2] = req_q2;
			  cmd_joint.position[3] = req_q3;
//			  cmd_joint.position[4] = req_q4;
//			  cmd_joint.position[5] = req_q5;
//			  cmd_joint.position[6] = req_q6;
			  cmd_joint.position[4] = req_wx;
			  cmd_joint.position[5] = req_wy;
			  cmd_joint.position[6] = req_wz;
			  // Publish cmd_joint
			  cmd_joint.header.stamp=ros::Time::now();
			  joint_pub_.publish(cmd_joint);
			  // Publish haptic feedback
			  if(feedback_==false){
			        haptic_feedback.x = 0.0;
			        haptic_feedback.y = 0.0;
			        haptic_feedback.z = 0.0;
			  }
			  haptic_pub_.publish(haptic_feedback);
			  ROS_DEBUG("baxter_teleop::phantom (joint space control): q0 %f, q1 %f, q2 %f, q3 %f, q4 %f, q5 %f, q6 %f", cmd_joint.position[0], cmd_joint.position[1], cmd_joint.position[2], cmd_joint.position[3], cmd_joint.position[4], cmd_joint.position[5], cmd_joint.position[6]);
		  }
      }else if(control_=="position"){
		  if(pos_){
        	  // Copy position and orientation to pose message
			  cmd_pose.pose.position.x = req_vx;
			  cmd_pose.pose.position.y = req_vy;
			  cmd_pose.pose.position.z = req_vz;
//			  cmd_pose.pose.orientation = req_q;
			  // To avoid singularities
			  cmd_pose.pose.orientation.w = 0.0;
			  cmd_pose.pose.orientation.x = 0.0;
			  cmd_pose.pose.orientation.y = 0.0;
			  cmd_pose.pose.orientation.z = 0.0;
			  cmd_pose.header.stamp=ros::Time::now();
    		  // Publish cmd_pose
			  pose_pub_.publish(cmd_pose);
			  // Publish haptic feedback
			  if(feedback_==false){
			        haptic_feedback.x = 0.0;
			        haptic_feedback.y = 0.0;
			        haptic_feedback.z = 0.0;
			  }
			  haptic_pub_.publish(haptic_feedback);
			  ROS_DEBUG("baxter_teleop::falcon (position mode): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);

		  }else if(ang_){
        	  // Copy position and orientation to pose message
//			  cmd_pose.pose.position.x = req_vx;
//			  cmd_pose.pose.position.y = req_vy;
//			  cmd_pose.pose.position.z = req_vz;
			  // To avoid singularities
			  cmd_pose.pose.position.x = 0.0;
			  cmd_pose.pose.position.y = 0.0;
			  cmd_pose.pose.position.z = 0.0;
			  cmd_pose.pose.orientation = req_q;
			  cmd_pose.header.stamp=ros::Time::now();
    		  // Publish cmd_pose
			  pose_pub_.publish(cmd_pose);
			  // Publish haptic feedback
			  if(feedback_==false){
			        haptic_feedback.x = 0.0;
			        haptic_feedback.y = 0.0;
			        haptic_feedback.z = 0.0;
			  }
			  haptic_pub_.publish(haptic_feedback);
			  ROS_DEBUG("baxter_teleop::falcon (orientation mode): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);
		  }
      }else if(control_=="velocity"){
          if(pos_){
        	  // Copy linear and angular velocities to twist message
        	  cmd_vel.twist.linear.x = req_vx;
        	  cmd_vel.twist.linear.y = req_vy;
        	  cmd_vel.twist.linear.z = req_vz;
        	  cmd_vel.twist.angular.x = req_wx;
        	  cmd_vel.twist.angular.y = req_wy;
        	  cmd_vel.twist.angular.z = req_wz;
    		  // Publish cmd_vel
    		  cmd_vel.header.stamp=ros::Time::now();
    		  vel_pub_.publish(cmd_vel);
    		  ROS_INFO("baxter_teleop::falcon (linear velocity mode): vx %f, vy %f, vz %f, wx %f, wy %f, wz %f", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z, cmd_vel.twist.angular.x, cmd_vel.twist.angular.y, cmd_vel.twist.angular.z);
          }else if(ang_){
        	  // Copy linear and angular velocities to twist message
        	  cmd_vel.twist.linear.x = req_vx;
        	  cmd_vel.twist.linear.y = req_vy;
        	  cmd_vel.twist.linear.z = req_vz;
        	  cmd_vel.twist.angular.x = req_wx;
        	  cmd_vel.twist.angular.y = req_wy;
        	  cmd_vel.twist.angular.z = req_wz;
    		  // Publish cmd_vel
    		  cmd_vel.header.stamp=ros::Time::now();
    		  vel_pub_.publish(cmd_vel);
    		  ROS_INFO("baxter_teleop::falcon (angular velocity mode): vx %f, vy %f, vz %f, wx %f, wy %f, wz %f", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z, cmd_vel.twist.angular.x, cmd_vel.twist.angular.y, cmd_vel.twist.angular.z);
         }
      }
    }
    else
    {
      // Publish zero commands if deadman_no_publish is false
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.linear.z = 0;
      cmd_vel.twist.angular.x = cmd_vel.twist.angular.y = cmd_vel.twist.angular.z = 0;
      if (!deadman_no_publish_)
      {
        // Publish cmd_vel
        cmd_vel.header.stamp=ros::Time::now();
        vel_pub_.publish(cmd_vel);
      }
    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }

};

int main(int argc, char **argv)
{
  // Initializing the roscpp Node
  ros::init(argc, argv, "baxter_falcon");

  // Starting the roscpp Node
  ros::NodeHandle n;
//  ros::NodeHandle n("~");

  //***********************************************

  const char* opt_no_publish = "--deadman_no_publish";

  bool no_publish = false;
  for(int i=1;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
      no_publish = true;
  }

  TeleopJoy teleop_joy(no_publish);
  teleop_joy.init();

  //***********************************************

  ros::Rate pub_rate(JOYSTICK_HZ);

  while (teleop_joy.n_.ok())
  {
    teleop_joy.send_cmd();
    ros::spinOnce();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}

