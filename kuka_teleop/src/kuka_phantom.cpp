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
#define NUM_AXIS                    6
#define NUM_BUTTONS                 2
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

//***********************************************

#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "baxter_core_msgs/EndpointState.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
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
  int pos_button, ang_button, lin_vel_button, ang_vel_button;
  int mode, num_modes, num_joints;
  bool lock_, sim_, feedback_, gripper_, last_gripper_;
  bool deadman_no_publish_, deadman_, last_deadman_;
  unsigned int req_gripper;
  std::string limb_, control_;
  std::string last_selected_topic_;
  std::string cmd_vel_topic_, cmd_pose_topic_, cmd_joint_topic_, cmd_gripper_topic_;
  std::string haptic_topic_, lock_topic_;
  omni_msgs::OmniFeedback haptic_feedback;
  omni_msgs::OmniButtonEvent haptic_button;
  std_msgs::Bool haptic_lock;
  geometry_msgs::WrenchStamped wrench;
  baxter_core_msgs::EndpointState endpoint;

  tf::Quaternion q;
  tf::Matrix3x3 Ree; // Rotation matrix of End Effector
  double sign_limb;
  bool init_pose;

  sensor_msgs::JointState last_processed_joint_message_;
  omni_msgs::OmniState last_processed_joy_message_;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_, pose_pub_, joint_pub_, gripper_pub_;
  ros::Publisher haptic_pub_, lock_pub_;
  ros::Subscriber joy_sub_, button_sub_, joint_sub_, wrench_sub_, endpoint_sub_;

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
    n_private_.param("haptic_topic", haptic_topic_, std::string("phantom/force_feedback"));
    n_private_.param("lock_topic", lock_topic_, std::string("phantom/lock"));
    n_private_.param("deadman_no_publish", deadman_no_publish_, deadman_no_publish_);
    n_private_.param("limb", limb_, std::string("right"));
    n_private_.param("sim", sim_, false);
	n_private_.param("control", control_, std::string("joint"));
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
    n_private_.param("lin_vel_button", lin_vel_button, 2);
    n_private_.param("ang_vel_button", ang_vel_button, 1);

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
    ROS_DEBUG("lin_vel_button: %d", lin_vel_button);
    ROS_DEBUG("ang_vel_button: %d", ang_vel_button);
    ROS_DEBUG("joy_msg_timeout: %f", joy_msg_timeout);

    vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic_, 1);
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>(cmd_pose_topic_, 1);
    joint_pub_ = n_.advertise<sensor_msgs::JointState>(cmd_joint_topic_, 1);
    gripper_pub_ = n_.advertise<std_msgs::Header>(cmd_gripper_topic_, 1);

    haptic_pub_ = n_.advertise<omni_msgs::OmniFeedback>(haptic_topic_, 1);
    lock_pub_ = n_.advertise<std_msgs::Bool>(lock_topic_, 1);

    joy_sub_ = n_.subscribe("phantom/state", 10, &TeleopJoy::joy_cb, this);
    button_sub_ = n_.subscribe("phantom/button", 10, &TeleopJoy::button_cb, this);
    joint_sub_ = n_.subscribe("phantom/joint_states", 10, &TeleopJoy::joint_cb, this);
    wrench_sub_ = n_.subscribe(limb_+"/ft_sensor", 10, &TeleopJoy::wrench_cb, this);
    endpoint_sub_ = n_.subscribe("robot/limb/"+limb_+"/endpoint_state", 10, &TeleopJoy::endpoint_cb, this);

	if(limb_=="right")
		sign_limb = 1.0;
	else if(limb_=="left")
		sign_limb = -1.0;
	else
		sign_limb = 0.0;

	init_pose = true;
  }

  ~TeleopJoy() { }

  /** Callback for phantom omni topic **/
  void joy_cb(const omni_msgs::OmniState::ConstPtr& joy_msg)
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

    lock_ = joy_msg->locked;
    deadman_ = joy_msg->deadman;

    if (!deadman_)
      return;

    // Update Cartesian references
    if(control_=="cartesian"){
		// Cartesian space control enabled
//    	// Version from paper "Teleoperation of Humanoid Baxter Robot Using Haptic Feedback"
//    	double Sx = 3.8; // Divide by 1000 if units in mm
//    	double Sy = 3.8;
//    	double Sz = 3.8;
//    	double Tx = 0.701;
//    	double Ty = -0.210;
//    	double Tz = 0.129;
//    	double xs = Tx - Sx*joy_msg->pose.position.y;
//    	double ys = Ty + Sy*joy_msg->pose.position.x;
//    	double zs = Tz + Sz*joy_msg->pose.position.z;
//    	double delta = -M_PI/4.0;
//		req_vx = cos(delta)*xs - sin(delta)*ys;
//		req_vy = sin(delta)*xs + cos(delta)*ys;
//		req_vz = zs;
		// Own version
    	double Sx = 2.5; // Divide by 1000 if units in mm
    	double Sy = 2.5;
    	double Sz = 2.5;
    	double Tx =  0.062966;
    	double Ty = -0.259458 * sign_limb; // Sign depends on the arm to be controlled (left -, right +)
    	double Tz =  0.119732;
    	double xs = -Sx*(joy_msg->pose.position.y-0.08)+0.3;
    	double ys =  Sy*joy_msg->pose.position.x;
    	double zs =  Sz*joy_msg->pose.position.z;
//		req_vx = Tx + cos(delta)*xs - sin(delta)*ys;
//		req_vy = Ty + sin(delta)*xs + cos(delta)*ys;
//		req_vz = Tz + zs;

		tf::Vector3 X(xs,ys,zs);
		tf::Vector3 T(Tx,Ty,Tz);
		double length = 0.0; // Length of Baxter's last link
		tf::Vector3 E(0,0,length);
    	double angle = 0.0;//-M_PI/4.0;// Angular offset of Baxter's first joint // Sign depends on the arm to be controlled (left -, right +)
		tf::Matrix3x3 Rz;
		Rz.setEulerYPR(0,0,angle);
		tf::Vector3 V = Rz*X+T + Ree*E;
		req_vx = V.x();
		req_vy = V.y();
		req_vz = V.z();

    }else{
		// Cartesian space control disabled
		req_vx = 0.0;
		req_vy = 0.0;
		req_vz = 0.0;
		req_wx = 0.0;
		req_wy = 0.0;
		req_wz = 0.0;
    }
  }

  /** Callback for phantom omni button event topic **/
  void button_cb(const omni_msgs::OmniButtonEvent::ConstPtr& button_msg)
  {
    haptic_button = *button_msg;
    if(init_pose==true)
    	init_pose=false;
    if(haptic_button.white_button==true){ // Unlock device when deadman button pressed (white)
        haptic_lock.data = false;
        lock_pub_.publish(haptic_lock);
    }else{ // Lock device when deadman button not pressed (white)
        haptic_lock.data = true;
        lock_pub_.publish(haptic_lock);
        if(haptic_button.grey_button==true){ // Gripper button (grey) pressed to open/close when white button is also pressed
            mode++;
            if(mode>=num_modes)    mode=0;
            if(mode==1)
              req_gripper = 1;
            else
              req_gripper = 0;
            cmd_gripper.stamp = ros::Time::now();
            cmd_gripper.seq = req_gripper;
            cmd_gripper.frame_id = "base";
            gripper_pub_.publish(cmd_gripper);
        }else{ // Gripper button (grey) not pressed

            // Save current position to lock haptic device
            haptic_feedback.position.x = last_processed_joy_message_.pose.position.x;
            haptic_feedback.position.y = last_processed_joy_message_.pose.position.y;
            haptic_feedback.position.z = last_processed_joy_message_.pose.position.z;
        }
    }
  }

  /** Callback for phantom omni joint states topic **/
  void joint_cb(const sensor_msgs::JointState::ConstPtr& joint_msg)
  {
    // Do not process the same message twice.
    if(joint_msg->header.stamp == last_processed_joint_message_.header.stamp) {
        // notify the user only if the problem persists
        if(ros::Time::now() - joint_msg->header.stamp > ros::Duration(5.0/JOYSTICK_HZ))
            ROS_WARN_THROTTLE(1.0, "Received Joint message with same timestamp multiple times. Ignoring subsequent messages.");
        deadman_ = false;
        return;
    }
    last_processed_joint_message_ = *joint_msg;

    // Update Cartesian references
    if(control_=="joint"){
		// Joint space control enabled
//		req_q0 =  joint_msg->position[0];                // s0
		req_q0 =  joint_msg->position[0] + sign_limb*M_PI/4.0; // s0 (Initial Offset of 45 degrees)
		req_q1 = -joint_msg->position[1];                // s1 (joint with opposite sign and offset)
		req_q2 =  0.0;                                   // e0
		req_q3 = -joint_msg->position[2] + M_PI/2.0;     // e1 (joint with opposite sign and offset)
		req_q4 =  joint_msg->position[3];// - M_PI*2.0;     // w0 (joint with offset)
		req_q5 = -joint_msg->position[4] + 2.0 + 0.6;    // w1 (joint with opposite sign and offset)
		req_q6 =  joint_msg->position[5];                // w2
    }else{
//		// Joint space control disabled
//		req_q0 = 0.0; // s0
//		req_q1 = 0.0; // s1
//		req_q2 = 0.0; // e0
//		req_q3 = 0.0; // e1
//		req_q4 = 0.0; // w0
//		req_q5 = 0.0; // w1
//		req_q6 = 0.0; // w2
//		req_q0 =  joint_msg->position[0];                // s0
		req_q0 =  joint_msg->position[0] + sign_limb*M_PI/4.0; // s0 (Initial Offset of 45 degrees)
		req_q1 = -joint_msg->position[1];                // s1 (joint with opposite sign and offset)
		req_q2 =  0.0;                                   // e0
		req_q3 = -joint_msg->position[2] + M_PI/2.0;     // e1 (joint with opposite sign and offset)
		req_q4 =  joint_msg->position[3];// - M_PI*2.0;  // w0 (joint with offset)
		req_q5 = -joint_msg->position[4] + 2.0 + 0.6;    // w1 (joint with opposite sign and offset)
		req_q6 =  joint_msg->position[5];                // w2

		tf::Matrix3x3 RM, R0, R1, R2, R3, R4, R5, R6, Ry;

		R0.setEulerYPR(req_q0-sign_limb*M_PI/4,0,0);
		R1.setEulerYPR(0,-req_q1,0);
		R2.setEulerYPR(0,0,req_q2);
		R3.setEulerYPR(0,-req_q3,0);
		R4.setEulerYPR(0,0,req_q4);
		R5.setEulerYPR(0,-req_q5,0);
		R6.setEulerYPR(0,0,req_q6);
		Ry.setEulerYPR(0,-M_PI/2,0);
		RM = Ry*R6*R5*R4*R3*R2*R1*R0;

		Ree = RM;

		RM.getRPY(req_wx, req_wy, req_wz);
		req_wx = -req_wx;
		req_wy = -req_wy;
		req_wz = -req_wz;
		RM.setRPY(req_wx, req_wy, req_wz);
		RM.getRotation(q);
		tf::quaternionTFToMsg(q,req_q);

    }
  }

  /** Callback for Baxter's endpoint topic **/
  void endpoint_cb(const baxter_core_msgs::EndpointState::ConstPtr& endpoint_msg)
  {
      endpoint = *endpoint_msg;
  }

  /** Callback for force/torque sensor topic **/
  void wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
  {
    wrench = *wrench_msg;

    //if(haptic_button.white_button==true){
    if(deadman_==true){

        double scale;
        tf::Vector3 force_global;
        if(sim_){
            scale = 0.1; // Sim
            force_global.setValue(wrench.wrench.force.y*scale, wrench.wrench.force.x*scale, -wrench.wrench.force.z*scale); // Sim
        }else{
        	scale = 0.5; // Real
        	force_global.setValue(wrench.wrench.force.y*scale, -wrench.wrench.force.x*scale, wrench.wrench.force.z*scale); // Real
        }

//        double dt = 0.01; // Sampling time
//        double RC = 0.04; // Time constant (Real)
//        if(sim_)
//            RC = 0.1; // Time constant (Sim)
//        double alpha = dt/(RC+dt);
//        haptic_feedback.force.x = alpha*force_global.x() + (1.0-alpha)*haptic_feedback.force.x;
//        haptic_feedback.force.y = alpha*force_global.y() + (1.0-alpha)*haptic_feedback.force.y;
//        haptic_feedback.force.z = alpha*force_global.z() + (1.0-alpha)*haptic_feedback.force.z;
        haptic_feedback.force.x = force_global.x();
        haptic_feedback.force.y = force_global.y();
        haptic_feedback.force.z = force_global.z();

        // Dead zone correction
        double dead_zone = 0.5;
        if(fabs(haptic_feedback.force.x)<dead_zone)
            haptic_feedback.force.x=0.0;
        else if(haptic_feedback.force.x>=dead_zone)
            haptic_feedback.force.x=haptic_feedback.force.x-dead_zone;
        else if(haptic_feedback.force.x<=-dead_zone)
            haptic_feedback.force.x=haptic_feedback.force.x+dead_zone;
        if(fabs(haptic_feedback.force.y)<dead_zone)
            haptic_feedback.force.y=0.0;
        else if(haptic_feedback.force.y>=dead_zone)
            haptic_feedback.force.y=haptic_feedback.force.y-dead_zone;
        else if(haptic_feedback.force.y<=-dead_zone)
            haptic_feedback.force.y=haptic_feedback.force.y+dead_zone;
        if(fabs(haptic_feedback.force.z)<dead_zone)
            haptic_feedback.force.z=0.0;
        else if(haptic_feedback.force.z>=dead_zone)
            haptic_feedback.force.z=haptic_feedback.force.z-dead_zone;
        else if(haptic_feedback.force.z<=-dead_zone)
            haptic_feedback.force.z=haptic_feedback.force.z+dead_zone;
        // Saturation
        double max_force = 20.0;
        if(haptic_feedback.force.x>max_force)
            haptic_feedback.force.x=max_force;
        else if(haptic_feedback.force.x<-max_force)
            haptic_feedback.force.x=-max_force;
        if(haptic_feedback.force.y>max_force)
            haptic_feedback.force.y=max_force;
        else if(haptic_feedback.force.y<-max_force)
            haptic_feedback.force.y=-max_force;
        if(haptic_feedback.force.z>max_force)
            haptic_feedback.force.z=max_force;
        else if(haptic_feedback.force.z<-max_force)
            haptic_feedback.force.z=-max_force;
        ROS_DEBUG("feedback: %f %f %f", haptic_feedback.force.x, haptic_feedback.force.y, haptic_feedback.force.z);
    }
  }


  void send_cmd()
  {
    if(deadman_ && last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
    	if(control_=="joint"){
      	  // Copy angles to joint message
		  cmd_joint.position[0]=req_q0;
		  cmd_joint.position[1]=req_q1;
		  cmd_joint.position[2]=req_q2;
		  cmd_joint.position[3]=req_q3;
		  cmd_joint.position[4]=req_q4;
		  cmd_joint.position[5]=req_q5;
		  cmd_joint.position[6]=req_q6;
		  // Publish cmd_joint
		  cmd_joint.header.stamp=ros::Time::now();
		  joint_pub_.publish(cmd_joint);
	      // Publish haptic feedback
		  if(feedback_==false){
		        haptic_feedback.force.x = 0.0;
		        haptic_feedback.force.y = 0.0;
		        haptic_feedback.force.z = 0.0;
		  }
	      haptic_pub_.publish(haptic_feedback);
	      ROS_DEBUG("baxter_teleop::phantom (joint space control): q0 %f, q1 %f, q2 %f, q3 %f, q4 %f, q5 %f, q6 %f", cmd_joint.position[0], cmd_joint.position[1], cmd_joint.position[2], cmd_joint.position[3], cmd_joint.position[4], cmd_joint.position[5], cmd_joint.position[6]);

    	}else if(control_=="cartesian"){
    	  cmd_pose.pose.position.x = req_vx;
    	  cmd_pose.pose.position.y = req_vy;
    	  cmd_pose.pose.position.z = req_vz;
    	  cmd_pose.pose.orientation = req_q;
		  // Publish cmd_pose
    	  cmd_pose.header.stamp=ros::Time::now();
    	  pose_pub_.publish(cmd_pose);
		  // Publish haptic feedback
		  if(feedback_==false){
		        haptic_feedback.force.x = 0.0;
		        haptic_feedback.force.y = 0.0;
		        haptic_feedback.force.z = 0.0;
		  }
		  haptic_pub_.publish(haptic_feedback);
		  ROS_DEBUG("baxter_teleop::phantom (Cartesian space control): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);
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
      // Set initial position until one button is pressed
      if(init_pose==true){
		haptic_lock.data = true;
		lock_pub_.publish(haptic_lock);

		// Initial haptic state
		haptic_feedback.position.x = -0.118;
		haptic_feedback.position.y = 0.056;
		haptic_feedback.position.z = 0.0;
		haptic_feedback.force.x = 0.0;
		haptic_feedback.force.y = 0.0;
		haptic_feedback.force.z = 0.0;

		// Publish haptic feedback
		haptic_pub_.publish(haptic_feedback);
      }
    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }

};

int main(int argc, char **argv)
{
  // Initializing the roscpp Node
  ros::init(argc, argv, "baxter_phantom");

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
