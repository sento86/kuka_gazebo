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
///\brief Converts imu commands on /imu to twist commands on /cmd_vel.

//***********************************************

#define DEFAULT_MAX_LEVEL           1.0
#define NUM_AXIS                    4
#define NUM_BUTTONS                 4
#define JOYSTICK_HZ                 40.0    //Desired frequency
#define JOYSTICK_TIMER              -1.0    //Time (secs) max. without reading (Timeout for joystick reset (<0 no timeout))

//***********************************************

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include "filters/transfer_function.h"
using namespace filters;

//***********************************************

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

//***********************************************

using namespace std;

//***********************************************

class TeleopJoy
{
  public:
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::TwistStamped cmd_vel_stmp;
  //geometry_msgs::Pose cmd_pose;
  //geometry_msgs::PoseStamped cmd_pose_stmp;
  geometry_msgs::Twist cmd_pose;
  geometry_msgs::TwistStamped cmd_pose_stmp;
  //joy::Joy joy;
  double req_vx, req_vy, req_vz, req_wx, req_wy, req_wz;
  double sign_vx, sign_vy, sign_vz, sign_wx, sign_wy, sign_wz;
  double max_vx, max_vy, max_vz, max_wx, max_wy, max_wz;
  double max_dx, max_dy, max_dz, max_dR, max_dP, max_dY;
  int axis_vx, axis_vy, axis_vz, axis_wx, axis_wy, axis_wz;
  int pos_button, ang_button, lin_vel_button, ang_vel_button;
  bool pos_, ang_, lin_vel_, ang_vel_;
  bool deadman_no_publish_, deadman_, last_deadman_;
  std::string last_selected_topic_;
  std::string cmd_vel_topic_, cmd_vel_stmp_topic_, cmd_pose_topic_, cmd_pose_stmp_topic_;

  sensor_msgs::Imu last_processed_joy_message_;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_, vel_stmp_pub_, pose_pub_, pose_stmp_pub_;
  ros::Subscriber joy_sub_;

  // Message filters
  ros::Time time_now, time_past;
  double dt;
  double vx, vy, vz;
  double ax, ay, az;
  double epsilon;
  double a;
  std::vector<double> ax_vec, ay_vec, az_vec;

  FilterBase<double > * filter_ax = new SingleChannelTransferFunctionFilter<double> ();
  FilterBase<double > * filter_ay = new SingleChannelTransferFunctionFilter<double> ();
  FilterBase<double > * filter_az = new SingleChannelTransferFunctionFilter<double> ();

  TeleopJoy(bool deadman_no_publish = false) :
	sign_vx(1.0), sign_vy(1.0), sign_vz(1.0),
	sign_wx(1.0), sign_wy(1.0), sign_wz(1.0),
    max_vx(0.1), max_vy(0.1), max_vz(0.1),
    max_wx(0.5), max_wy(0.5), max_wz(0.5),
    max_dx(0.1), max_dy(0.1), max_dz(0.1),
    max_dR(0.5), max_dP(0.5), max_dY(0.5),
	deadman_no_publish_(deadman_no_publish),
    deadman_(false), last_deadman_(false),
    pos_(false), ang_(false),
    lin_vel_(false), ang_vel_(false),
    n_private_("~")
  { }

  void init()
  {
	cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;
	cmd_vel_stmp.twist = cmd_vel;
	//cmd_pose.position.x = cmd_pose.position.y = cmd_pose.position.z = 0.0;
	//cmd_pose.orientation.w = cmd_pose.orientation.x = cmd_pose.orientation.y = cmd_pose.orientation.z = 0.0;
	//cmd_pose_stmp.pose = cmd_pose;
	cmd_pose.linear.x = cmd_pose.linear.y = cmd_pose.linear.z = 0.0;
	cmd_pose.angular.x = cmd_pose.angular.y = cmd_pose.angular.z = 0.0;
	cmd_pose_stmp.twist = cmd_pose;

	// Set messages topics
	n_private_.param("cmd_vel_topic", cmd_vel_topic_, std::string("cmd_vel"));
	n_private_.param("cmd_vel_stmp_topic", cmd_vel_stmp_topic_, std::string("cmd_vel/stamped"));
	n_private_.param("cmd_pose_topic", cmd_pose_topic_, std::string("cmd_pose"));
	n_private_.param("cmd_pose_stmp_topic", cmd_pose_stmp_topic_, std::string("cmd_pose/stamped"));
	n_private_.param("deadman_no_publish", deadman_no_publish_, deadman_no_publish_);

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

	vel_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
	vel_stmp_pub_ = n_.advertise<geometry_msgs::TwistStamped>(cmd_vel_stmp_topic_, 1);
	//pose_pub_ = n_.advertise<geometry_msgs::Pose>(cmd_pose_topic_, 1);
	//pose_stmp_pub_ = n_.advertise<geometry_msgs::PoseStamped>(cmd_pose_stmp_topic_, 1);
	pose_pub_ = n_.advertise<geometry_msgs::Twist>(cmd_pose_topic_, 1);
	pose_stmp_pub_ = n_.advertise<geometry_msgs::TwistStamped>(cmd_pose_stmp_topic_, 1);

	joy_sub_ = n_.subscribe("imu", 10, &TeleopJoy::joy_cb, this);

	// Message filters
	dt = 0.0;
    vx = 0.0; vy = 0.0; vz = 0.0;
    ax = 0.0; ay = 0.0; az = 0.0;
    epsilon = 1e-6;
    double h = 1.0/JOYSTICK_HZ; // Time-step
    double Tf = 0.1; // Filter time-constant
    a = h/(h+Tf);
    filter_ax->configure("LowPassSingle", n_private_);
    filter_ay->configure("LowPassSingle", n_private_);
    filter_az->configure("LowPassSingle", n_private_);

    time_past = time_past;
  }

  ~TeleopJoy() { }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Imu::ConstPtr& joy_msg)
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

    deadman_ = true;

//    if (!deadman_)
//      return;

    // Update axis values
//    req_vx = joy_msg->linear_acceleration.x * max_vx * sign_vx;
//    req_vy = joy_msg->linear_acceleration.y * max_vy * sign_vy;
//    req_vz = joy_msg->linear_acceleration.z * max_vz * sign_vz;
//    req_wx = joy_msg->angular_velocity.x * max_dR * sign_wx;
//    req_wy = joy_msg->angular_velocity.y * max_dP * sign_wy;
//    req_wz = joy_msg->angular_velocity.z * max_dY * sign_wz;

//    filter_ax->update(joy_msg->linear_acceleration.x, ax);
//    filter_ay->update(joy_msg->linear_acceleration.y, ay);
//    filter_az->update(joy_msg->linear_acceleration.z, az);
//    ax = joy_msg->linear_acceleration.x;
//    ay = joy_msg->linear_acceleration.y;
//    az = joy_msg->linear_acceleration.z;
    ax = ax*(1-a) + a*joy_msg->linear_acceleration.x;
    ay = ay*(1-a) + a*joy_msg->linear_acceleration.y;
    az = az*(1-a) + a*joy_msg->linear_acceleration.z;

    tf::Vector3 g(0.0, 0.0, -9.81); // Gravity
    tf::Vector3 a(ax, ay, az); // Acceleration
    tf::Quaternion q;
    tf::quaternionMsgToTF(joy_msg->orientation, q);
    tf::Matrix3x3 R(q);
    tf::Vector3 a_g = R*a + g;

    time_now = ros::Time::now();
    dt = (time_now-time_past).toSec();
    if(fabs(a_g.x())>0.2)
    	vx = vx + a_g.x()*dt;
    else
    	vx = 0.8*vx;
    if(fabs(a_g.y())>0.2)
    	vy = vy + a_g.y()*dt;
    else
        vy = 0.8*vy;
    if(fabs(a_g.z())>0.2)
    	vz = vz + a_g.z()*dt;
    else
    	vz = 0.8*vz;
    time_past = time_now;

    ROS_ERROR_STREAM(dt << " " << a_g.x() << " " << a_g.y() << " " << a_g.z() << " " );

    req_vx = vx * sign_vx;
    req_vy = vy * sign_vy;
    req_vz = vz * sign_vz;
    req_wx = joy_msg->angular_velocity.x * sign_wx;
    req_wy = joy_msg->angular_velocity.y * sign_wy;
    req_wz = joy_msg->angular_velocity.z * sign_wz;

    // Set a deadzone to get rid of noise
    if(fabs(req_vx)<1e-2)
    	req_vx = 0.0;
    if(fabs(req_vy)<1e-2)
    	req_vy = 0.0;
    if(fabs(req_vz)<1e-2)
    	req_vz = 0.0;
    if(fabs(req_wx)<1e-2)
    	req_wx = 0.0;
    if(fabs(req_wy)<1e-2)
    	req_wy = 0.0;
    if(fabs(req_wz)<1e-2)
    	req_wz = 0.0;

    // Enforce max/mins for velocity
    // Joystick should be [-1, 1], but it might not be
	req_vx = max(min(req_vx, max_vx), -max_vx);
	req_vy = max(min(req_vy, max_vy), -max_vy);
	req_vz = max(min(req_vz, max_vz), -max_vz);
	req_wx = max(min(req_wx, max_dR), -max_dR);
	req_wy = max(min(req_wy, max_dP), -max_dP);
	req_wz = max(min(req_wz, max_dY), -max_dY);
  }


  void send_cmd()
  {
    if(deadman_ && last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
      // Copy linear and angular velocities to twist message
    	  cmd_vel.linear.x = req_vx;
		  cmd_vel.linear.y = req_vy;
    	  cmd_vel.linear.z = req_vz;
          cmd_vel.angular.x = req_wx;
          cmd_vel.angular.y = req_wy;
          cmd_vel.angular.z = req_wz;
		  // Publish cmd_vel
		  vel_pub_.publish(cmd_vel);
		  // Publish cmd_vel_stmp with time stamp
		  cmd_vel_stmp.twist = cmd_vel;
		  cmd_vel_stmp.header.stamp=ros::Time::now();
		  vel_stmp_pub_.publish(cmd_vel_stmp);

//          //cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0;
//          //cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;
//          /*cmd_pose.position.x = 0.0;
//          cmd_pose.position.y = 0.0;
//          cmd_pose.position.z = 0.0;
//          tf::Quaternion q;
//          q.setEuler(req_wz,req_wy,req_wx);
//          tf::quaternionTFToMsg(q, cmd_pose.orientation);*/
//          cmd_pose.linear.x = 0.0;
//          cmd_pose.linear.y = 0.0;
//          cmd_pose.linear.z = 0.0;
//          cmd_pose.angular.x = req_wx;
//          cmd_pose.angular.y = req_wy;
//          cmd_pose.angular.z = req_wz;
//		  // Publish cmd_vel
//		  pose_pub_.publish(cmd_pose);
//		  // Publish cmd_pose_stmp with time stamp
//		  //cmd_pose_stmp.pose = cmd_pose;
//		  cmd_pose_stmp.twist = cmd_pose;
//		  cmd_pose_stmp.header.stamp=ros::Time::now();
//		  pose_stmp_pub_.publish(cmd_pose_stmp);
//		  ROS_INFO("baxter_teleop::xsens mode %d, dx %f, dy %f, dz %f, dR %f, dP %f, dY %f", 0, req_vx, req_vy, req_vz, req_wx, req_wy, req_wz);

    }
    else
    {
      // Publish zero commands if deadman_no_publish is false
      cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0;
      cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;
      if (!deadman_no_publish_)
      {
        // Publish cmd_vel
        vel_pub_.publish(cmd_vel);
        // Publish cmd_vel_stmp with time stamp
        cmd_vel_stmp.twist = cmd_vel;
        cmd_vel_stmp.header.stamp=ros::Time::now();
        vel_stmp_pub_.publish(cmd_vel_stmp);
      }
    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }

};

int main(int argc, char **argv)
{
  // Initializing the roscpp Node
  ros::init(argc, argv, "baxter_xsens");

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

