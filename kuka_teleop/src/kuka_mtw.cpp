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
#define NUM_AXIS                    7
#define NUM_BUTTONS                 2
#define JOYSTICK_HZ                 100.0  //Desired frequency
#define JOYSTICK_TIMER             -1.0    //Time (secs) max. without reading (Timeout for joystick reset (<0 no timeout))

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
#include <tf/transform_listener.h>

//***********************************************

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"

//#include <mtw_driver/QuaternionStampedMultiarray.h>
#include <mtw_driver/TransformStampedMultiarray.h>
//#include <tf2_msgs/TFMessage.h>

//***********************************************

using namespace std;

//***********************************************

class TeleopJoy
{
  public:
  geometry_msgs::TwistStamped cmd_vel;
  geometry_msgs::PoseStamped cmd_pose;
  sensor_msgs::JointState cmd_joint, joint_states;
  std_msgs::Header cmd_gripper;
  tf2_msgs::TFMessage msg_tf;
  mtw_driver::TransformStampedMultiarray msg_mtw;
  //joy::Joy joy;
  geometry_msgs::Quaternion req_q;
  double req_q0, req_q1, req_q2, req_q3, req_q4, req_q5, req_q6;
  double req_vx, req_vy, req_vz, req_wx, req_wy, req_wz;
  int mode, num_modes, num_joints;
  bool pos_, ang_, lin_vel_, ang_vel_;
  bool deadman_no_publish_, deadman_, last_deadman_;
  unsigned int req_gripper;
  std::string limb_, control_, body_;
  std::string last_selected_topic_;
  std::string cmd_vel_topic_, cmd_pose_topic_, cmd_joint_topic_, cmd_gripper_topic_, joint_states_topic_;

  tf::Quaternion q;

  sensor_msgs::Imu last_processed_joy_message_;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_, pose_pub_, pose_joint_pub_, gripper_pub_, joint_states_pub_;
  ros::Subscriber joy_sub_, tf_sub_, mtw_sub_;

//  // Message filters
//  ros::Time time_now, time_past;
//  double dt;
//  double vx, vy, vz;
//  double ax, ay, az;
//  double epsilon;
//  double a;
//  std::vector<double> ax_vec, ay_vec, az_vec;
//
//  FilterBase<double > * filter_ax = new SingleChannelTransferFunctionFilter<double> ();
//  FilterBase<double > * filter_ay = new SingleChannelTransferFunctionFilter<double> ();
//  FilterBase<double > * filter_az = new SingleChannelTransferFunctionFilter<double> ();

  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::string ns;


  TeleopJoy(bool deadman_no_publish = false) :
    mode(0), num_modes(3), num_joints(7),
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
	n_private_.param("joint_states_topic", joint_states_topic_, std::string("/joint_states"));
	n_private_.param("deadman_no_publish", deadman_no_publish_, deadman_no_publish_);
	n_private_.param("limb", limb_, limb_);
	n_private_.param("control", control_, std::string("joint"));
	n_private_.param("body", body_, std::string("human"));

	n_private_.param("num_modes", num_modes, num_modes);
	n_private_.param("num_joints", num_joints, num_joints);

	if(limb_=="head"){

		cmd_joint.name.resize(2);
		cmd_joint.position.resize(2);
		//cmd_joint.velocity.resize(2);
		//cmd_joint.effort.resize(2);
		for(int i=0; i<2; i++){
			std::string joint_name = "q"+std::to_string(i);
			cmd_joint.name[i]=joint_name;
			cmd_joint.position[i]=0.0;
		}
		joint_states = cmd_joint;
		joint_states.name[0] = limb_+"_pan";
		joint_states.name[1] = limb_+"_nod";

	}else{
		cmd_joint.name.resize(num_joints);
		cmd_joint.position.resize(num_joints);
		//cmd_joint.velocity.resize(num_joints);
		//cmd_joint.effort.resize(num_joints);
		for(int i=0; i<num_joints; i++){
			std::string joint_name = "q"+std::to_string(i);
			cmd_joint.name[i]=joint_name;
			cmd_joint.position[i]=0.0;
		}

		joint_states = cmd_joint;
		joint_states.name[0] = limb_+"_s0";
		joint_states.name[1] = limb_+"_s1";
		joint_states.name[2] = limb_+"_e0";
		joint_states.name[3] = limb_+"_e1";
		joint_states.name[4] = limb_+"_w0";
		joint_states.name[5] = limb_+"_w1";
		joint_states.name[6] = limb_+"_w2";

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

	vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic_, 1);
	pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>(cmd_pose_topic_, 1);
	pose_joint_pub_ = n_.advertise<sensor_msgs::JointState>(cmd_joint_topic_, 1);
	gripper_pub_ = n_.advertise<std_msgs::Header>(cmd_gripper_topic_, 1);
	joint_states_pub_ = n_.advertise<sensor_msgs::JointState>(joint_states_topic_, 1);

//	joy_sub_ = n_.subscribe("imu", 10, &TeleopJoy::joy_cb, this);
//	tf_sub_ = n_.subscribe("tf", 1, &TeleopJoy::tf_cb, this);
	mtw_sub_ = n_.subscribe("transforms", 1, &TeleopJoy::mtw_cb, this);

//	// Message filters
//	dt = 0.0;
//    vx = 0.0; vy = 0.0; vz = 0.0;
//    ax = 0.0; ay = 0.0; az = 0.0;
//    epsilon = 1e-6;
//    double h = 1.0/JOYSTICK_HZ; // Time-step
//    double Tf = 0.1; // Filter time-constant
//    a = h/(h+Tf);
//    filter_ax->configure("LowPassSingle", n_private_);
//    filter_ay->configure("LowPassSingle", n_private_);
//    filter_az->configure("LowPassSingle", n_private_);

  }

  ~TeleopJoy() { }

//  /** Callback for tf topic **/
//  void tf_cb(const tf2_msgs::TFMessage::ConstPtr& tf_msg)
//  {
//	  msg_tf = *tf_msg;
//
//		//listener.lookupTransform("/"+body_+"/base", "/"+body_+"/"+limb_+"_gripper",
//		//ros::Time(0), transform);
//
//		for(int i=0; i<msg_tf.transforms.size(); i++){
//			if(msg_tf.transforms[i].child_frame_id==std::string(body_+"/"+limb_+"_grip")){
//			ROS_ERROR_STREAM(msg_tf.transforms[i].child_frame_id);
//				tf::Transform tranform_stmp;
//				tf::transformMsgToTF(msg_tf.transforms[i].transform,tranform_stmp);
//				tf::StampedTransform transform_(tranform_stmp,msg_tf.transforms[i].header.stamp,std::string("/"+body_+"/base"),std::string("/"+body_+"/"+limb_+"_gripper"));
//
//				req_vx = transform_.getOrigin().x();
//				req_vy = transform_.getOrigin().y();
//				req_vz = transform_.getOrigin().z();
//				//tf::Matrix3x3 M(transform.getRotation());
//				//M.getEulerYPR(req_wz, req_wy, req_wx);
//				q = transform_.getRotation();
//				tf::quaternionTFToMsg(q, req_q);
//				break;
//			}
//		}
//  }

  static void toEulerAngle(const tf::Quaternion& q, double& roll, double& pitch, double& yaw)
  {
  	// roll (x-axis rotation)
  	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
  	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  	roll = atan2(sinr, cosr);

  	// pitch (y-axis rotation)
  	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  	if (fabs(sinp) >= 1){
  		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  	  	//roll = roll + M_PI / 2; // Added for Baxter's control
  	}else
  		pitch = asin(sinp);

  	// yaw (z-axis rotation)
  	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
  	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  	yaw = atan2(siny, cosy);
  }

  // Calculates rotation matrix to euler angles
  // The result is the same as MATLAB except the order
  // of the euler angles ( x and z are swapped ).
  static bool toEulerAngle(const tf::Matrix3x3& R, double& roll, double& pitch, double& yaw)
  {
      //assert(isRotationMatrix(R));
      double sy = sqrt(R.getRow(0).getX() * R.getRow(0).getX() +  R.getRow(1).getX() * R.getRow(1).getX() );

      bool singular = sy < 1e-6; // If

      if (!singular)
      {
          roll = atan2(R.getRow(2).getY() , R.getRow(2)[2]);
          pitch = atan2(-R.getRow(2).getX(), sy);
          yaw = atan2(R.getRow(1).getX(), R.getRow(0).getX());
      }
      else
      {
          roll = atan2(-R.getRow(1).getZ(), R.getRow(1).getY());
          pitch = atan2(-R.getRow(2).getX(), sy);
          yaw = 0;
      }
      return !singular;
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

  /** Callback for mtw topic **/
  void mtw_cb(const mtw_driver::TransformStampedMultiarray::ConstPtr& mtw_msg)
  {
	  msg_mtw = *mtw_msg;

	  tf::Quaternion q0, q1, q01, qi;
	  double R0, P0, Y0;
	  double roll, pitch, yaw;
	  for(int i = 0; i < msg_mtw.data.size(); i++){

		  for(int j = 0; j < msg_mtw.data.size(); j++){
			  if(limb_=="head"){
				  if((msg_mtw.data.at(i).header.frame_id==std::string("back_torso")) &&
					 (msg_mtw.data.at(j).header.frame_id==std::string("back_head"))){
					  tf::quaternionMsgToTF(msg_mtw.data.at(i).transform.rotation, q0);
					  tf::quaternionMsgToTF(msg_mtw.data.at(j).transform.rotation, q1);
					  qi.setEulerZYX(0.0,-M_PI/2.0,0.0);
					  q0 = q0*qi;
					  q1 = q1*qi;
					  tf::Matrix3x3(q0).getRPY(R0, P0, Y0);
					  tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
					  //if(toEulerAngle(tf::Matrix3x3(q1), roll, pitch, yaw))
					  {
					  ROS_DEBUG("head: %f %f %f", roll, pitch, yaw);
					  req_q0 = angle_diff(yaw, Y0);
					  //req_q0 = yaw - Y0;
					  req_q1 = pitch;
					  }
				  }
			  }else{

				  if((msg_mtw.data.at(i).header.frame_id==std::string("back_torso")) &&
					 (msg_mtw.data.at(j).header.frame_id==std::string(limb_+"_arm"))){
					  tf::quaternionMsgToTF(msg_mtw.data.at(i).transform.rotation, q0);
					  tf::quaternionMsgToTF(msg_mtw.data.at(j).transform.rotation, q1);
					  qi.setEulerZYX(0.0,-M_PI/2.0,0.0);
					  q0 = q0*qi;
					  tf::Matrix3x3(q0).getRPY(R0, P0, Y0);
					  tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
//					  tf::Matrix3x3(q01).getRPY(R01, P01, Y01);
					  //if(toEulerAngle(tf::Matrix3x3(q1), roll, pitch, yaw))
					  {
					  ROS_DEBUG("arm: %f %f %f", roll, pitch, yaw);
					  if(limb_=="right"){
						  req_q0 = angle_diff(yaw, Y0) + M_PI/4.0;
						  //req_q0 = yaw - Y0 + M_PI/4.0;
						  req_q1 = -pitch;
						  req_q2 = -roll + M_PI/2.0;
					  }else if(limb_=="left"){
						  req_q0 = angle_diff(yaw, Y0) - M_PI/4.0;
						  //req_q0 = yaw - Y0 - M_PI/4.0;
						  req_q1 = -pitch;
						  req_q2 = -roll - M_PI/2.0;
					  }else{
						  req_q0 = angle_diff(yaw, Y0);
						  //req_q0 = yaw - Y0;
						  req_q1 = -pitch;
						  req_q2 = -roll;
					  }
					  }
				  }else if((msg_mtw.data.at(i).header.frame_id==std::string(limb_+"_arm")) &&
						   (msg_mtw.data.at(j).header.frame_id==std::string(limb_+"_elbow"))){
					  tf::quaternionMsgToTF(msg_mtw.data.at(i).transform.rotation, q0);
					  tf::quaternionMsgToTF(msg_mtw.data.at(j).transform.rotation, q1);
					  q01 = q0.inverse()*q1;
					  toEulerAngle(tf::Matrix3x3(q01), roll, pitch, yaw);
					  //if(toEulerAngle(tf::Matrix3x3(q01), roll, pitch, yaw))
					  {
					  ROS_DEBUG("forearm: %f %f %f", roll, pitch, yaw);
					  if(limb_=="right"){
						  req_q3 = yaw;// - angle_diff(Y0, Y0i);
						  req_q4 = -roll - M_PI/2.0;
					  }else if(limb_=="left"){
						  req_q3 = -yaw;// - angle_diff(Y0, Y0i);
						  req_q4 = -roll + M_PI/2.0;
					  }else{
						  req_q3 = yaw;
						  req_q4 = -roll;
					  }
					  }
				  }else if((msg_mtw.data.at(i).header.frame_id==std::string(limb_+"_elbow")) &&
						   (msg_mtw.data.at(j).header.frame_id==std::string(limb_+"_hand"))){
					  tf::quaternionMsgToTF(msg_mtw.data.at(i).transform.rotation, q0);
					  tf::quaternionMsgToTF(msg_mtw.data.at(j).transform.rotation, q1);
					  q01 = q0.inverse()*q1;
					  toEulerAngle(tf::Matrix3x3(q01), roll, pitch, yaw);
					  //if(toEulerAngle(tf::Matrix3x3(q01), roll, pitch, yaw))
					  {
					  ROS_DEBUG("hand: %f %f %f", roll, pitch, yaw);
					  req_q5 = -pitch + M_PI/2.0;
					  req_q6 = -yaw;
					  }
				  }else if((msg_mtw.data.at(i).header.frame_id==std::string(limb_+"_hand")) &&
						   (msg_mtw.data.at(j).header.frame_id==std::string(limb_+"_grip"))){
					  tf::quaternionMsgToTF(msg_mtw.data.at(i).transform.rotation, q0);
					  tf::quaternionMsgToTF(msg_mtw.data.at(j).transform.rotation, q1);
					  ROS_DEBUG("angle: %f", abs(q0.dot(q1)));
//					  // Original condition to close/open gripper
//					  if(abs(q0.dot(q1))>0.98)
//						  req_gripper = 1;
//					  else
//						  req_gripper = 0;
					  // New condition to enable/disable teleoperation
					  if(abs(q0.dot(q1))<0.4)
						  req_gripper = 1;
					  else
						  req_gripper = 0;
				  }
			  }
		  }
	  }
  }


  void send_cmd()
  {

	//deadman_ = true;
	deadman_ = req_gripper;


    	if(limb_=="head"){
			// Copy joint position to cmd_joint message
			joint_states.position[0]=req_q0;
			joint_states.position[1]=req_q1;
			// Publish joint_states with time stamp
			joint_states.header.stamp=ros::Time::now();
			joint_states_pub_.publish(joint_states);

			// Copy joint position to cmd_joint message
			cmd_joint.position[0]=req_q0;
			cmd_joint.position[1]=req_q1;
			// Publish cmd_joint with time stamp
			cmd_joint.header.stamp=ros::Time::now();
			pose_joint_pub_.publish(cmd_joint);
			ROS_DEBUG("baxter_teleop::mtw mode %d, q0 %f, q1 %f", mode, cmd_joint.position[0], cmd_joint.position[1]);

			try{
				listener.lookupTransform("/"+body_+"/base", "/"+body_+"/back_head",
				ros::Time(0), transform);
				req_vx = transform.getOrigin().x();
				req_vy = transform.getOrigin().y();
				req_vz = transform.getOrigin().z();
				//tf::Matrix3x3 M(transform.getRotation());
				//M.getEulerYPR(req_wz, req_wy, req_wx);
				q = transform.getRotation();
				tf::quaternionTFToMsg(q, req_q);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(0.01).sleep();
			}

			// Copy position and orientation to pose message
			cmd_pose.pose.position.x = req_vx;
			cmd_pose.pose.position.y = req_vy;
			cmd_pose.pose.position.z = req_vz;
			cmd_pose.pose.orientation = req_q;
			// Publish cmd_pose
			cmd_pose.header.stamp=ros::Time::now();
			pose_pub_.publish(cmd_pose);
			ROS_DEBUG("baxter_teleop::xsens (Cartesian space control): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);

    	}

	if(deadman_)
	{
    	//}else{
        if(limb_!="head"){

			// Copy joint position to cmd_joint message
			joint_states.position[0]=req_q0;
			joint_states.position[1]=req_q1;
			joint_states.position[2]=req_q2;
			joint_states.position[3]=req_q3;
			joint_states.position[4]=req_q4;
			joint_states.position[5]=req_q5;
			joint_states.position[6]=req_q6;
			// Publish joint_states with time stamp
			joint_states.header.stamp=ros::Time::now();
			joint_states_pub_.publish(joint_states);

			if(control_=="joint"){
				// Copy joint position to cmd_joint message
				cmd_joint.position[0]=req_q0;
				cmd_joint.position[1]=req_q1;
				cmd_joint.position[2]=req_q2;
				cmd_joint.position[3]=req_q3;
				cmd_joint.position[4]=req_q4;
				cmd_joint.position[5]=req_q5;
				cmd_joint.position[6]=req_q6;
				// Publish cmd_joint with time stamp
				cmd_joint.header.stamp=ros::Time::now();
				pose_joint_pub_.publish(cmd_joint);
				ROS_DEBUG("baxter_teleop::mtw mode %d, q0 %f, q1 %f, q2 %f, q3 %f, q4 %f, q5 %f, q6 %f", mode, cmd_joint.position[0], cmd_joint.position[1], cmd_joint.position[2], cmd_joint.position[3], cmd_joint.position[4], cmd_joint.position[5], cmd_joint.position[6]);

			}else if(control_=="cartesian"){

				try{
					listener.lookupTransform("/"+body_+"/base", "/"+body_+"/"+limb_+"_gripper",
					ros::Time(0), transform);
					req_vx = transform.getOrigin().x();
					req_vy = transform.getOrigin().y();
					req_vz = transform.getOrigin().z();
					//tf::Matrix3x3 M(transform.getRotation());
					//M.getEulerYPR(req_wz, req_wy, req_wx);
					q = transform.getRotation();
					tf::quaternionTFToMsg(q, req_q);
				}
				catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(0.01).sleep();
				}

				// Copy position and orientation to pose message
				cmd_pose.pose.position.x = req_vx;
				cmd_pose.pose.position.y = req_vy;
				cmd_pose.pose.position.z = req_vz;
				cmd_pose.pose.orientation = req_q;
				// Publish cmd_pose
				cmd_pose.header.stamp=ros::Time::now();
				pose_pub_.publish(cmd_pose);
				ROS_DEBUG("baxter_teleop::xsens (Cartesian space control): dx %f, dy %f, dz %f, w %f, x %f, y %f, z %f", cmd_pose.pose.position.x, cmd_pose.pose.position.y, cmd_pose.pose.position.z, cmd_pose.pose.orientation.w, cmd_pose.pose.orientation.x, cmd_pose.pose.orientation.y, cmd_pose.pose.orientation.z);
			}
//			if(req_gripper != cmd_gripper.seq){
//				cmd_gripper.stamp=ros::Time::now();
//				cmd_gripper.seq = req_gripper;
//				cmd_gripper.frame_id = "base";
//				gripper_pub_.publish(cmd_gripper);
//			}
    	}
	}
//    else
//    {
//		// Publish zero commands if deadman_no_publish is false
//		cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.linear.z = 0;
//		cmd_vel.twist.angular.x = cmd_vel.twist.angular.y = cmd_vel.twist.angular.z = 0;
//		if (!deadman_no_publish_)
//		{
//			// Publish cmd_vel
//			cmd_vel.header.stamp=ros::Time::now();
//			vel_pub_.publish(cmd_vel);
//		}
//    }
//
//    //make sure we store the state of our last deadman
//    last_deadman_ = deadman_;
  }

};

int main(int argc, char **argv)
{
  // Initializing the roscpp Node
  ros::init(argc, argv, "baxter_mtw");

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

