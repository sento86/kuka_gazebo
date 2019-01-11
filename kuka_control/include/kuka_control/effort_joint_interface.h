#include "ros/ros.h"
#include <kdl/jntarrayvel.hpp>
#include <kuka_control/Efforts.h>
#include <kdl_parser/kdl_parser.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

#include <effort_controllers/joint_position_controller.h>  // used for controlling individual joints

namespace joint_trajectory_controller
{
    typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
    typedef JointTrajectorySegment<SegmentImpl> Segment;
    typedef typename Segment::State State;
}

template<>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
    HardwareInterfaceAdapter() :
            joint_handles_ptr(0)
    {}

    bool init(std::vector<hardware_interface::JointHandle> &joint_handles, ros::NodeHandle &nh)
    {
    	std::string ns = ros::names::parentNamespace(nh.getNamespace());

        // Store pointer to joint handles
        joint_handles_ptr = &joint_handles;

        // Parse the URDF string into a URDF model.
        urdf::Model urdf_model;
        if (!urdf_model.initParam(ns+"/robot_description")) {
            ROS_ERROR("Failed to parse urdf model from robot description");
            return false;
        }
        ROS_INFO("Parsed urdf model from robot description");

        // Compute the KDL tree of the robot from the URDF.
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
            ROS_ERROR("Failed to parse kdl tree from urdf model");
            return false;
        }
        ROS_INFO("Parsed kdl tree from urdf model");

        // Extract chain from KDL tree.
        KDL::Chain chain;
        if (!tree.getChain("base_link", "link_6", chain)) {
            ROS_ERROR("Failed to extract chain from 'base_link' to 'link_6' in kdl tree");
            return false;
        }
        ROS_INFO("Extracted chain from kdl tree");

        // Init effort command publisher.
        publisher.reset(new EffortsPublisher(nh, "efforts", 10));

        // links joints efforts to publisher message.
        joints_efforts = &(publisher->msg_.data);

        // Reset and resize joint states/controls.
        unsigned int n_joints = chain.getNrOfJoints();
        inner_loop_control.resize(n_joints);
        outer_loop_control.resize(n_joints);
        joints_effort_limits.resize(n_joints);
        (*joints_efforts).resize(n_joints);
        joints_state.resize(n_joints);

        for (unsigned int idx = 0; idx < chain.getNrOfJoints(); idx++) {
            // Get joint name.
            std::string name = chain.getSegment(idx).getJoint().getName();

            // Extract joint effort limits from urdf.
            if (!(urdf_model.getJoint(name)) ||
                !(urdf_model.getJoint(name)->limits) ||
                !(urdf_model.getJoint(name)->limits->effort)) {
                ROS_ERROR("No effort limit specified for joint '%s'", name.c_str());
                return false;
            }
            joints_effort_limits.data[idx] = urdf_model.getJoint(name)->limits->effort;
        }
        ROS_INFO("Extracted joint effort limits");

    	double R=0, P=0, Y=0;
    	nh.getParam(ns+"/R0", R);
    	nh.getParam(ns+"/P0", P);
    	nh.getParam(ns+"/Y0", Y);
    	tf::Quaternion q;
    	q.setRPY(R, P, Y);
    	tf::Vector3 g(0, 0, -9.81);
    	g = tf::Matrix3x3(q)*g;

        // Init inverse dynamics solver.
        id_solver.reset(new KDL::ChainIdSolver_RNE(chain, KDL::Vector(g.x(),g.y(),g.z())));
        ROS_INFO("Initialized kdl inverse dynamics solver");

        //************************************************

        // Get number of joints
        n_joints_ = chain.getNrOfJoints();

        // Store nodehandle
        nh_ = nh;

        // Get joint sub-controllers
        XmlRpc::XmlRpcValue xml_struct;
        if (!nh_.getParam("gains", xml_struct))
        {
          ROS_ERROR_NAMED("position", "No 'gains' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
          return false;
        }

        // Make sure it's a struct
        if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR_NAMED("position", "The 'gains' parameter is not a struct (namespace '%s')", nh_.getNamespace().c_str());
          return false;
        }

        // Get number of PID controllers
        n_PIDs_ = xml_struct.size();
        ROS_INFO_STREAM_NAMED("position", "Initializing BaxterPositionController with " << n_PIDs_ << " PID controllers.");

        ROS_ERROR_STREAM(n_PIDs_);

        position_controllers_.resize(n_PIDs_);
        Kp.resize(n_PIDs_);
        Ki.resize(n_PIDs_);
        Kd.resize(n_PIDs_);

        state_error_integral.resize(n_joints_);
        std::fill(state_error_integral.begin(), state_error_integral.end(), 0.0);


        int i = 0;  // track the joint id
        for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
        {
          // Get joint controller
          if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_ERROR_NAMED("position", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                            nh_.getNamespace().c_str());
            return false;
          }

          // Get joint controller name
          std::string joint_controller_name = joint_it->first;

          ROS_ERROR_STREAM(joint_controller_name);

          if (nh_.getParam("gains/"+joint_controller_name+"/p", Kp[i]))
              ROS_ERROR_STREAM(Kp[i]);
          if (nh_.getParam("gains/"+joint_controller_name+"/i", Ki[i]))
              ROS_ERROR_STREAM(Ki[i]);
          if (nh_.getParam("gains/"+joint_controller_name+"/d", Kd[i]))
              ROS_ERROR_STREAM(Kd[i]);

//          // Get the joint-namespace nodehandle
//          {
//            ros::NodeHandle joint_nh(nh_, "joints/" + joint_controller_name);
//            ROS_DEBUG_STREAM_NAMED("init", "Loading sub-controller '" << joint_controller_name
//                                                                     << "', Namespace: " << joint_nh.getNamespace());
//
//            position_controllers_[i].reset(new effort_controllers::JointPositionController());
//            position_controllers_[i]->init(robot, joint_nh);
//
//            // DEBUG
//            // position_controllers_[i]->printDebug();
//
//          }  // end of joint-namespaces
//
//          // Add joint name to map (allows unordered list to quickly be mapped to the ordered index)
//          joint_to_index_map_.insert(std::pair<std::string, std::size_t>(position_controllers_[i]->getJointName(), i));

          // increment joint i
          ++i;
        }

        //************************************************

        return true;
    }

    void starting(const ros::Time & /*time*/)
    {
        if (!joint_handles_ptr) { return; }

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx) {
            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand(0);
        }
    }

    void stopping(const ros::Time & /*time*/)
    {}

    void updateCommand(const ros::Time &     /*time*/,
                       const ros::Duration & /*period*/,
                       const joint_trajectory_controller::State &desired_state,
                       const joint_trajectory_controller::State &state_error)
    {
        if (!joint_handles_ptr) { return; }

        for (size_t idx = 0; idx < joint_handles_ptr->size(); ++idx) {

            // Update joint state with current position (q) and velocity (qdot).
            joints_state.q.data[idx] = (*joint_handles_ptr)[idx].getPosition();
            joints_state.qdot.data[idx] = (*joint_handles_ptr)[idx].getVelocity();

            // Compute outer loop control.
            // todo: dynamic reconfigure parameters.
            outer_loop_control.data[idx] = Kp[idx] * state_error.position[idx] + Kd[idx] * state_error.velocity[idx] + Ki[idx] * state_error_integral[idx];

            //state_error_integral[idx] += state_error.position[idx]*0.01;
            state_error_integral[idx] += state_error.position[idx];
        }

        // No external forces (except gravity).
        KDL::Wrenches external_forces(joint_handles_ptr->size());

        // Solve inverse dynamics (inner loop control).
        if (id_solver->CartToJnt(
                joints_state.q,
                joints_state.qdot,
                outer_loop_control,
                external_forces,
                inner_loop_control) != 0) {
            ROS_ERROR("error solving inverse dynamics");
            return;
        };

        for (unsigned int idx = 0; idx < joint_handles_ptr->size(); ++idx) {
            (*joints_efforts)[idx] = inner_loop_control.data[idx];

            // Limit based on min/max efforts.
            (*joints_efforts)[idx] = std::min((*joints_efforts)[idx], joints_effort_limits.data[idx]);
            (*joints_efforts)[idx] = std::max((*joints_efforts)[idx], -joints_effort_limits.data[idx]);

            // Write joint effort command.
            (*joint_handles_ptr)[idx].setCommand((*joints_efforts)[idx]);

        }

        // Publish efforts.
        if (publisher->trylock()) {
            publisher->msg_.header.stamp = ros::Time::now();
            publisher->unlockAndPublish();
        }
    }

private:

    // Joints handles.
    std::vector<hardware_interface::JointHandle> *joint_handles_ptr;

    // Realtime effort command publisher.
    typedef realtime_tools::RealtimePublisher<kuka_control::Efforts> EffortsPublisher;
    boost::scoped_ptr<EffortsPublisher> publisher;

    // Inverse Dynamics Solver.
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver;

    // Joints state.
    KDL::JntArrayVel joints_state;

    // Joints commands.
    KDL::JntArray
            joints_effort_limits,
            inner_loop_control,
            outer_loop_control;

    // Joints efforts.
    std::vector<double> *joints_efforts;

    //************************************************

    ros::NodeHandle nh_;

    size_t n_PIDs_;
    size_t n_joints_;

    std::vector<double> Kp;
    std::vector<double> Ki;
    std::vector<double> Kd;

    std::vector<double> state_error_integral;

    std::map<std::string, std::size_t> joint_to_index_map_;  // allows incoming messages to be quickly ordered

    // Create an effort-based joint position controller for every joint
    std::vector<boost::shared_ptr<effort_controllers::JointPositionController> > position_controllers_;
};
