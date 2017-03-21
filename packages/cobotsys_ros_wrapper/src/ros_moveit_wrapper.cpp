//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "ros_moveit_wrapper.h"


class ros_moveit_wrapper::MoveItWrapperImpl {
public:
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup* joint_model_group;

    std::string tcp_link;
    std::string joint_group_name;

    MoveItWrapperImpl(){
        joint_group_name = "manipulator";
        tcp_link = "ee_link";
    }

    bool loadModel(const std::string& robot_description){
        robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
        kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();


        auto jmNames = kinematic_model->getJointModelGroupNames();
        for (auto& n : jmNames) {
            ROS_INFO("Joint Model Group Name: %s", n.c_str());
        }
        joint_model_group = kinematic_model->getJointModelGroup(joint_group_name);
        const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();


        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        for (auto& name : joint_model_group->getLinkModelNames()) {
            ROS_INFO("Link Name: %s", name.c_str());
        }

        // Joint Limits
        // ^^^^^^^^^^^^
        // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
        /* Set one joint in the right arm outside its joint limit */
        joint_values[0] = 1.57;
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

        /* Check whether any joint is outside its joint limits */
        ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

        /* Enforce the joint limits for this state and check again*/
        kinematic_state->enforceBounds();
        ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

        // Forward Kinematics
        // ^^^^^^^^^^^^^^^^^^
        // Now, we can compute forward kinematics for a set of random joint
        // values. Note that we would like to find the pose of the
        // "r_wrist_roll_link" which is the most distal link in the
        // "right_arm" of the robot.
        kinematic_state->setToRandomPositions(joint_model_group);
        const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(tcp_link);

        /* Print end-effector pose. Remember that this is in the model frame */
        ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
        ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

        // Inverse Kinematics
        // ^^^^^^^^^^^^^^^^^^
        // We can now solve inverse kinematics (IK) for the right arm of the
        // PR2 robot. To solve IK, we will need the following:
        // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain): end_effector_state that we computed in the step above.
        // * The number of attempts to be made at solving IK: 5
        // * The timeout for each attempt: 0.1 s
        bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

        // Now, we can print out the IK solution (if found):
        if (found_ik) {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i) {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        } else {
            ROS_INFO("Did not find IK solution");
        }

        // Get the Jacobian
        // ^^^^^^^^^^^^^^^^
        // We can also get the Jacobian from the :moveit_core:`RobotState`.
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,
                                     jacobian);
        ROS_INFO_STREAM("Jacobian: " << jacobian);
        // END_TUTORIAL
        return true;
    }
};

ros_moveit_wrapper::ros_moveit_wrapper()
        : m_impl(new MoveItWrapperImpl){
}

ros_moveit_wrapper::~ros_moveit_wrapper(){
}

bool ros_moveit_wrapper::loadRobotModel(const std::string& robot_description){
    return m_impl->loadModel(robot_description);
    return false;
}

bool ros_moveit_wrapper::getIK(const cv::Point3d& pos, const cv::Vec3d& normal, std::vector<double>& jointValue){
    Eigen::Affine3d input;
    //input.fromPositionOrientationScale()
    return false;
}

void ros_moveit_wrapper::forwardKinematics(const std::vector<double>& jointValue){
    m_impl->kinematic_state->setJointGroupPositions(m_impl->joint_model_group, jointValue);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (m_impl->kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    m_impl->kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (m_impl->kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // Now, we can compute forward kinematics for a set of random joint
    // values. Note that we would like to find the pose of the
    // "r_wrist_roll_link" which is the most distal link in the
    // "right_arm" of the robot.
//    m_impl->kinematic_state->setToRandomPositions(m_impl->joint_model_group);

    const Eigen::Affine3d& end_effector_state = m_impl->kinematic_state->getGlobalLinkTransform(m_impl->tcp_link);

    auto r = end_effector_state.rotation();
    auto t = end_effector_state.translation();
}
