//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_KINEMATICSOLVER_H
#define PROJECT_KINEMATICSOLVER_H


#include "cobotsys_abstract_kinematic_solver.h"
#include <QObject>
#include "../orocos_kdl/src/chain.hpp"
#include "../orocos_kdl/src/chainiksolverpos_lma.hpp"
#include "../orocos_kdl/src/chainfksolverpos_recursive.hpp"

using namespace cobotsys;
class KinematicSolver : public cobotsys::AbstractKinematicSolver {
    struct JointLimits{
      double lower;
      double upper;
      double effort;
      double velocity;
    };
public:
	KinematicSolver();
    virtual ~KinematicSolver();

public:
	/**
	* 用于逆运动求解计算，模型文件设置通过 setup() 来指定。
	* @param[in] cur 当前机器人的状态(关节角)
	* @param[in] target 目标机器人的末端坐标(相对于基坐标)
	* @param[out] result 求解结果(最优结果), 关节角
	* @retval true 求解成功
	* @retval false 无解
	*/
	//virtual bool CartToJnt(const vector<double>& currentJoint, const Eigen::Vector3f xyz, const Eigen::Vector3f rpy, vector<double>& targetJoint);
	virtual bool cartToJnt(const Eigen::VectorXd& initialJoint, const Eigen::Affine3d& targetPos, Eigen::VectorXd& targetJoint);
	virtual bool jntToCart(const Eigen::VectorXd& targetJoint, Eigen::Affine3d& targetPos);

	virtual bool cartToJnt(const std::vector<double>& initialJoint, const std::vector<double>& targetPos, std::vector<double>& targetJoint);
	virtual bool jntToCart(const std::vector<double>& targetJoint, std::vector<double>& targetPos);

    virtual bool setup(const QString& configFilePath="CONFIG/force_control/kinematic_solver_config.json");

protected:
    KDL::Segment segmentParser();
	QString m_defaultSolverInfo;
    KDL::Chain m_robot_chain;
    std::vector<JointLimits> m_robot_joint_limits;
	KDL::ChainFkSolverPos_recursive* m_fk_solver;
	KDL::ChainIkSolverPos_LMA* m_ik_solver;
};

namespace tf {

	/// Converts a KDL rotation into an Eigen quaternion
	void quaternionKDLToEigen(const KDL::Rotation &k, Eigen::Quaterniond &e);

	/// Converts an Eigen quaternion into a KDL rotation
	void quaternionEigenToKDL(const Eigen::Quaterniond &e, KDL::Rotation &k);

	/// Converts a KDL frame into an Eigen Affine3d
	void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e);

	/// Converts a KDL frame into an Eigen Isometry3d
	void transformKDLToEigen(const KDL::Frame &k, Eigen::Isometry3d &e);

	/// Converts an Eigen Affine3d into a KDL frame
	void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k);

	/// Converts an Eigen Isometry3d into a KDL frame
	void transformEigenToKDL(const Eigen::Isometry3d &e, KDL::Frame &k);

	/// Converts a KDL twist into an Eigen matrix
	void twistKDLToEigen(const KDL::Twist &k, Eigen::Matrix<double, 6, 1> &e);

	/// Converts an Eigen matrix into a KDL Twist
	void twistEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Twist &k);

	/// Converts a KDL vector into an Eigen matrix
	void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e);

	/// Converts an Eigen matrix into a KDL vector
	void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> &e, KDL::Vector &k);

	/// Converts a KDL wrench into an Eigen matrix
	void wrenchKDLToEigen(const KDL::Wrench &k, Eigen::Matrix<double, 6, 1> &e);

	/// Converts an Eigen matrix into a KDL wrench
	void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Wrench &k);

} // namespace

#endif //PROJECT_KINEMATICSOLVER_H
