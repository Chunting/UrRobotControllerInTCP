//
// Created by 杨帆 on 17-4-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_KINEMATIC_SOLVER_H
#define COBOTSYS_COBOTSYS_ABSTRACT_KINEMATIC_SOLVER_H

#include <vector>
#include "cobotsys_abstract_object.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
using std::vector;
namespace cobotsys {

/**
 * @addtogroup framework
 * @{
 */
class AbstractKinematicSolver : public AbstractObject {
public:
	AbstractKinematicSolver();
    virtual ~AbstractKinematicSolver();

    /**
     * 用于逆运动求解计算，模型文件设置通过 setup() 来指定。
     * @param[in] initialJoint 当前机器人的状态(关节角)
     * @param[in] targetPos 目标机器人的末端坐标(相对于基坐标)
     * @param[out] targetJoint 求解结果(最优结果), 关节角
     * @retval 0 求解成功，其余为错误码
     */
	virtual int cartToJnt(const Eigen::VectorXd& initialJoint, const Eigen::Affine3d& targetPos, Eigen::VectorXd& targetJoint)=0;
	virtual int cartToJnt(const std::vector<double>& initialJoint, const std::vector<double>& targetPos, std::vector<double>& targetJoint)=0;
	virtual int cartToJnt(const Eigen::VectorXd& initialJoint, const std::vector<double>& targetPos, std::vector<double>& targetJoint) = 0;
	
	/**
	* 用于正向运动求解计算，模型文件设置通过 setup() 来指定。
	* @param[in] targetJoint 机器人的状态(关节角)
	* @param[out] targetPos 目标机器人的末端坐标(相对于基坐标)
	* @retval 0 求解成功，其余为错误码
	*/
	virtual int jntToCart(const Eigen::VectorXd& targetJoint, Eigen::Affine3d& targetPos)=0;
	virtual int jntToCart(const std::vector<double>& targetJoint, std::vector<double>& targetPos)=0;

	/**
	* 用于将世界坐标系中的3维向量转化为机器人末端坐标系中的3维向量。
	* @param[in] jointArray 机器人的状态(关节角)
	* @param[in] vector_world 世界坐标系下3维向量
	* @param[out] vector_ee 机器人末端坐标系(相对于基坐标)中的3维向量。
	* @retval 0 求解成功，其余为错误码
	*/
	virtual int vector_WorldToEE(const Eigen::VectorXd& jointArray, const Eigen::Vector3d& vector_world, Eigen::Vector3d& vector_ee)=0;
	virtual int vector_WorldToEE(const std::vector<double>& jointArray, const Eigen::Vector3d& vector_world, Eigen::Vector3d& vector_ee)=0;
	
	/**
	* 用于将机器人末端坐标系中的姿态转化为世界坐标系中的姿态。
	* @param[in] jointArray 机器人的状态(关节角)
	* @param[in] vector_world 世界坐标系下3维向量
	* @param[out] vector_ee 机器人末端坐标系(相对于基坐标)中的3维向量。
	* @retval 0 求解成功，其余为错误码
	*/
	virtual int pose_EEToWorld(const Eigen::VectorXd& jointArray, const std::vector<double>& pose_ee, std::vector<double>& pose_world)=0;
	virtual int pose_EEToWorld(const std::vector<double>& jointArray, const std::vector<double>& pose_ee, std::vector<double>& pose_world) = 0;
};
/**
 * @}
 */
}


#endif //COBOTSYS_COBOTSYS_ABSTRACT_KINEMATIC_SOLVER_H
