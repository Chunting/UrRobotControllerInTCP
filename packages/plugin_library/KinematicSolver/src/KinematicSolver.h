//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_KINEMATICSOLVER_H
#define PROJECT_KINEMATICSOLVER_H


#include "cobotsys_abstract_ik_solver.h"
#include <QObject>
//#include "..\..\orocos_kdl\src\"

using namespace cobotsys;
class KinematicSolver : virtual public cobotsys::AbstractIKSolver {
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
    virtual bool solve(const vector<double>& cur, const vector<double>& target, vector<double>& result);
    virtual bool setup(const QString& configFilePath);

protected:
	QString m_defaultSolverInfo;
    QString m_urdf_path;
};


#endif //PROJECT_KINEMATICSOLVER_H
