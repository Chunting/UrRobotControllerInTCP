//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_IK_SOLVER_H
#define COBOTSYS_COBOTSYS_ABSTRACT_IK_SOLVER_H

#include <vector>
#include "cobotsys_abstract_object.h"

using std::vector;
namespace cobotsys {

/**
 * @addtogroup framework
 * @{
 */
class AbstractIKSolver : public AbstractObject {
public:
    AbstractIKSolver();
    virtual ~AbstractIKSolver();

    /**
     * 用于逆运动求解计算，模型文件设置通过 setup() 来指定。
     * @param[in] cur 当前机器人的状态(关节角)
     * @param[in] target 目标机器人的末端坐标(相对于基坐标)
     * @param[out] result 求解结果(最优结果), 关节角
     * @retval true 求解成功
     * @retval false 无解
     */
    virtual bool solve(const vector<double>& cur, const vector<double>& target, vector<double>& result) = 0;
};
/**
 * @}
 */
}


#endif //COBOTSYS_COBOTSYS_ABSTRACT_IK_SOLVER_H
