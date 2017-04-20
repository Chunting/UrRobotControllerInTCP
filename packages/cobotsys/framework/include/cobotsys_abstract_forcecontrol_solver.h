//
// Created by longhuicai on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_FORCE_CONTROL_SOLVER_H
#define COBOTSYS_COBOTSYS_ABSTRACT_FORCE_CONTROL_SOLVER_H

#include "cobotsys.h"
#include <vector>
#include "cobotsys_abstract_object.h"
#include "cobotsys_data_types.h"

namespace cobotsys {

/**
 * @addtogroup framework
 * @{
 */

 /**
 * @defgroup force control solver
 * @brief 力反馈机器人控制求解器，输入力和位姿数据，计算求解目标位姿。
 * @{
 */

class AbstractForceControlSolver : public AbstractObject {
public:
	AbstractForceControlSolver();
    virtual ~AbstractForceControlSolver();

    /**
     * 执行求解功能，根据输入力和位姿数据，计算求解目标位姿.
     * @param[in] wrench 当前力的数据
     * @param[in] currentQ 当前位姿， 关节角
     * @param[out] targetQ 求解结果, 目标位姿， 关节角
     * @retval 0 求解成功，其余为错误码
     */
	virtual int solve(const Wrench& wrench, const std::vector<double>& currentQ, std::vector<double>& targetQ)=0;

	/**
	* 执行求解功能，根据输入力和位姿数据，计算求解目标位姿.
	* 力和位姿数据通过observer事件获得
	* @param[out] targetQ 求解结果, 目标位姿， 关节角
	* @retval 0 求解成功，其余为错误码
	*/
	virtual int solve(std::vector<double>& targetQ) = 0;

	void setKinematicSolver(const shared_ptr<AbstractKinematicSolver>& kinSolver) { m_ptrKinematicSolver = kinSolver; }

protected:
	shared_ptr<AbstractKinematicSolver> m_ptrKinematicSolver;
};
/**
 * @}
 */
}


#endif //COBOTSYS_COBOTSYS_ABSTRACT_FORCE_CONTROL_SOLVER_H
