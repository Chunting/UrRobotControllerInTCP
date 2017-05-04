//
// Created by eleven on 17-4-14.
//

#ifndef COBOTSYS_UR10ROBOTMOVEDRIVER_H
#define COBOTSYS_UR10ROBOTMOVEDRIVER_H

#include "cobotsys_abstract_arm_robot_move_driver.h"
#include "cobotsys_abstract_arm_robot_realtime_driver.h"
#include <Eigen/Dense>
#include <vector>

using namespace cobotsys;

class UR10RobotMoveDriver : public AbstractArmRobotMoveDriver {
    friend class UR10RobotRealTimeStatusObserver;
public:
    UR10RobotMoveDriver();
    ~UR10RobotMoveDriver();

    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy);

    virtual bool move(uint32_t moveId, const std::vector<RobotWaypoint>& waypoints);

    virtual void attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer);

    virtual void setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver);

    virtual void setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver) {
        m_kinematicSolver = kinematicSolver;
    }

    virtual std::shared_ptr<AbstractKinematicSolver>& getKinematicSolver() { return m_kinematicSolver; }

    virtual bool setup(const QString& configFilePath);

    virtual bool start() { return true; };

    virtual void clearAll() {}

protected:
    void notify(uint32_t moveId);

private:
    std::vector<std::shared_ptr<ArmRobotMoveStatusObserver> > m_observers;
    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_realTimeDriver;
    std::shared_ptr<AbstractKinematicSolver> m_kinematicSolver;

    Eigen::Matrix4d m_cam2base;
    std::vector<double> m_initialJoint;  // 记录机器人最近的位置姿态
    std::mutex m_mutex;
};


#endif //COBOTSYS_UR10ROBOTMOVEDRIVER_H
