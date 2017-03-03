//
// Created by 潘绪洋 on 17-3-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_QT_ROS_NODE_H
#define PROJECT_COBOTSYS_QT_ROS_NODE_H

#include <QThread>
#include <QStringListModel>

/**
 * @file /eros_qtalker/include/eros_qtalker/qnode.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 *
 * @note 这个文件的思路是从ROS的qt_ros里找到的。
 *
 **/

namespace cobotsys {


class QtRosNode : public QThread {
Q_OBJECT
public:
    QtRosNode(int argc, char** argv, const std::string &name );
    virtual ~QtRosNode();

    bool on_init();
    bool on_init(const std::string &master_url, const std::string &host_url);
    void shutdown();
    virtual void run() = 0;

    const std::string& nodeName() { return node_name; }

Q_SIGNALS:
    void rosShutdown();

protected:
    virtual void ros_comms_init() = 0;
    int init_argc;
    char** init_argv;
    const std::string node_name;

};


}

#endif //PROJECT_COBOTSYS_QT_ROS_NODE_H
