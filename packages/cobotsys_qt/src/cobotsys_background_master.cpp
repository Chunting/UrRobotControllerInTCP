//
// Created by 潘绪洋 on 17-2-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QUuid>
#include "cobotsys_background_master.h"


namespace cobotsys {


BackgroundMaster::BackgroundMaster(QObject *parent) : ComputeMaster(parent){

    _instance_id = QUuid::createUuid().toString();
    _callback_manager = std::make_shared<JsonCallbackManager>(_instance_id);
}


BackgroundMaster::~BackgroundMaster(){
}

void BackgroundMaster::processClientData(QTcpSocket *clientLink, const QByteArray &ba){





}




//
}