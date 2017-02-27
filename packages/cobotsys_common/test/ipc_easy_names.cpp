//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "easy_shared_names.h"

using namespace cobotsys::common;

int main(){
    EasySharedNames::remove();
    EasySharedNames sha, shb;

    sha.pushName("Hello");
    shb.pushName(" ");
    sha.pushName("World");
    shb.pushName("!");

    auto names = sha.getNames();
    for (auto &name : names) {
        std::cout << name;
    }
    std::cout << std::endl;
    return 0;
}