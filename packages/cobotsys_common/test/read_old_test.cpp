//
// Created by 潘绪洋 on 17-2-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <shared_memory.h>

int main(){


    char tbuf[100] = {0};
    cobotsys::common::SharedMemory aa, bb, cc;
    if (bb.open("a")) {
        bb.read(tbuf);
    }


    printf("%s\n", tbuf);

    return 0;
}