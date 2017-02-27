//
// Created by 潘绪洋 on 17-2-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <shared_memory.h>

int main(){
    boost::interprocess::shared_memory_object shdmem(boost::interprocess::open_or_create, "Highscore",
                                                     boost::interprocess::read_write);
    shdmem.truncate(1024);
    boost::interprocess::mapped_region region(shdmem, boost::interprocess::read_write);
    int *i1 = static_cast<int *>(region.get_address());
    *i1 = 99;
    boost::interprocess::mapped_region region2(shdmem, boost::interprocess::read_write);
    int *i2 = static_cast<int *>(region2.get_address());
    std::cout << *i2 << std::endl;


    cobotsys::common::SharedMemory aa, bb, cc;
    char tbuf[100] = {0};
    if (aa.create("a", 100)) {
        if (bb.open("a")) {
            aa.write("Hello2");
            bb.read(tbuf);
        }
    }

    printf("%s\n", tbuf);

    return 0;
}