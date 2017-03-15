//
// Created by 潘绪洋 on 17-2-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COMMON_MACRO_H
#define PROJECT_COMMON_MACRO_H

#include <iostream>
#include <iomanip>
#include <string>


#define STD_CERR (std::cerr << std::string(__FILE__).substr(std::string(__FILE__).find_last_of('/') + 1) << ":" << __LINE__ << ":")


#endif //PROJECT_COMMON_MACRO_H
