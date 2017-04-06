//
// Created by 潘绪洋 on 17-4-6.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_data_types.h"

namespace cobotsys {
std::string toString(ImageType imageType){
    switch (imageType) {
    case ImageType::Color: return "Color";
    case ImageType::Depth: return "Depth";
    case ImageType::Ir: return "Ir";
    case ImageType::Mono: return "Mono";
    }
}
}