//
// Created by zxj on 17-4-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "cobotsys_abstract_factory_macro.h"
#include "BinpickingDetectorBox.h"

COBOTSYS_FACTORY_BEGIN(BinpickingDetectorBoxFactory)
        COBOTSYS_FACTORY_EXPORT(BinpickingDetectorBox)
COBOTSYS_FACTORY_END(BinpickingDetectorBoxFactory, "1.0")