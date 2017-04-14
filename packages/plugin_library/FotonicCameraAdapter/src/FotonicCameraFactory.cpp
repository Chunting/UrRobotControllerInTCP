//
// Created by 于天水 on 17-4-12.
//

#include <extra2.h>
#include <cobotsys_abstract_factory_macro.h>
#include "FotonicCamera.h"

COBOTSYS_FACTORY_BEGIN(FotonicCameraFactory)
        COBOTSYS_FACTORY_EXPORT(FotonicCamera)
COBOTSYS_FACTORY_END(FotonicCameraFactory, "1.0")