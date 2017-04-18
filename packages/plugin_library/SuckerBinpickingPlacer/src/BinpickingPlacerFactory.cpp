//
// Created by eleven on 17-4-14.
//

#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_factory_macro.h>
#include "SuckerBinpickingPlacer.h"

COBOTSYS_FACTORY_BEGIN(BinpickingPlacerFactory)
        COBOTSYS_FACTORY_EXPORT(SuckerBinpickingPlacer)
COBOTSYS_FACTORY_END(BinpickingPlacerFactory, "1.0")