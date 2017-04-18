//
// Created by eleven on 17-4-14.
//
#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_factory_macro.h>
#include "SuckerBinpickingPicker.h"

COBOTSYS_FACTORY_BEGIN(BinpickingPickerFactory)
        COBOTSYS_FACTORY_EXPORT(SuckerBinpickingPicker)
COBOTSYS_FACTORY_END(BinpickingPickerFactory, "1.0")