//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "ForceController_wrapper.h"
#include "cobotsys_abstract_factory_macro.h"

COBOTSYS_FACTORY_BEGIN(ForceGuideControllerFactory)
COBOTSYS_FACTORY_EXPORT(ForceController_wrapper)
COBOTSYS_FACTORY_END(ForceGuideControllerFactory, "1.0")