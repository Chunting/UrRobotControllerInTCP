//


#include <cobotsys_abstract_object_factory.h>
#include <extra2.h>
#include "ForceControlSolver.h"
#include "solver/ForceController_wrapper.h"
#include "cobotsys_abstract_factory_macro.h"

COBOTSYS_FACTORY_BEGIN(ForceControlSolverFactory)
  COBOTSYS_FACTORY_EXPORT(ForceControlSolver)
  COBOTSYS_FACTORY_EXPORT(ForceController_wrapper)
COBOTSYS_FACTORY_END(ForceControlSolverFactory, "1.0")