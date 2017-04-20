function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["force_error"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	size: 48};
	 this.metricsArray.fcn["ForceController.cpp:rate_scheduler"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::ForceControllerClass"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getBlockParameters"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getRTM"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::initialize"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 4,
	stackTotal: 4};
	 this.metricsArray.fcn["ForceControllerClass::setBlockParameters"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::step"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 16,
	stackTotal: 16};
	 this.metricsArray.fcn["ForceControllerClass::terminate"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::~ForceControllerClass"] = {file: "/home/sail/CLionProjects/cobotsys/packages/plugin_library/ForceControl/ForceControllerDesign/work/ForceController_ert_rtw/ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memset"] = {file: "/home/sail/opt/MATLAB/R2016b/sys/lcc/include/string.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data;}
}
	 CodeMetrics.instance = new CodeMetrics();
