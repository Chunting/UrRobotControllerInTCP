function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["PID_D"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.var["PID_P"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.var["dead_zone_end"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.var["dead_zone_start"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.var["filter_den"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 16};
	 this.metricsArray.var["filter_num"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 16};
	 this.metricsArray.var["force_error"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.var["force_filter"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 48};
	 this.metricsArray.fcn["ForceController.cpp:rate_scheduler"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::ForceControllerClass"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getBlockParameters"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getRTM"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::initialize"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 12,
	stackTotal: 12};
	 this.metricsArray.fcn["ForceControllerClass::setBlockParameters"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::step"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 32,
	stackTotal: 32};
	 this.metricsArray.fcn["ForceControllerClass::terminate"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::~ForceControllerClass"] = {file: "D:\\Projects\\cobotsys\\packages\\plugin_library\\ForceControl\\MemoryInsertor\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memset"] = {file: "C:\\MATLAB\\R2016b\\sys\\lcc\\include\\string.h",
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
