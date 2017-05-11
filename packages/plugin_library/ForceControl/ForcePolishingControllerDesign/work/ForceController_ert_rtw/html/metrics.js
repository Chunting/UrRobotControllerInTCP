function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["CV"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["K_lamuda"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["PID_D"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["PID_I"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["PID_N"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["PID_P"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["dead_zone_end"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["dead_zone_start"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["filter_den"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 16};
	 this.metricsArray.var["filter_num"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 16};
	 this.metricsArray.var["force_filter"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["saturation_lower_limit"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.var["saturation_upper_limit"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	size: 8};
	 this.metricsArray.fcn["ForceController.cpp:rate_scheduler"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::ForceControllerClass"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getBlockParameters"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::getRTM"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::initialize"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::setBlockParameters"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::step"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 40,
	stackTotal: 40};
	 this.metricsArray.fcn["ForceControllerClass::terminate"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceControllerClass::~ForceControllerClass"] = {file: "E:\\cobotsys\\packages\\plugin_library\\ForceControl\\ForcePolishingControllerDesign\\work\\ForceController_ert_rtw\\ForceController.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memset"] = {file: "C:\\MATLAB\\R2017a\\sys\\lcc\\include\\string.h",
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
	 return data; }; 
	 this.codeMetricsSummary = '<a href="ForceController_metrics.html">Global Memory: 120(bytes) Maximum Stack: 40(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
