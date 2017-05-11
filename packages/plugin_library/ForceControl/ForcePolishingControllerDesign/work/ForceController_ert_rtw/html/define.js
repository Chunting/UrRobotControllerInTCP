function CodeDefine() { 
this.def = new Array();
this.def["ForceController_Obj"] = {file: "ert_main_cpp.html",line:20,type:"var"};
this.def["fz_touch"] = {file: "ert_main_cpp.html",line:23,type:"var"};
this.def["dis_offset"] = {file: "ert_main_cpp.html",line:26,type:"var"};
this.def["rt_OneStep"] = {file: "ert_main_cpp.html",line:40,type:"fcn"};
this.def["main"] = {file: "ert_main_cpp.html",line:76,type:"fcn"};
this.def["force_filter"] = {file: "ForceController_cpp.html",line:18,type:"var"};
this.def["CV"] = {file: "ForceController_cpp.html",line:19,type:"var"};
this.def["PID_D"] = {file: "ForceController_cpp.html",line:22,type:"var"};
this.def["PID_I"] = {file: "ForceController_cpp.html",line:25,type:"var"};
this.def["K_lamuda"] = {file: "ForceController_cpp.html",line:28,type:"var"};
this.def["PID_N"] = {file: "ForceController_cpp.html",line:31,type:"var"};
this.def["PID_P"] = {file: "ForceController_cpp.html",line:34,type:"var"};
this.def["dead_zone_end"] = {file: "ForceController_cpp.html",line:37,type:"var"};
this.def["dead_zone_start"] = {file: "ForceController_cpp.html",line:40,type:"var"};
this.def["filter_den"] = {file: "ForceController_cpp.html",line:43,type:"var"};
this.def["filter_num"] = {file: "ForceController_cpp.html",line:47,type:"var"};
this.def["saturation_lower_limit"] = {file: "ForceController_cpp.html",line:51,type:"var"};
this.def["saturation_upper_limit"] = {file: "ForceController_cpp.html",line:54,type:"var"};
this.def["rate_scheduler"] = {file: "ForceController_cpp.html",line:64,type:"fcn"};
this.def["step"] = {file: "ForceController_cpp.html",line:77,type:"fcn"};
this.def["initialize"] = {file: "ForceController_cpp.html",line:189,type:"fcn"};
this.def["terminate"] = {file: "ForceController_cpp.html",line:226,type:"fcn"};
this.def["getRTM"] = {file: "ForceController_cpp.html",line:280,type:"fcn"};
this.def["RT_MODEL_ForceController_T"] = {file: "ForceController_h.html",line:27,type:"type"};
this.def["B_ForceController_T"] = {file: "ForceController_h.html",line:32,type:"type"};
this.def["DW_ForceController_T"] = {file: "ForceController_h.html",line:40,type:"type"};
this.def["ExtU_ForceController_T"] = {file: "ForceController_h.html",line:45,type:"type"};
this.def["ExtY_ForceController_T"] = {file: "ForceController_h.html",line:50,type:"type"};
this.def["P_ForceController_T"] = {file: "ForceController_h.html",line:87,type:"type"};
this.def["ForceController_P"] = {file: "ForceController_h.html",line:172,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:47,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:48,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:49,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:50,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:51,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:52,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:61,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:63,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:64,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:65,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:86,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_cpp.html"] = "../ert_main.cpp";
	this.html2Root["ert_main_cpp.html"] = "ert_main_cpp.html";
	this.html2SrcPath["ForceController_cpp.html"] = "../ForceController.cpp";
	this.html2Root["ForceController_cpp.html"] = "ForceController_cpp.html";
	this.html2SrcPath["ForceController_h.html"] = "../ForceController.h";
	this.html2Root["ForceController_h.html"] = "ForceController_h.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_cpp.html","ForceController_cpp.html","ForceController_h.html","rtwtypes_h.html"];
