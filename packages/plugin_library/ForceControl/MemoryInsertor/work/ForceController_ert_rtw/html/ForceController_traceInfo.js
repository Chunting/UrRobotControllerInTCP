function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/force_ee */
	this.urlHashMap["ForceController:40"] = "ert_main.cpp:22&ForceController.cpp:48";
	/* <Root>/gravity_ee */
	this.urlHashMap["ForceController:45"] = "ert_main.cpp:25&ForceController.cpp:49";
	/* <Root>/Dead Zone */
	this.urlHashMap["ForceController:16"] = "msg=rtwMsg_notTraceable&block=ForceController:16";
	/* <Root>/Discrete PID Controller */
	this.urlHashMap["ForceController:4"] = "ForceController.h:163";
	/* <Root>/Gain */
	this.urlHashMap["ForceController:54"] = "ForceController.h:144";
	/* <Root>/Rate Transition */
	this.urlHashMap["ForceController:47"] = "ForceController.h:145";
	/* <Root>/Saturation */
	this.urlHashMap["ForceController:17"] = "msg=rtwMsg_notTraceable&block=ForceController:17";
	/* <Root>/Sum2 */
	this.urlHashMap["ForceController:46"] = "ForceController.cpp:19,47&ForceController.h:87";
	/* <Root>/filter */
	this.urlHashMap["ForceController:14"] = "msg=rtwMsg_notTraceable&block=ForceController:14";
	/* <Root>/poseOffset_ee */
	this.urlHashMap["ForceController:42"] = "ert_main.cpp:28&ForceController.cpp:61,83&ForceController.h:35";
	/* <S1>/Derivative Gain */
	this.urlHashMap["ForceController:4:1668"] = "ForceController.cpp:55,126&ForceController.h:41";
	/* <S1>/Filter */
	this.urlHashMap["ForceController:4:1670"] = "ForceController.cpp:54,76&ForceController.h:30";
	/* <S1>/Filter Coefficient */
	this.urlHashMap["ForceController:4:1671"] = "ForceController.cpp:53,136&ForceController.h:47";
	/* <S1>/Integral Gain */
	this.urlHashMap["ForceController:4:1667"] = "ForceController.cpp:71,131&ForceController.h:44";
	/* <S1>/Integrator */
	this.urlHashMap["ForceController:4:1669"] = "ForceController.cpp:62,70&ForceController.h:29";
	/* <S1>/Proportional Gain */
	this.urlHashMap["ForceController:4:1666"] = "ForceController.cpp:63,141&ForceController.h:50";
	/* <S1>/Sum */
	this.urlHashMap["ForceController:4:1665"] = "ForceController.cpp:64";
	/* <S1>/SumD */
	this.urlHashMap["ForceController:4:1672"] = "ForceController.cpp:56";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "ForceController"};
	this.sidHashMap["ForceController"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "ForceController:4"};
	this.sidHashMap["ForceController:4"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<Root>/force_ee"] = {sid: "ForceController:40"};
	this.sidHashMap["ForceController:40"] = {rtwname: "<Root>/force_ee"};
	this.rtwnameHashMap["<Root>/gravity_ee"] = {sid: "ForceController:45"};
	this.sidHashMap["ForceController:45"] = {rtwname: "<Root>/gravity_ee"};
	this.rtwnameHashMap["<Root>/Dead Zone"] = {sid: "ForceController:16"};
	this.sidHashMap["ForceController:16"] = {rtwname: "<Root>/Dead Zone"};
	this.rtwnameHashMap["<Root>/Discrete PID Controller"] = {sid: "ForceController:4"};
	this.sidHashMap["ForceController:4"] = {rtwname: "<Root>/Discrete PID Controller"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "ForceController:54"};
	this.sidHashMap["ForceController:54"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/Rate Transition"] = {sid: "ForceController:47"};
	this.sidHashMap["ForceController:47"] = {rtwname: "<Root>/Rate Transition"};
	this.rtwnameHashMap["<Root>/Saturation"] = {sid: "ForceController:17"};
	this.sidHashMap["ForceController:17"] = {rtwname: "<Root>/Saturation"};
	this.rtwnameHashMap["<Root>/Sum2"] = {sid: "ForceController:46"};
	this.sidHashMap["ForceController:46"] = {rtwname: "<Root>/Sum2"};
	this.rtwnameHashMap["<Root>/filter"] = {sid: "ForceController:14"};
	this.sidHashMap["ForceController:14"] = {rtwname: "<Root>/filter"};
	this.rtwnameHashMap["<Root>/poseOffset_ee"] = {sid: "ForceController:42"};
	this.sidHashMap["ForceController:42"] = {rtwname: "<Root>/poseOffset_ee"};
	this.rtwnameHashMap["<S1>/u"] = {sid: "ForceController:4:1"};
	this.sidHashMap["ForceController:4:1"] = {rtwname: "<S1>/u"};
	this.rtwnameHashMap["<S1>/Derivative Gain"] = {sid: "ForceController:4:1668"};
	this.sidHashMap["ForceController:4:1668"] = {rtwname: "<S1>/Derivative Gain"};
	this.rtwnameHashMap["<S1>/Filter"] = {sid: "ForceController:4:1670"};
	this.sidHashMap["ForceController:4:1670"] = {rtwname: "<S1>/Filter"};
	this.rtwnameHashMap["<S1>/Filter Coefficient"] = {sid: "ForceController:4:1671"};
	this.sidHashMap["ForceController:4:1671"] = {rtwname: "<S1>/Filter Coefficient"};
	this.rtwnameHashMap["<S1>/Integral Gain"] = {sid: "ForceController:4:1667"};
	this.sidHashMap["ForceController:4:1667"] = {rtwname: "<S1>/Integral Gain"};
	this.rtwnameHashMap["<S1>/Integrator"] = {sid: "ForceController:4:1669"};
	this.sidHashMap["ForceController:4:1669"] = {rtwname: "<S1>/Integrator"};
	this.rtwnameHashMap["<S1>/Proportional Gain"] = {sid: "ForceController:4:1666"};
	this.sidHashMap["ForceController:4:1666"] = {rtwname: "<S1>/Proportional Gain"};
	this.rtwnameHashMap["<S1>/Sum"] = {sid: "ForceController:4:1665"};
	this.sidHashMap["ForceController:4:1665"] = {rtwname: "<S1>/Sum"};
	this.rtwnameHashMap["<S1>/SumD"] = {sid: "ForceController:4:1672"};
	this.sidHashMap["ForceController:4:1672"] = {rtwname: "<S1>/SumD"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "ForceController:4:10"};
	this.sidHashMap["ForceController:4:10"] = {rtwname: "<S1>/y"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
