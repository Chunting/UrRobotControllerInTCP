function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/force_ee */
	this.urlHashMap["ForceController:40"] = "ert_main.cpp:22&ForceController.cpp:132";
	/* <Root>/gravity_ee */
	this.urlHashMap["ForceController:45"] = "ert_main.cpp:25&ForceController.cpp:133";
	/* <Root>/2-order TF */
	this.urlHashMap["ForceController:70"] = "ForceController.cpp:18,39,43,75,137,189,213&ForceController.h:31,44,105,129,132";
	/* <Root>/Dead Zone */
	this.urlHashMap["ForceController:16"] = "ForceController.cpp:31,35,78,87&ForceController.h:123,126";
	/* <Root>/Discrete PID Controller */
	this.urlHashMap["ForceController:4"] = "ForceController.h:209";
	/* <Root>/Gain */
	this.urlHashMap["ForceController:54"] = "ForceController.cpp:89,217&ForceController.h:47";
	/* <Root>/LPF1 */
	this.urlHashMap["ForceController:71"] = "ForceController.h:210";
	/* <Root>/Rate Limiter */
	this.urlHashMap["ForceController:55"] = "ForceController.cpp:112,126,196,227,230,233&ForceController.h:33,56,59,62";
	/* <Root>/Rate Transition */
	this.urlHashMap["ForceController:47"] = "ForceController.h:191";
	/* <Root>/Saturation */
	this.urlHashMap["ForceController:17"] = "msg=rtwMsg_notTraceable&block=ForceController:17";
	/* <Root>/Sum2 */
	this.urlHashMap["ForceController:46"] = "ForceController.cpp:19,131&ForceController.h:106";
	/* <Root>/poseOffset_ee */
	this.urlHashMap["ForceController:42"] = "ert_main.cpp:28&ForceController.cpp:128,148&ForceController.h:38";
	/* <S1>/Derivative Gain */
	this.urlHashMap["ForceController:4:1687"] = "ForceController.cpp:23,95&ForceController.h:117";
	/* <S1>/Proportional Gain */
	this.urlHashMap["ForceController:4:1686"] = "ForceController.cpp:27,92&ForceController.h:120";
	/* <S1>/Sum */
	this.urlHashMap["ForceController:4:1685"] = "ForceController.cpp:105";
	/* <S2>/Lowpass Filter1 */
	this.urlHashMap["ForceController:60"] = "msg=rtwMsg_notTraceable&block=ForceController:60";
	/* <S2>/Lowpass Filter2 */
	this.urlHashMap["ForceController:61"] = "msg=rtwMsg_notTraceable&block=ForceController:61";
	/* <S2>/Lowpass Filter3 */
	this.urlHashMap["ForceController:62"] = "msg=rtwMsg_notTraceable&block=ForceController:62";
	/* <S2>/Lowpass Filter4 */
	this.urlHashMap["ForceController:63"] = "msg=rtwMsg_notTraceable&block=ForceController:63";
	/* <S2>/Lowpass Filter5 */
	this.urlHashMap["ForceController:64"] = "msg=rtwMsg_notTraceable&block=ForceController:64";
	/* <S2>/Lowpass Filter6 */
	this.urlHashMap["ForceController:65"] = "msg=rtwMsg_notTraceable&block=ForceController:65";
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
	this.rtwnameHashMap["<S2>"] = {sid: "ForceController:71"};
	this.sidHashMap["ForceController:71"] = {rtwname: "<S2>"};
	this.rtwnameHashMap["<S3>"] = {sid: "ForceController:4:1678"};
	this.sidHashMap["ForceController:4:1678"] = {rtwname: "<S3>"};
	this.rtwnameHashMap["<Root>/force_ee"] = {sid: "ForceController:40"};
	this.sidHashMap["ForceController:40"] = {rtwname: "<Root>/force_ee"};
	this.rtwnameHashMap["<Root>/gravity_ee"] = {sid: "ForceController:45"};
	this.sidHashMap["ForceController:45"] = {rtwname: "<Root>/gravity_ee"};
	this.rtwnameHashMap["<Root>/2-order TF"] = {sid: "ForceController:70"};
	this.sidHashMap["ForceController:70"] = {rtwname: "<Root>/2-order TF"};
	this.rtwnameHashMap["<Root>/Dead Zone"] = {sid: "ForceController:16"};
	this.sidHashMap["ForceController:16"] = {rtwname: "<Root>/Dead Zone"};
	this.rtwnameHashMap["<Root>/Discrete PID Controller"] = {sid: "ForceController:4"};
	this.sidHashMap["ForceController:4"] = {rtwname: "<Root>/Discrete PID Controller"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "ForceController:54"};
	this.sidHashMap["ForceController:54"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/LPF1"] = {sid: "ForceController:71"};
	this.sidHashMap["ForceController:71"] = {rtwname: "<Root>/LPF1"};
	this.rtwnameHashMap["<Root>/Rate Limiter"] = {sid: "ForceController:55"};
	this.sidHashMap["ForceController:55"] = {rtwname: "<Root>/Rate Limiter"};
	this.rtwnameHashMap["<Root>/Rate Transition"] = {sid: "ForceController:47"};
	this.sidHashMap["ForceController:47"] = {rtwname: "<Root>/Rate Transition"};
	this.rtwnameHashMap["<Root>/Saturation"] = {sid: "ForceController:17"};
	this.sidHashMap["ForceController:17"] = {rtwname: "<Root>/Saturation"};
	this.rtwnameHashMap["<Root>/Sum2"] = {sid: "ForceController:46"};
	this.sidHashMap["ForceController:46"] = {rtwname: "<Root>/Sum2"};
	this.rtwnameHashMap["<Root>/poseOffset_ee"] = {sid: "ForceController:42"};
	this.sidHashMap["ForceController:42"] = {rtwname: "<Root>/poseOffset_ee"};
	this.rtwnameHashMap["<S1>/u"] = {sid: "ForceController:4:1"};
	this.sidHashMap["ForceController:4:1"] = {rtwname: "<S1>/u"};
	this.rtwnameHashMap["<S1>/Derivative Gain"] = {sid: "ForceController:4:1687"};
	this.sidHashMap["ForceController:4:1687"] = {rtwname: "<S1>/Derivative Gain"};
	this.rtwnameHashMap["<S1>/Differentiator"] = {sid: "ForceController:4:1688"};
	this.sidHashMap["ForceController:4:1688"] = {rtwname: "<S1>/Differentiator"};
	this.rtwnameHashMap["<S1>/Proportional Gain"] = {sid: "ForceController:4:1686"};
	this.sidHashMap["ForceController:4:1686"] = {rtwname: "<S1>/Proportional Gain"};
	this.rtwnameHashMap["<S1>/Sum"] = {sid: "ForceController:4:1685"};
	this.sidHashMap["ForceController:4:1685"] = {rtwname: "<S1>/Sum"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "ForceController:4:10"};
	this.sidHashMap["ForceController:4:10"] = {rtwname: "<S1>/y"};
	this.rtwnameHashMap["<S2>/In1"] = {sid: "ForceController:72"};
	this.sidHashMap["ForceController:72"] = {rtwname: "<S2>/In1"};
	this.rtwnameHashMap["<S2>/Demux"] = {sid: "ForceController:66"};
	this.sidHashMap["ForceController:66"] = {rtwname: "<S2>/Demux"};
	this.rtwnameHashMap["<S2>/Lowpass Filter1"] = {sid: "ForceController:60"};
	this.sidHashMap["ForceController:60"] = {rtwname: "<S2>/Lowpass Filter1"};
	this.rtwnameHashMap["<S2>/Lowpass Filter2"] = {sid: "ForceController:61"};
	this.sidHashMap["ForceController:61"] = {rtwname: "<S2>/Lowpass Filter2"};
	this.rtwnameHashMap["<S2>/Lowpass Filter3"] = {sid: "ForceController:62"};
	this.sidHashMap["ForceController:62"] = {rtwname: "<S2>/Lowpass Filter3"};
	this.rtwnameHashMap["<S2>/Lowpass Filter4"] = {sid: "ForceController:63"};
	this.sidHashMap["ForceController:63"] = {rtwname: "<S2>/Lowpass Filter4"};
	this.rtwnameHashMap["<S2>/Lowpass Filter5"] = {sid: "ForceController:64"};
	this.sidHashMap["ForceController:64"] = {rtwname: "<S2>/Lowpass Filter5"};
	this.rtwnameHashMap["<S2>/Lowpass Filter6"] = {sid: "ForceController:65"};
	this.sidHashMap["ForceController:65"] = {rtwname: "<S2>/Lowpass Filter6"};
	this.rtwnameHashMap["<S2>/Mux"] = {sid: "ForceController:67"};
	this.sidHashMap["ForceController:67"] = {rtwname: "<S2>/Mux"};
	this.rtwnameHashMap["<S2>/Out1"] = {sid: "ForceController:73"};
	this.sidHashMap["ForceController:73"] = {rtwname: "<S2>/Out1"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
