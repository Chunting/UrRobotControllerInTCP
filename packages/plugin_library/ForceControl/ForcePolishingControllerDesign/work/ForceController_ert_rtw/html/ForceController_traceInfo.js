function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Fz_touch */
	this.urlHashMap["ForceController:40"] = "ert_main.cpp:22&ForceController.cpp:85,174&ForceController.h:44";
	/* <Root>/2-order TF */
	this.urlHashMap["ForceController:70"] = "ForceController.cpp:18,44,48,88,173,209,236&ForceController.h:36,55,122,155,158";
	/* <Root>/Dead Zone */
	this.urlHashMap["ForceController:16"] = "ForceController.cpp:38,41,91,100&ForceController.h:149,152";
	/* <Root>/Discrete PID Controller */
	this.urlHashMap["ForceController:4"] = "ForceController.h:245";
	/* <Root>/Gain */
	this.urlHashMap["ForceController:54"] = "ForceController.cpp:150,264&ForceController.h:82";
	/* <Root>/Gain1 */
	this.urlHashMap["ForceController:74"] = "ForceController.cpp:29,161&ForceController.h:31,140";
	/* <Root>/LPF1 */
	this.urlHashMap["ForceController:71"] = "ForceController.h:246";
	/* <Root>/Memory */
	this.urlHashMap["ForceController:83"] = "ForceController.cpp:103,212,239&ForceController.h:58";
	/* <Root>/Rate Limiter */
	this.urlHashMap["ForceController:55"] = "ForceController.cpp:122,136,221,254,257,260&ForceController.h:39,73,76,79";
	/* <Root>/Rate Limiter
Dynamic */
	this.urlHashMap["ForceController:82"] = "ForceController.h:239,247";
	/* <Root>/Rate Transition */
	this.urlHashMap["ForceController:47"] = "ForceController.h:227";
	/* <Root>/Saturation */
	this.urlHashMap["ForceController:17"] = "ForceController.cpp:19,52,55,138,147&ForceController.h:123,161,164";
	/* <Root>/Sum3 */
	this.urlHashMap["ForceController:81"] = "ForceController.cpp:102";
	/* <Root>/disOffset */
	this.urlHashMap["ForceController:42"] = "ert_main.cpp:25&ForceController.cpp:149,156,180&ForceController.h:49";
	/* <S1>/Derivative Gain */
	this.urlHashMap["ForceController:4:1684"] = "ForceController.cpp:23,109&ForceController.h:134";
	/* <S1>/Filter */
	this.urlHashMap["ForceController:4:1686"] = "ForceController.cpp:108,168,218,248,251&ForceController.h:38,67,70";
	/* <S1>/Filter Coefficient */
	this.urlHashMap["ForceController:4:1687"] = "ForceController.cpp:32,107&ForceController.h:143";
	/* <S1>/Integral Gain */
	this.urlHashMap["ForceController:4:1683"] = "ForceController.cpp:26,158&ForceController.h:137";
	/* <S1>/Integrator */
	this.urlHashMap["ForceController:4:1685"] = "ForceController.cpp:116,164,215,242,245&ForceController.h:37,61,64";
	/* <S1>/Proportional Gain */
	this.urlHashMap["ForceController:4:1682"] = "ForceController.cpp:35,117&ForceController.h:146";
	/* <S1>/Sum */
	this.urlHashMap["ForceController:4:1681"] = "ForceController.cpp:115";
	/* <S1>/SumD */
	this.urlHashMap["ForceController:4:1688"] = "ForceController.cpp:110";
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
	/* <S3>/Delay Input2 */
	this.urlHashMap["ForceController:82:4"] = "msg=rtwMsg_notTraceable&block=ForceController:82:4";
	/* <S3>/Difference Inputs1 */
	this.urlHashMap["ForceController:82:5"] = "msg=rtwMsg_notTraceable&block=ForceController:82:5";
	/* <S3>/Difference Inputs2 */
	this.urlHashMap["ForceController:82:6"] = "msg=rtwMsg_notTraceable&block=ForceController:82:6";
	/* <S3>/FixPt
Data Type
Duplicate */
	this.urlHashMap["ForceController:82:7"] = "msg=rtwMsg_notTraceable&block=ForceController:82:7";
	/* <S3>/delta fall limit */
	this.urlHashMap["ForceController:82:9"] = "msg=rtwMsg_notTraceable&block=ForceController:82:9";
	/* <S3>/delta rise limit */
	this.urlHashMap["ForceController:82:10"] = "msg=rtwMsg_notTraceable&block=ForceController:82:10";
	/* <S3>/sample time */
	this.urlHashMap["ForceController:82:11"] = "msg=rtwMsg_notTraceable&block=ForceController:82:11";
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
	this.rtwnameHashMap["<S3>"] = {sid: "ForceController:82"};
	this.sidHashMap["ForceController:82"] = {rtwname: "<S3>"};
	this.rtwnameHashMap["<Root>/Fz_touch"] = {sid: "ForceController:40"};
	this.sidHashMap["ForceController:40"] = {rtwname: "<Root>/Fz_touch"};
	this.rtwnameHashMap["<Root>/2-order TF"] = {sid: "ForceController:70"};
	this.sidHashMap["ForceController:70"] = {rtwname: "<Root>/2-order TF"};
	this.rtwnameHashMap["<Root>/Dead Zone"] = {sid: "ForceController:16"};
	this.sidHashMap["ForceController:16"] = {rtwname: "<Root>/Dead Zone"};
	this.rtwnameHashMap["<Root>/Discrete PID Controller"] = {sid: "ForceController:4"};
	this.sidHashMap["ForceController:4"] = {rtwname: "<Root>/Discrete PID Controller"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "ForceController:54"};
	this.sidHashMap["ForceController:54"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/Gain1"] = {sid: "ForceController:74"};
	this.sidHashMap["ForceController:74"] = {rtwname: "<Root>/Gain1"};
	this.rtwnameHashMap["<Root>/LPF1"] = {sid: "ForceController:71"};
	this.sidHashMap["ForceController:71"] = {rtwname: "<Root>/LPF1"};
	this.rtwnameHashMap["<Root>/Memory"] = {sid: "ForceController:83"};
	this.sidHashMap["ForceController:83"] = {rtwname: "<Root>/Memory"};
	this.rtwnameHashMap["<Root>/Rate Limiter"] = {sid: "ForceController:55"};
	this.sidHashMap["ForceController:55"] = {rtwname: "<Root>/Rate Limiter"};
	this.rtwnameHashMap["<Root>/Rate Limiter Dynamic"] = {sid: "ForceController:82"};
	this.sidHashMap["ForceController:82"] = {rtwname: "<Root>/Rate Limiter Dynamic"};
	this.rtwnameHashMap["<Root>/Rate Transition"] = {sid: "ForceController:47"};
	this.sidHashMap["ForceController:47"] = {rtwname: "<Root>/Rate Transition"};
	this.rtwnameHashMap["<Root>/Saturation"] = {sid: "ForceController:17"};
	this.sidHashMap["ForceController:17"] = {rtwname: "<Root>/Saturation"};
	this.rtwnameHashMap["<Root>/Sum3"] = {sid: "ForceController:81"};
	this.sidHashMap["ForceController:81"] = {rtwname: "<Root>/Sum3"};
	this.rtwnameHashMap["<Root>/disOffset"] = {sid: "ForceController:42"};
	this.sidHashMap["ForceController:42"] = {rtwname: "<Root>/disOffset"};
	this.rtwnameHashMap["<S1>/u"] = {sid: "ForceController:4:1"};
	this.sidHashMap["ForceController:4:1"] = {rtwname: "<S1>/u"};
	this.rtwnameHashMap["<S1>/Derivative Gain"] = {sid: "ForceController:4:1684"};
	this.sidHashMap["ForceController:4:1684"] = {rtwname: "<S1>/Derivative Gain"};
	this.rtwnameHashMap["<S1>/Filter"] = {sid: "ForceController:4:1686"};
	this.sidHashMap["ForceController:4:1686"] = {rtwname: "<S1>/Filter"};
	this.rtwnameHashMap["<S1>/Filter Coefficient"] = {sid: "ForceController:4:1687"};
	this.sidHashMap["ForceController:4:1687"] = {rtwname: "<S1>/Filter Coefficient"};
	this.rtwnameHashMap["<S1>/Integral Gain"] = {sid: "ForceController:4:1683"};
	this.sidHashMap["ForceController:4:1683"] = {rtwname: "<S1>/Integral Gain"};
	this.rtwnameHashMap["<S1>/Integrator"] = {sid: "ForceController:4:1685"};
	this.sidHashMap["ForceController:4:1685"] = {rtwname: "<S1>/Integrator"};
	this.rtwnameHashMap["<S1>/Proportional Gain"] = {sid: "ForceController:4:1682"};
	this.sidHashMap["ForceController:4:1682"] = {rtwname: "<S1>/Proportional Gain"};
	this.rtwnameHashMap["<S1>/Sum"] = {sid: "ForceController:4:1681"};
	this.sidHashMap["ForceController:4:1681"] = {rtwname: "<S1>/Sum"};
	this.rtwnameHashMap["<S1>/SumD"] = {sid: "ForceController:4:1688"};
	this.sidHashMap["ForceController:4:1688"] = {rtwname: "<S1>/SumD"};
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
	this.rtwnameHashMap["<S3>/up"] = {sid: "ForceController:82:1"};
	this.sidHashMap["ForceController:82:1"] = {rtwname: "<S3>/up"};
	this.rtwnameHashMap["<S3>/u"] = {sid: "ForceController:82:2"};
	this.sidHashMap["ForceController:82:2"] = {rtwname: "<S3>/u"};
	this.rtwnameHashMap["<S3>/lo"] = {sid: "ForceController:82:3"};
	this.sidHashMap["ForceController:82:3"] = {rtwname: "<S3>/lo"};
	this.rtwnameHashMap["<S3>/Delay Input2"] = {sid: "ForceController:82:4"};
	this.sidHashMap["ForceController:82:4"] = {rtwname: "<S3>/Delay Input2"};
	this.rtwnameHashMap["<S3>/Difference Inputs1"] = {sid: "ForceController:82:5"};
	this.sidHashMap["ForceController:82:5"] = {rtwname: "<S3>/Difference Inputs1"};
	this.rtwnameHashMap["<S3>/Difference Inputs2"] = {sid: "ForceController:82:6"};
	this.sidHashMap["ForceController:82:6"] = {rtwname: "<S3>/Difference Inputs2"};
	this.rtwnameHashMap["<S3>/FixPt Data Type Duplicate"] = {sid: "ForceController:82:7"};
	this.sidHashMap["ForceController:82:7"] = {rtwname: "<S3>/FixPt Data Type Duplicate"};
	this.rtwnameHashMap["<S3>/Saturation Dynamic"] = {sid: "ForceController:82:8"};
	this.sidHashMap["ForceController:82:8"] = {rtwname: "<S3>/Saturation Dynamic"};
	this.rtwnameHashMap["<S3>/delta fall limit"] = {sid: "ForceController:82:9"};
	this.sidHashMap["ForceController:82:9"] = {rtwname: "<S3>/delta fall limit"};
	this.rtwnameHashMap["<S3>/delta rise limit"] = {sid: "ForceController:82:10"};
	this.sidHashMap["ForceController:82:10"] = {rtwname: "<S3>/delta rise limit"};
	this.rtwnameHashMap["<S3>/sample time"] = {sid: "ForceController:82:11"};
	this.sidHashMap["ForceController:82:11"] = {rtwname: "<S3>/sample time"};
	this.rtwnameHashMap["<S3>/Y"] = {sid: "ForceController:82:12"};
	this.sidHashMap["ForceController:82:12"] = {rtwname: "<S3>/Y"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
