function RTW_SidParentMap() {
    this.sidParentMap = new Array();
    this.sidParentMap["ForceController:40"] = "ForceController";
    this.sidParentMap["ForceController:45"] = "ForceController";
    this.sidParentMap["ForceController:16"] = "ForceController";
    this.sidParentMap["ForceController:4"] = "ForceController";
    this.sidParentMap["ForceController:54"] = "ForceController";
    this.sidParentMap["ForceController:47"] = "ForceController";
    this.sidParentMap["ForceController:17"] = "ForceController";
    this.sidParentMap["ForceController:46"] = "ForceController";
    this.sidParentMap["ForceController:14"] = "ForceController";
    this.sidParentMap["ForceController:42"] = "ForceController";
    this.sidParentMap["ForceController:4:1"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1668"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1670"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1671"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1667"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1669"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1666"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1665"] = "ForceController:4";
    this.sidParentMap["ForceController:4:1672"] = "ForceController:4";
    this.sidParentMap["ForceController:4:10"] = "ForceController:4";
    this.getParentSid = function(sid) { return this.sidParentMap[sid];}
}
    RTW_SidParentMap.instance = new RTW_SidParentMap();
