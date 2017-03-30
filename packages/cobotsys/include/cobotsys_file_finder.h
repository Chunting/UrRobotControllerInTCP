//
// Created by 潘绪洋 on 17-2-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_FILE_FINDER_H
#define PROJECT_COBOTSYS_FILE_FINDER_H

#include <sys/stat.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <map>

#include <cobotsys_logger.h>

namespace cobotsys {


class FileFinder {

public:
    enum PreDefPath {
        Bin,
        Data,
        Config,
        Script,
        AppRuntimeDir
    };
public:
    static bool isFileExist(const std::string& filePath);
    static std::string find(const std::string& base_name);
    static void addSearchPath(const std::string& path_to_find, PreDefPath pathType = AppRuntimeDir);
    static std::string realPathOf(const std::string& path);

    static void loadDataPaths();

    static std::string getPreDefPath(PreDefPath pathType = Data);
protected:
    static void storSearchPath(const std::string& path_to_find, PreDefPath pathType);
    static std::vector<std::string> base_paths;
    static std::map<PreDefPath, std::string> pre_def_path;
};
}

#endif //PROJECT_COBOTSYS_FILE_FINDER_H
