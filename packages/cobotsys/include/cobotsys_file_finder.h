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

#include <cobotsys_logger.h>

namespace cobotsys {

class FileFinder {
public:
    static bool isFileExist(const std::string &filePath);
    static std::string find(const std::string &base_name);
    static void addSearchPath(const std::string &path_to_find);
    static std::string realPathOf(const std::string &path);

    static void loadDataPaths();
protected:
    static std::vector<std::string> base_paths;
};
}

#endif //PROJECT_COBOTSYS_FILE_FINDER_H
