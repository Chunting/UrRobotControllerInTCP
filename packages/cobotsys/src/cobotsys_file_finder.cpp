//
// Created by 潘绪洋 on 17-2-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "cobotsys_file_finder.h"

namespace cobotsys {

std::vector<std::string> FileFinder::base_paths;

std::string FileFinder::find(const std::string &base_name){
    if (isFileExist(base_name))
        return realPathOf(base_name);

#ifdef WIN32
    std::string path_slash = "\\";
#else
    std::string path_slash = "/";
#endif

    std::string path;
    for (const auto &base : base_paths) {
        if (base[0] == '.')
            path = "." + path_slash + base + path_slash + base_name;
        else
            path = base + path_slash + base_name;
        if (isFileExist(path))
            return realPathOf(path);
    }

    return std::string();
}

void FileFinder::loadDataPaths(){
    addSearchPath(".");
    addSearchPath("../data");
    addSearchPath("../../data");

    COBOT_LOG.message("File Finder") << "Current Path: " << realPathOf(".");
}

bool FileFinder::isFileExist(const std::string &filePath){
    struct stat stFileInfo;
    int intStat;

    // Attempt to get the file attributes
    intStat = stat(filePath.c_str(), &stFileInfo);
    if (intStat == 0) {
        return true;
    } else {
        return false;
    }
}

std::string FileFinder::realPathOf(const std::string &path){
    std::vector<char> ch_buf(1024);
#ifdef WIN32
    auto pfull = _fullpath(&ch_buf[0], path.c_str(), ch_buf.size());
#else
    auto pfull = realpath(path.c_str(), &ch_buf[0]);
#endif
    if (pfull) {
        return pfull;
    }
    return std::string();
}

void FileFinder::addSearchPath(const std::string &path_to_find){
    bool is_found;
    auto rpath = realPathOf(path_to_find);
    if (rpath.size()) {
        base_paths.push_back(rpath);
        COBOT_LOG.message("File Finder") << "Add Path: " << rpath;
    }
}
}
