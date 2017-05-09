//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys.h"
#include "cobotsys_logger.h"
#include "cobotsys_file_finder.h"


/**
 * @mainpage cobotsys
 *
 *
 * 多看代码。。。
 *
 * @section 安装与使用
 *
 * 参看源文件目录下的README.md
 *
 * @see framework
 *
 *
 *
 */


// TODO 下面定义的宏可以改成跨平台的方案
#define PATH_SLASH_CHAR     '/'


namespace cobotsys {


namespace internal_info {

static std::string internal_app_path;
static std::string internal_app_name;
static std::string internal_app_directory;


void parser_app_name(const char* arg) {

    internal_app_path = arg;

    auto slash_pos = internal_app_path.find_last_of(PATH_SLASH_CHAR);
    if (slash_pos == internal_app_path.npos) {
        internal_app_name = internal_app_path;
        internal_app_directory = FileFinder::realPathOf(".");
    } else {
        internal_app_name = internal_app_path.substr(slash_pos + 1);
        internal_app_directory = internal_app_path.substr(0, slash_pos);
    }

    COBOT_LOG.setCurrentInstanceName(internal_app_name);
    COBOT_LOG.notice("App Info") << internal_app_directory << ", " << internal_app_name;
}
}


void init_library(int argc, char** argv) {
    FileFinder::loadDataPaths();
    internal_info::parser_app_name(argv[0]);
    FileFinder::dumpAvailablePath();
}
}


std::ostream& operator<<(std::ostream& oss, const QString& str) {
    oss << str.toLocal8Bit().constData();
    return oss;
}

std::ostream& operator<<(std::ostream& oss, const QJsonObject& obj) {
    oss << QJsonDocument(obj).toJson(QJsonDocument::Compact).constData();
    return oss;
}

bool isFixedPitch(const QFont& font) {
    const QFontInfo fi(font);
    COBOT_LOG.message("FontFinder") << fi.family() << fi.fixedPitch();
    return fi.fixedPitch();
}


QFont enumMonospaceFont() {
    QFont font("monospace");
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::Monospace);
    if (isFixedPitch(font)) return font;
    font.setStyleHint(QFont::TypeWriter);
    if (isFixedPitch(font)) return font;
    font.setFamily("courier");
    if (isFixedPitch(font)) return font;
    return font;
}

QFont getMonospaceFont() {
    static bool monospaceInit = false;
    static QFont monospaceFont;
    if (monospaceInit)
        return monospaceFont;

    monospaceFont = enumMonospaceFont();
    monospaceInit = true;
    return monospaceFont;
}