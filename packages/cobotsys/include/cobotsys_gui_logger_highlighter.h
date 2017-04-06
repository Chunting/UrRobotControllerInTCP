//
// Created by 潘绪洋 on 17-1-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_GUI_LOGGER_HIGHLIGHTER_H
#define PROJECT_COBOTSYS_GUI_LOGGER_HIGHLIGHTER_H

#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <map>
#include <QRegularExpression>
#include <QDebug>
#include <QFont>
#include <QTextCharFormat>
#include <cobotsys_logger.h>

namespace cobotsys {
namespace gui {

class LoggerHighlighter : QSyntaxHighlighter {
Q_OBJECT
public:
    LoggerHighlighter(QTextDocument* parent = nullptr);
    ~LoggerHighlighter();

public:
    struct LineFormat {
        QTextCharFormat entry;
        QTextCharFormat text;
    };
    std::map<QString, LineFormat> formats;
    LineFormat defaultFormat;
    QTextCharFormat fmt_url;

    LineFormat& getFormat(const QString& entry);
    bool hasFormat(const QString& entry);


    static void loadDefaultLoggerStyle(LoggerHighlighter* highlighter);
    static LoggerHighlighter* highlightEditorWithDefaultStyle(QTextDocument* document);
protected:
    void highlightBlock(const QString& text);
    void highlightGeneralLine(const QString& text);
    void highlightUrl(const QString& text);
};
}
}


#endif //PROJECT_COBOTSYS_GUI_LOGGER_HIGHLIGHTER_H
