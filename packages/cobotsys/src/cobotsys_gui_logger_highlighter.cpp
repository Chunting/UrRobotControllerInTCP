//
// Created by 潘绪洋 on 17-1-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "cobotsys_gui_logger_highlighter.h"


namespace cobotsys {
namespace gui {
LoggerHighlighter::LoggerHighlighter(QTextDocument* parent)
        : QSyntaxHighlighter(parent) {
    defaultFormat.entry.setFontWeight(QFont::Bold);
}

LoggerHighlighter::~LoggerHighlighter() {
}

LoggerHighlighter::LineFormat& LoggerHighlighter::getFormat(const QString& entry) {
    return formats[entry.toUpper()];
}

bool LoggerHighlighter::hasFormat(const QString& entry) {
    return formats.find(entry.toUpper()) != formats.end();
}
}

void gui::LoggerHighlighter::highlightBlock(const QString& text) {
    highlightGeneralLine(text);
    highlightUrl(text);
}

void gui::LoggerHighlighter::loadDefaultLoggerStyle(gui::LoggerHighlighter* highlighter) {
    if (highlighter == nullptr) {
        COBOT_LOG.error() << "Highlighter is nullptr";
        return;
    };

    auto set_warning = [=](const QString& s) {
        highlighter->getFormat(s).entry.setForeground(Qt::darkYellow);
        highlighter->getFormat(s).entry.setFontWeight(QFont::Bold);
        highlighter->getFormat(s).entry.setFontItalic(true);
        highlighter->getFormat(s).text.setForeground(Qt::darkYellow);
    };

    auto set_success = [=](const QString& s) {
        highlighter->getFormat(s).entry.setForeground(Qt::darkGreen);
        highlighter->getFormat(s).entry.setFontWeight(QFont::Bold);
        highlighter->getFormat(s).entry.setFontItalic(true);
        highlighter->getFormat(s).text.setForeground(Qt::darkGreen);
    };

    auto set_error = [=](const QString& s) {
        highlighter->getFormat(s).entry.setForeground(Qt::red);
        highlighter->getFormat(s).entry.setFontWeight(QFont::Bold);
        highlighter->getFormat(s).entry.setFontItalic(true);
        highlighter->getFormat(s).text.setForeground(Qt::red);
    };

    auto set_notice = [=](const QString& s) {
        auto color = QColor::fromRgb(12, 61, 207);
        highlighter->getFormat(s).entry.setForeground(color);
        highlighter->getFormat(s).entry.setFontWeight(QFont::Bold);
        highlighter->getFormat(s).entry.setFontItalic(true);
        highlighter->getFormat(s).text.setForeground(color);
        highlighter->getFormat(s).text.setFontWeight(QFont::Bold);
    };

    set_warning("Warning");
    set_warning("WARNING");
    set_success("Success");
    set_success("SUCCESS");
    set_error("Error");
    set_error("ERROR");
    set_error("Failure");
    set_error("FAILURE");
    set_notice("Notice");
    set_notice("NOTICE");

    highlighter->getFormat("INFO").entry.setFontWeight(QFont::Bold);

    highlighter->fmt_url.setForeground(Qt::darkBlue);
    highlighter->fmt_url.setUnderlineStyle(QTextCharFormat::SingleUnderline);
    highlighter->fmt_url.setUnderlineColor(Qt::blue);
    highlighter->fmt_url.setFontItalic(true);
    highlighter->fmt_url.setFontWeight(QFont::Bold);
}

gui::LoggerHighlighter* gui::LoggerHighlighter::highlightEditorWithDefaultStyle(QTextDocument* document) {
    if (document) {
        LoggerHighlighter* highlighter = new LoggerHighlighter(document);
        loadDefaultLoggerStyle(highlighter);
        return highlighter;
    }
    return nullptr;
}

void gui::LoggerHighlighter::highlightGeneralLine(const QString& text) {
    static QRegularExpression reg("(\\[\\s*(.*?)\\])(.*)");

    auto mexpr = reg.match(text);
    if (mexpr.hasMatch()) {
//        qDebug() << mexpr.captured() << mexpr.captured(1) << mexpr.captured(2) << mexpr.captured(3);
        setFormat(mexpr.capturedStart(1), mexpr.capturedLength(1), defaultFormat.entry);
        if (hasFormat(mexpr.captured(2))) {
            auto& fmt = getFormat(mexpr.captured(2));
            setFormat(mexpr.capturedStart(2), mexpr.capturedLength(2), fmt.entry);
            setFormat(mexpr.capturedStart(3), mexpr.capturedLength(3), fmt.text);
        } else {
            setFormat(mexpr.capturedStart(2), mexpr.capturedLength(2), defaultFormat.entry);
        }
    }
}

void gui::LoggerHighlighter::highlightUrl(const QString& text) {
    static QRegularExpression reg("((https|http|ftp|rtsp|mms)?:\\/\\/)[^\\s]+");
    auto mexpr = reg.match(text);
    if (mexpr.hasMatch()) {
        setFormat(mexpr.capturedStart(), mexpr.capturedLength(), fmt_url);
    }
}
}