//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "LoggerViewWidget.h"
#include <QVBoxLayout>
#include <cobotsys.h>

LoggerViewWidget::LoggerViewWidget(QWidget *parent)
        : QWidget(parent){
    auto pLayout = new QVBoxLayout;
    pLayout->setContentsMargins(0, 0, 0, 0);

    QFont font("Courier");

    _plainTextEdit = new QPlainTextEdit(this);
    _plainTextEdit->setReadOnly(true);
    _plainTextEdit->setFont(font);
    _plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);

    _highlighter = cobotsys::gui::LoggerHighlighter::highlightEditorWithDefaultStyle(_plainTextEdit->document());

    _textCursor = _plainTextEdit->textCursor();

    pLayout->addWidget(_plainTextEdit);
    setLayout(pLayout);

    _editUpdateTimer = new QTimer(this);
    _editUpdateTimer->setInterval(1000 / 30); // 30Hz
    connect(_editUpdateTimer, &QTimer::timeout, this, &LoggerViewWidget::updateTextToUI);
    _editUpdateTimer->start();
}


void LoggerViewWidget::appendText(const std::string &entry, const std::string &message){
    static QRegularExpression reg("^\\[\\s*([-A-Za-z_0-9]*)\\s*\\](.*)");

    std::stringstream oss;
    cobotsys::cout_formater formater(oss);

    if (entry.empty()) {
        QString qmsg = QString::fromLocal8Bit(message.c_str()).trimmed();

        auto iter = reg.match(qmsg);
        if (iter.hasMatch()) {
            auto qEntry = iter.captured(1);
            if (qEntry.size() > COBOT_LOG.prefixWidth())
                qEntry = qEntry.leftJustified(COBOT_LOG.prefixWidth(), '.', true);
            auto qMessage = iter.captured(2).trimmed();
            appendText(qEntry.toLocal8Bit().constData(), qMessage.toLocal8Bit().constData());
        } else {
            _cachedMessage += QString::fromLocal8Bit(message.c_str()) + "\n";
        }
    } else {
        formater.section_width = COBOT_LOG.prefixWidth();
        formater.section(entry);
        formater.oss << " " << message;

        QString final_message = QString::fromLocal8Bit(oss.str().c_str()).trimmed() + "\n";
        _cachedMessage += final_message;
    }
}

void LoggerViewWidget::updateTextToUI(){
    if (_cachedMessage.size()) {
        _textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
        _textCursor.insertText(_cachedMessage);

        _plainTextEdit->setTextCursor(_textCursor);
        _cachedMessage.clear();
    }
}

void LoggerViewWidget::clear(){
    _plainTextEdit->clear();
}

void LoggerViewWidget::bindCurrentProcessLogger(){
    COBOT_LOG.setAppendFilter([=](const std::string &e, const std::string &m){ appendText(e, m); });
}

void LoggerViewWidget::setOpacity(){
    auto pl = _plainTextEdit->palette();
    pl.setBrush(QPalette::Base, QBrush(QColor(0, 0, 0, 64)));
    _plainTextEdit->setPalette(pl);
    _plainTextEdit->setAutoFillBackground(false);
}
