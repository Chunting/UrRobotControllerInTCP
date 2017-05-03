//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_gui_logger_highlighter.h>
#include "BasicLoggerWidget.h"

BasicLoggerWidget::BasicLoggerWidget() {
    m_autoScrollBottom = true;
    setupUi();
}

BasicLoggerWidget::~BasicLoggerWidget() {
    COBOT_LOG.clrFilter(this);
}

bool BasicLoggerWidget::setup(const QString& configFilePath) {
    return true;
}

void BasicLoggerWidget::updateTextToUI() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    if (m_cachedMessage.size()) {
        m_textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
        m_textCursor.insertText(m_cachedMessage);

        if (m_autoScrollBottom) {
            m_plainTextEdit->setTextCursor(m_textCursor);
        }
        m_cachedMessage.clear();
    }
}

void BasicLoggerWidget::appendText(const std::string& message) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    static QRegularExpression reg("^\\[\\s*([-A-Za-z_0-9]*)\\s*\\](.*)");

    QString final_message = QString::fromLocal8Bit(message.c_str());
    m_cachedMessage += final_message;
}

void BasicLoggerWidget::setupUi() {
    resize(1024, 600);
    setWindowTitle(tr("Logger"));

    m_plainTextEdit = new QPlainTextEdit(this);
    m_plainTextEdit->setContextMenuPolicy(Qt::CustomContextMenu);

    m_boxLayout = new QVBoxLayout;

    m_boxLayout->setContentsMargins(QMargins());
    m_boxLayout->addWidget(m_plainTextEdit);

    setLayout(m_boxLayout);

    QFont font = getMonospaceFont();
    m_plainTextEdit->setReadOnly(true);
    m_plainTextEdit->setFont(font);
    m_plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);

    m_textCursor = m_plainTextEdit->textCursor();
    cobotsys::gui::LoggerHighlighter::highlightEditorWithDefaultStyle(m_plainTextEdit->document());

    m_editUpdateTimer = new QTimer(this);
    m_editUpdateTimer->setInterval(1000 / 30); // 30Hz
    connect(m_editUpdateTimer, &QTimer::timeout, this, &BasicLoggerWidget::updateTextToUI);
    m_editUpdateTimer->start();

    connect(m_plainTextEdit, &QWidget::customContextMenuRequested, this, &BasicLoggerWidget::customMenu);

    COBOT_LOG.addFilter(this, [=](const std::string& m) { appendText(m); });
}

void BasicLoggerWidget::customMenu() {
    auto menu = m_plainTextEdit->createStandardContextMenu();
    menu->addSeparator();

    auto action = menu->addAction(tr("Clear"));
    connect(action, &QAction::triggered, [=]() { m_plainTextEdit->clear(); });

    action = menu->addAction(tr("Auto Scroll Bottom"));
    action->setCheckable(true);
    action->setChecked(m_autoScrollBottom);
    connect(action, &QAction::triggered, [=](bool bChecked) { m_autoScrollBottom = bChecked; });

    menu->exec(QCursor::pos());
}
