//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_gui_logger_highlighter.h>
#include <extra2.h>
#include <cobotsys_file_finder.h>
#include "BasicLoggerWidget.h"

BasicLoggerWidget::BasicLoggerWidget() {
    m_autoScrollBottom = true;
    m_enableFilter = false;
    m_enableGUIFilterMenu = true;
    m_ctxMenuFont = getMonospaceFont();
    m_ctxMenuFont.setBold(true);
    setupUi();
}

BasicLoggerWidget::~BasicLoggerWidget() {
    COBOT_LOG.clrFilter(this);
}

bool BasicLoggerWidget::setup(const QString& configFilePath) {
    QJsonObject jsonConfig;
    QString cpath = configFilePath;

    if (cpath.isEmpty())
        cpath = "CONFIG/LoggerFilter.json"; // Give a default config json file.

    if (loadJson(jsonConfig, cpath)) {
        m_configPath = cobotsys::FileFinder::find(cpath);
        COBOT_LOG.debug("DEBUG_0") << m_configPath;

        auto loggerConfig = jsonConfig["LoggerWidgetSetup"].toObject();
        m_enableFilter = loggerConfig["EnableFilter"].toBool(false);
        m_enableGUIFilterMenu = loggerConfig["EnableGUIFilterMenu"].toBool(true);

        for (const auto& iter : loggerConfig["Type"].toArray()) {
            m_typeFilter[iter.toString()] = false;
        }

        for (const auto& iter : loggerConfig["Name"].toArray()) {
            m_nameFilter[iter.toString()] = false;
        }
    }
    return true;
}


void BasicLoggerWidget::appendFilterMessage() {
    QTextStream tss(&m_cachedMessage);
    while (!tss.atEnd()) {
        auto line = tss.readLine();
        if (line.size()) {
            QString type_, name_;
            parserLogTypeName(line, type_, name_);

            if (isTypeVisible(type_) && isNameVisible(name_)) {
                m_textCursor.insertText(line);
                m_textCursor.insertBlock();
            }
        }
    }
}

void BasicLoggerWidget::updateTextToUI() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_cachedMessage.size()) {
        m_textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);

        if (m_enableFilter) {
            appendFilterMessage();
        } else {
            m_textCursor.insertText(m_cachedMessage);
        }


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
    menu->setFont(m_ctxMenuFont);

    menu->addSeparator();
    if (m_enableGUIFilterMenu) {
        addTextFilter(menu);
        menu->addSeparator();
    }

    auto action = menu->addAction(tr("Clear"));
    connect(action, &QAction::triggered, [=]() { m_plainTextEdit->clear(); });

    action = menu->addAction(tr("Auto Scroll Bottom"));
    action->setCheckable(true);
    action->setChecked(m_autoScrollBottom);
    connect(action, &QAction::triggered, [=](bool bChecked) { m_autoScrollBottom = bChecked; });

    action = menu->addAction(tr("Go Bottom"));
    connect(action, &QAction::triggered, [=](bool) { m_plainTextEdit->setTextCursor(m_textCursor); });

    menu->exec(QCursor::pos());
}

void BasicLoggerWidget::parserLogTypeName(const QString& line_, QString& type_, QString& name_) {
    static QRegularExpression reg(
            "\\[\\s*(.*)\\]\\s\\d{4}-\\d{2}-\\d{2}\\s\\d{2}:\\d{2}:\\d{2}\\.\\d{3}\\s(\\[\\s*(.*)\\s*\\])?");

    auto rmatch = reg.match(line_);
    if (rmatch.hasMatch()) {
        type_ = rmatch.captured(1);
        name_ = rmatch.captured(3);
    } else {
        type_.clear();
        name_.clear();
    }
}

void BasicLoggerWidget::showLogType(const QString& type_, bool show_) {
    m_typeFilter[type_] = show_;
}

void BasicLoggerWidget::showLogName(const QString& name_, bool show_) {
    m_nameFilter[name_] = show_;
}

void BasicLoggerWidget::loopAllTextBlock(std::function<void(QTextBlock&)> func) {
    auto doc_ = m_plainTextEdit->document();
    auto blockIter_ = doc_->firstBlock();
    while (blockIter_.isValid()) {
        if (func) {
            func(blockIter_);
        }
        blockIter_ = blockIter_.next();
    }
}

bool BasicLoggerWidget::isTypeVisible(const QString& type_) {
    if (m_typeFilter.find(type_) != m_typeFilter.end()) {
        return m_typeFilter[type_];
    } else {
        m_typeFilter[type_] = true;
        return true;
    }
}

bool BasicLoggerWidget::isNameVisible(const QString& name_) {
    if (m_nameFilter.find(name_) != m_nameFilter.end()) {
        return m_nameFilter[name_];
    } else {
        m_nameFilter[name_] = true;
        return true;
    }
}

void BasicLoggerWidget::addTextFilter(QMenu* menu) {
    QAction* action;
    auto filterMenu = menu->addMenu(tr("Text Filter"));
    filterMenu->setFont(m_ctxMenuFont);

    action = filterMenu->addAction(tr("Enable Filter"));
    action->setCheckable(true);
    action->setChecked(m_enableFilter);
    connect(action, &QAction::triggered, [=](bool bChecked) { m_enableFilter = bChecked; });

    filterMenu->addSeparator();
    addConfigMenu(filterMenu);

    if (m_enableFilter) {
        filterMenu->addSeparator();
        addHideItems(filterMenu);

        auto cursorCur = m_plainTextEdit->cursorForPosition(m_plainTextEdit->mapFromGlobal(QCursor::pos()));
        auto blockCur = cursorCur.block();
        auto lineCur = blockCur.text();
        QString type_, name_;
        parserLogTypeName(lineCur, type_, name_);

        if (lineCur.size()) {
            if (m_typeFilter.find(type_) == m_typeFilter.end()) {
                m_typeFilter[type_] = true;
            }
            if (m_nameFilter.find(name_) == m_nameFilter.end()) {
                m_nameFilter[name_] = true;
            }

            // Type filter
            filterMenu->addSeparator();
            auto typeIter = m_typeFilter.find(type_);
            if (typeIter != m_typeFilter.end()) {
                action = filterMenu->addAction(typeIter->first);
                action->setCheckable(true);
                action->setChecked(typeIter->second);
                connect(action, &QAction::triggered, [=](bool b) { showLogType(type_, b); });
            } else {
            }

            // Name filter
            auto nameIter = m_nameFilter.find(name_);
            if (nameIter != m_nameFilter.end()) {
                action = filterMenu->addAction(nameIter->first);
                action->setCheckable(true);
                action->setChecked(nameIter->second);
                connect(action, &QAction::triggered, [=](bool b) { showLogName(name_, b); });
            }
        }
    }
}

void BasicLoggerWidget::addHideItems(QMenu* menu) {
    QAction* action;
    bool noHideItems = true;
    auto hideItems = menu->addMenu(tr("Hide Types"));
    hideItems->setFont(m_ctxMenuFont);
    action = hideItems->addAction(tr("Show All"));
    connect(action, &QAction::triggered, [=](bool) { m_typeFilter.clear(); });

    for (auto& iter : m_typeFilter) {
        if (!iter.second) {
            action = hideItems->addAction(iter.first);
            action->setCheckable(true);
            action->setChecked(iter.second);
            connect(action, &QAction::triggered, [=](bool b) { showLogType(iter.first, b); });
            noHideItems = false;
        }
    }
    if (noHideItems) hideItems->setEnabled(false);

    noHideItems = true;
    hideItems = menu->addMenu(tr("Hide Names"));
    hideItems->setFont(m_ctxMenuFont);
    action = hideItems->addAction(tr("Show All"));
    connect(action, &QAction::triggered, [=](bool) { m_nameFilter.clear(); });

    for (auto& iter : m_nameFilter) {
        if (!iter.second) {
            action = hideItems->addAction(iter.first);
            action->setCheckable(true);
            action->setChecked(iter.second);
            connect(action, &QAction::triggered, [=](bool b) { showLogName(iter.first, b); });
            noHideItems = false;
        }
    }
    if (noHideItems) hideItems->setEnabled(false);
}

void BasicLoggerWidget::saveConfig() {
    if (m_configPath.isEmpty()) {
        COBOT_LOG.notice("LoggerWin") << "Current Widget Create Without Config. Save Ignore.";
    } else {
        QJsonObject jsonConfig;
        if (loadJson(jsonConfig, m_configPath)) {
            QJsonObject loggerConfig;
            loggerConfig["EnableFilter"] = m_enableFilter;
            loggerConfig["EnableGUIFilterMenu"] = m_enableGUIFilterMenu;
            QJsonArray types, names;
            for (auto& iter : m_typeFilter) { if (!iter.second) types.push_back(iter.first); }
            for (auto& iter : m_nameFilter) { if (!iter.second) names.push_back(iter.first); }
            loggerConfig["Type"] = types;
            loggerConfig["Name"] = names;
            jsonConfig["LoggerWidgetSetup"] = loggerConfig;

            QJsonDocument configDoc(jsonConfig);
            auto ba = configDoc.toJson();
            QFile configFile(m_configPath);
            if (configFile.open(QIODevice::WriteOnly)) {
                configFile.write(ba);
            } else {
                COBOT_LOG.warning("LoggerWin") << configFile.errorString() << ", " << m_configPath;
            }
        }
    }
}

void BasicLoggerWidget::loadConfig() {
    if (m_configPath.isEmpty()) {
        COBOT_LOG.notice("LoggerWin") << "Current Widget Create Without Config. Load Ignore.";
    } else {
        setup(m_configPath);
    }
}

void BasicLoggerWidget::addConfigMenu(QMenu* menu) {
    QAction* action;

    auto cMenu = menu->addMenu(tr("Config"));
    cMenu->setFont(m_ctxMenuFont);

    action = cMenu->addAction(tr("Load"));
    connect(action, &QAction::triggered, this, &BasicLoggerWidget::loadConfig);

    action = cMenu->addAction(tr("Save"));
    connect(action, &QAction::triggered, this, &BasicLoggerWidget::saveConfig);
}


