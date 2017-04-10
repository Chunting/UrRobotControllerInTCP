//
// Created by 潘绪洋 on 17-4-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_global_object_factory.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_gui_logger_highlighter.h>
#include <extra2.h>
#include "SimpleWidgetViewer.h"
#include <QSysInfo>

SimpleWidgetViewer::SimpleWidgetViewer() {
    ui.setupUi(this);
    m_closer = new WidgetCloser;

    connect(ui.btnCreate, &QPushButton::released, this, &SimpleWidgetViewer::actionCreateWidget);
    connect(ui.btnCreateNoJson, &QPushButton::released, this, &SimpleWidgetViewer::actionCreateWidgetNoJson);
    connect(ui.btnClear, &QPushButton::released, this, &SimpleWidgetViewer::actionClear);
    connect(m_closer, &WidgetCloser::widgetClosed, this, &SimpleWidgetViewer::resetCurObj);

    createTextLogUi();
}

SimpleWidgetViewer::~SimpleWidgetViewer() {
    COBOT_LOG.clrFilter(this);
    INFO_DESTRUCTOR(this);
}

bool SimpleWidgetViewer::setup(const QString& configFilePath) {
    refreshWidgetList();
    return true;
}

void SimpleWidgetViewer::refreshWidgetList() {
    if (!GlobalObjectFactory::instance()) return;

    ui.comboBox->clear();
    auto factory_names = GlobalObjectFactory::instance()->getFactoryNames();
    for (auto& name : factory_names) {
        auto types = GlobalObjectFactory::instance()->getFactorySupportedNames(name);

        for (auto& type : types) {
            auto obj = GlobalObjectFactory::instance()->createObject(name, type);
            auto widget = std::dynamic_pointer_cast<QWidget>(obj);
            if (widget) {
                QStringList data;
                QString text;
                text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                data << name.c_str();
                data << type.c_str();
                ui.comboBox->addItem(text, data);
            }
        }
    }
    COBOT_LOG.addFilter(this, [=](const std::string& e, const std::string& m) { appendText(e, m); });
}

void SimpleWidgetViewer::actionCreateWidget() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.comboBox->count() == 0)
        return;

    QString robotConfig = QFileDialog::getOpenFileName(this,
                                                       tr("Get Widget Config JSON file ..."),
                                                       QString(FileFinder::getPreDefPath().c_str()),
                                                       tr("JSON files (*.JSON *.json)"));

    QStringList obj_info = ui.comboBox->currentData().toStringList();
    QString factory = obj_info.at(0);
    QString typen = obj_info.at(1);

    m_pWidget = GlobalObjectFactory::instance()->createObject(factory, typen);
    auto widget = std::dynamic_pointer_cast<QWidget>(m_pWidget);
    if (widget) {
        widget->installEventFilter(m_closer);
        if (robotConfig.isEmpty()) {
            widget->show();
        } else {
            if (m_pWidget->setup(robotConfig)) {
                widget->show();
            }
        }
    }
}

void SimpleWidgetViewer::resetCurObj() {
    m_pWidget = nullptr;
}

void SimpleWidgetViewer::updateTextToUI() {
    if (m_cachedMessage.size()) {
        m_textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
        m_textCursor.insertText(m_cachedMessage);

        ui.plainTextEdit->setTextCursor(m_textCursor);
        m_cachedMessage.clear();
    }
}

void SimpleWidgetViewer::appendText(const std::string& entry, const std::string& message) {
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
            m_cachedMessage += QString::fromLocal8Bit(message.c_str()) + "\n";
        }
    } else {
        formater.section_width = COBOT_LOG.prefixWidth();
        formater.section(entry);
        formater.oss << " " << message;

        QString final_message = QString::fromLocal8Bit(oss.str().c_str()).trimmed() + "\n";
        m_cachedMessage += final_message;
    }
}

void SimpleWidgetViewer::createTextLogUi() {
    QFont font = getMonospaceFont();
    ui.plainTextEdit->setReadOnly(true);
    ui.plainTextEdit->setFont(font);
    ui.plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);

    m_textCursor = ui.plainTextEdit->textCursor();

    cobotsys::gui::LoggerHighlighter::highlightEditorWithDefaultStyle(ui.plainTextEdit->document());

    m_editUpdateTimer = new QTimer(this);
    m_editUpdateTimer->setInterval(1000 / 30); // 30Hz
    connect(m_editUpdateTimer, &QTimer::timeout, this, &SimpleWidgetViewer::updateTextToUI);
    m_editUpdateTimer->start();
}

void SimpleWidgetViewer::actionClear() {
    ui.plainTextEdit->clear();
}

void SimpleWidgetViewer::actionCreateWidgetNoJson() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.comboBox->count() == 0)
        return;

    QStringList obj_info = ui.comboBox->currentData().toStringList();
    QString factory = obj_info.at(0);
    QString typen = obj_info.at(1);

    m_pWidget = GlobalObjectFactory::instance()->createObject(factory, typen);
    auto widget = std::dynamic_pointer_cast<QWidget>(m_pWidget);
    if (widget) {
        widget->installEventFilter(m_closer);
        widget->show();
    }
}
