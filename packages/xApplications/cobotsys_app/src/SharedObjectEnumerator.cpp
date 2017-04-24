//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SharedObjectEnumerator.h"

SharedObjectEnumerator::SharedObjectEnumerator() {
    setupUi();
}

SharedObjectEnumerator::~SharedObjectEnumerator() {
}

bool SharedObjectEnumerator::initWidgetList() {
    if (GlobalObjectFactory::instance()) {
        m_widgetList.clear();
        m_comboBox->clear();
        auto factory_names = GlobalObjectFactory::instance()->getFactoryNames();
        for (auto& name : factory_names) {
            auto types = GlobalObjectFactory::instance()->getFactorySupportedNames(name);

            for (auto& type : types) {
                auto obj = GlobalObjectFactory::instance()->createObject(name, type);
                auto widget = std::dynamic_pointer_cast<QWidget>(obj);
                if (widget) {
                    m_widgetList.push_back({name, type});

                    QStringList data;
                    QString text;
                    text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                    data << name.c_str();
                    data << type.c_str();
                    m_comboBox->addItem(text, data);
                }
            }
        }
    }
    return (m_widgetList.size() != 0);
}

void SharedObjectEnumerator::setupUi() {
    m_pushButton = new QPushButton(this);
    m_comboBox = new QComboBox(this);

    m_pushButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
    m_pushButton->setText(tr("Create"));

    m_boxLayout = new QHBoxLayout;
    m_boxLayout->setContentsMargins(QMargins());
    m_boxLayout->addWidget(m_comboBox);
    m_boxLayout->addWidget(m_pushButton);
    setLayout(m_boxLayout);

    connect(m_pushButton, &QPushButton::released, this, &SharedObjectEnumerator::onPush);
}

void SharedObjectEnumerator::onPush() {
    QStringList obj_info = m_comboBox->currentData().toStringList();
    QString factory = obj_info.at(0);
    QString typen = obj_info.at(1);
    m_factory = factory;
    m_type = typen;
    accept();
}
