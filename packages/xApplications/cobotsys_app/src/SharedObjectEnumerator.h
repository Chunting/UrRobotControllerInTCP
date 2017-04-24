//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_SHAREDOBJECTENUMERATOR_H
#define COBOTSYS_SHAREDOBJECTENUMERATOR_H

#include <QDialog>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <cobotsys_abstract_widget.h>
#include <cobotsys_global_object_factory.h>
#include <string>

using namespace cobotsys;

class SharedObjectEnumerator : public QDialog {
Q_OBJECT
public:
    SharedObjectEnumerator();
    virtual ~SharedObjectEnumerator();

    bool initWidgetList();

    const QString& getFactory() const { return m_factory; }

    const QString& getType() const { return m_type; }

protected:
    void setupUi();

    void onPush();
protected:
    struct SharedObjectKey {
        std::string factory;
        std::string type;
    };

    std::vector<SharedObjectKey> m_widgetList;
    QHBoxLayout* m_boxLayout;
    QComboBox* m_comboBox;
    QPushButton* m_pushButton;

    QString m_factory;
    QString m_type;
};


#endif //COBOTSYS_SHAREDOBJECTENUMERATOR_H
