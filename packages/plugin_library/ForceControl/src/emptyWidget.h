//
// Created by sail on 17-4-6.
//

#ifndef PROJECT_EMPTYWIDGET_H
#define PROJECT_EMPTYWIDGET_H

#include <cobotsys_abstract_widget.h>
#include <cobotsys_abstract_controller.h>
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include "ui_emptyWidget.h"

using namespace cobotsys;

class emptyWidget: public AbstractWidget  {
Q_OBJECT
public:
    emptyWidget();
    virtual ~emptyWidget();

    virtual bool setup(const QString& configFilePath);

	void startController();
	void stopController();

private:
	Ui::emptyWidget ui;
	std::shared_ptr<AbstractController> m_ptrController;
	std::shared_ptr<QWidget> m_loggerWidget;
};


#endif //PROJECT_EMPTYWIDGET_H
