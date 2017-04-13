//
// Created by sail on 17-4-6.
//

#include <cobotsys_abstract_controller.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include "emptyWidget.h"
#include <QPushButton>

emptyWidget::emptyWidget() {
	//ui
	ui.setupUi(this);

	connect(ui.startButton, &QPushButton::released, this, &emptyWidget::startController);
	connect(ui.stopButton, &QPushButton::released, this, &emptyWidget::stopController);
}

emptyWidget::~emptyWidget() {

}

bool emptyWidget::setup(const QString &configFilePath) {

	//controller
	std::shared_ptr<cobotsys::AbstractObject> pObject;

	pObject = GlobalObjectFactory::instance()->createObject("ForceGuideControllerFactory, Ver 1.0", "ForceGuideController");

	m_ptrController = std::dynamic_pointer_cast<cobotsys::AbstractController>(pObject);

	QString sConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
		QObject::tr("Get Robot Config JSON file ..."),
		QString(FileFinder::getPreDefPath().c_str()),
		QObject::tr("JSON files (*.JSON *.json)"));
	if (!sConfig.isEmpty())
		m_ptrController->setup(sConfig);

    return true;
}

void emptyWidget::startController() {
	if (m_ptrController) {
		m_ptrController->start();
	}
}

void emptyWidget::stopController() {
	if (m_ptrController) {
		m_ptrController->stop();
	}
}