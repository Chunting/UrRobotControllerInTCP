//
// Created by sail on 17-4-6.
//

#include <cobotsys_abstract_controller.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include "emptyWidget.h"
#include <QPushButton>
#include <QLayout>
#include <QVBoxLayout>
emptyWidget::emptyWidget() {
	//ui
    ui.setupUi(this);
    if (GlobalObjectFactory::instance()) {
        auto obj = GlobalObjectFactory::instance()->createObject("SimpleUiFactory, Ver 1.0", "BasicLoggerWidget");
        m_loggerWidget = std::dynamic_pointer_cast<QWidget>(obj);
    }
    auto boxLayout = new QVBoxLayout;
	boxLayout->setContentsMargins(QMargins());
	boxLayout->addWidget(m_loggerWidget.get());
	ui.logger->setLayout(boxLayout);

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
	QString sConfig = "CONFIG/force_guide_controller_config.json";
	if (sConfig.isEmpty()) {
		sConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
			QObject::tr("Get controller Config JSON file ..."),
			QString(FileFinder::getPreDefPath().c_str()),
			QObject::tr("JSON files (*.JSON *.json)"));
	}
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