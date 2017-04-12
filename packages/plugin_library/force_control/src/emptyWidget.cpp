//
// Created by sail on 17-4-6.
//

#include <cobotsys_abstract_controller.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include "emptyWidget.h"

emptyWidget::emptyWidget() {

}

emptyWidget::~emptyWidget() {

}

bool emptyWidget::setup(const QString &configFilePath) {
	cobotsys::GlobalObjectFactory globalObjectFactory;

	std::shared_ptr<cobotsys::AbstractObject> pObject;

	pObject = globalObjectFactory.createObject("ForceGuideControllerFactory, Ver 1.0", "ForceGuideController");

	auto pController = std::dynamic_pointer_cast<cobotsys::AbstractController>(pObject);

	QString sConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
		QObject::tr("Get Robot Config JSON file ..."),
		QString(FileFinder::getPreDefPath().c_str()),
		QObject::tr("JSON files (*.JSON *.json)"));
	if (!sConfig.isEmpty())
		pController->setup(sConfig);

	pController->start();

    return true;
}
