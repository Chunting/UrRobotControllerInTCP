//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCEGUIDECONTROLLER_H
#define PROJECT_FORCEGUIDECONTROLLER_H

#include <mutex>
#include <cobotsys_abstract_controller.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class ForceGuideController : public QObject, public AbstractController {
	Q_OBJECT
public:
	ForceGuideController();
	virtual ~ForceGuideController();

	virtual bool setup(const QString& configFilePath);
	virtual bool start();
	virtual void pause();
	virtual void stop();
protected:
	bool m_bcontrol;

};


#endif //PROJECT_FORCEGUIDECONTROLLER_H
