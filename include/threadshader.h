#ifndef THREADSHADER_H
#define THREADSHADER_H

#include <QMutex>
#include <QThread>
#include <QDebug>
#include <QMap>
#include <QVariant>

#include "activityvisualizer.h"
#include "diagrambox.h"
#include "shaderwidget.h"
#include "shaderproxy.h"
#include "shadermovebar.h"
#include "visufunctions.h"
#include "types.h"

class DiagramScene;

/**
 * @brief The ThreadShader class inherits from QThread class. It limits the CPU use of OpenGL 3d visu
 * by unlocking a mutex each time a certain amount of time has passed. Every time the mutex is unlocked,
 * the ShaderProxy object is then allowed to load a new frame.
 */

class ThreadShader: public QThread
{

	Q_OBJECT

public :
	ThreadShader(DiagramBox* box, VisuType type, QMap<QString, QVariant> parameters);
	~ThreadShader();

	ShaderProxy *proxy() const;

	ShaderMoveBar *shaderMoveBar() const;

	void setRunning(bool running);

protected:
	void run() override;

private :
	QMutex m_mutex;
	int m_delay = 30;

	ShaderWidget *m_widget;
	ShaderMoveBar *m_shaderMoveBar;
	ShaderProxy *m_proxy;

	bool m_running = true; //While this boolean is set as true, the thread is running. When it is set as false, it finishes its process.
};
#endif // THREADSHADER_H
