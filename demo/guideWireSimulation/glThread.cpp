#include "glThread.h"
#include "glDrawer.h"
#include <QtWidgets\qmessagebox.h>
//#include <QMessageBox>

GLThread::GLThread(GLDrawer * gl):QThread()
{
	glw = gl;
}

GLThread::~GLThread()
{
	
}

void GLThread::run()
{
	while (true)
	{
		(glw->core)->Update(glw->timeslice, glw->avoid);
		QMessageBox::about(0, QString("update"), QString("update"));
		glw->update();
//		glw->updateGL();
	}
}

void GLThread::stop()
{
	if (this->isRunning())
		this->quit();
}




