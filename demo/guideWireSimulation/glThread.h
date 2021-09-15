#ifndef GLTHREAD_H
#define  GLTHREAD_H
#include <QThread>

class GLDrawer;
class GLThread : public QThread
{
	Q_OBJECT
public:
	GLThread(GLDrawer * glWidget);

public slots:
	void stop();

protected:
	void run();
	~GLThread();

private:
	GLDrawer * glw;
};
#endif