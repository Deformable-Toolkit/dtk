#include "guidewiresimulation.h"

//#include <QtGui/QApplication>
//#include <QtGui/QGraphicsView>
#include "glDrawer.h"
#include<qapplication.h>
#include<qgraphicsview.h>
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	guideWireSimulation w;
	w.show();
	
	//QGraphicsView view;
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	//view.show();

	/*QGraphicsScene scene;
	QGraphicsView view(&scene);
	view.setViewport(new GLDrawer(0));
	view.show();*/
	
	//GLDrawer ddd(0);
	//ddd.updateGL();
	//ddd.show();
	return a.exec();
}
