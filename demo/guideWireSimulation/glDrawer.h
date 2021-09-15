#ifndef GLDRAWER_H
#define GLDRAWER_H



//#include "dtkError.h"
#include "controlCore.h"
#include "vascularOpenGLRender.h"
#include "guideWireOpenGLRender.h"

//#include<QGLWidget>
#include<qopenglwidget.h>

using namespace dtk;
using namespace std;

	class GLDrawer : public QOpenGLWidget
	{
		Q_OBJECT        // must include this if you use Qt signals/slots

	public:
		GLDrawer( QWidget *parent)//:QOpenGLWidget
			//: QGLWidget(parent)
		{
			xscale = 1.0;
			yscale = 1.0;
			zscale = 1.0;

			xRotate = 0;
			yRotate = 0;

			xTranslate = 0;
			yTranslate = 0;

			lastX = 0;
			lastY = 0;

			eyePosition = dtkT3<double>(0, 0, 0);
			centerPosition = dtkT3<double>(0, 0, -1);
			upVector = dtkT3<double>(0, 1, 0);
			screenXAxis = dtkT3<double>(1, 0, 0);

			leftRightLength = 200;
			bottomTopLength = 200;
			nearFarLength = 200;

			// set center times
			centerCount = 0;

			// set time slice
			timeslice = 0.03;

			avoid.resize(12);
			
		}


	public:
		// space transform
		double xscale;
		double yscale;
		double zscale;

		int xRotate;
		int yRotate;

		double xTranslate;
		double yTranslate;

		int lastX;
		int lastY;

		dtkT3<double> eyePosition;  // Specifies the position of the eye point. 
		dtkT3<double> centerPosition; // Specifies the position of the reference point.
		dtkT3<double> upVector; // Specifies the direction of the up vector.
		dtkT3<double> screenXAxis; // Specifies the direction of screen x axis.

		double leftRightLength; // orth left right
		double bottomTopLength; // orth up down
		double nearFarLength; // orth near far

		// model kernel
		controlCore::Ptr core;
		dtkPoints::Ptr guideWirePoints;

		// model render
		guideWireOpenGLRender::Ptr guideWireRender;
		vascularOpenGLRender::Ptr vascularRender;

		// vascular grid filename
		std::string vascularFile;

		dtkID centerCount;
		double timeslice;
		vector<dtkID3> avoid;

		// OpenGL
		void initializeGL();
		void resizeGL( int w, int h );
		void paintGL();

		// kernel model
		void initializeVascularModel();
		void initializeGWModel();

	protected:


		// User operation event response
		void wheelEvent ( QWheelEvent * e );
		void keyPressEvent(QKeyEvent * e);
		void mouseMoveEvent ( QMouseEvent * e );
		void mousePressEvent(QMouseEvent * e);

	};


#endif;
