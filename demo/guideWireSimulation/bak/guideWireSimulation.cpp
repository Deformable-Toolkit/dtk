#include "guidewiresimulation.h"
#include "glDrawer.h"
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QLabel>



// global variable

guideWireSimulation::guideWireSimulation(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	// setup ui
	SimulationParameterdialog = new QDialog(this);
	simuParamsWidget.setupUi(SimulationParameterdialog);
	ui.setupUi(this);
	this->setWindowIcon(QIcon(":/Resources/igst.jpg"));
	this->setWindowTitle(QString("real-time simulate guide wire in minimally invasive vascular interventions"));

	OpenGLWidget = new GLDrawer(ui.centralWidget);
	ui.gridLayout->addWidget(OpenGLWidget);

	// set toolbar
	fileOpenAction = new QAction(tr("&File"), this); 
	fileOpenAction->setStatusTip(tr("Load  vascular netgrid.")); 
	fileOpenAction->setIcon(QIcon(":/Resources/load_vascular.png")); 
	connect(fileOpenAction, SIGNAL(triggered()), this, SLOT(open())); 

	startAction = new QAction(tr("&Start"), this); 
	startAction->setStatusTip(tr("Start simulation.")); 
	startAction->setIcon(QIcon(":/Resources/start.png")); 
	connect(startAction, SIGNAL(triggered()), this, SLOT(start())); 

	pauseAction = new QAction(tr("&Pause"), this); 
	pauseAction->setStatusTip(tr("Pause  simulation.")); 
	pauseAction->setIcon(QIcon(":/Resources/pause.png")); 
	connect(pauseAction, SIGNAL(triggered()), this, SLOT(pause())); 


	quitAction = new QAction(tr("&Quit"), this); 
	quitAction->setStatusTip(tr("Quit  simulation.")); 
	quitAction->setIcon(QIcon(":/Resources/quit.png")); 
	connect(quitAction, SIGNAL(triggered()), this, SLOT(quit())); 

	centerAction = new QAction(tr("&Center"), this); 
	centerAction->setStatusTip(tr("Place vasulcar in center.")); 
	centerAction->setIcon(QIcon(":/Resources/center.png")); 
	connect(centerAction, SIGNAL(triggered()), this, SLOT(center())); 

	zoomOutAction = new QAction(tr("&Zoom Out"), this); 
	zoomOutAction->setStatusTip(tr("Zoom out.")); 
	zoomOutAction->setIcon(QIcon(":/Resources/zoom-out.png")); 
	connect(zoomOutAction, SIGNAL(triggered()), this, SLOT(zoomOut())); 

	zoomInAction = new QAction(tr("&Zoom In"), this); 
	zoomInAction->setStatusTip(tr("Zoom in.")); 
	zoomInAction->setIcon(QIcon(":/Resources/zoom-in.png")); 
	connect(zoomInAction, SIGNAL(triggered()), this, SLOT(zoomIn())); 


	ui.toolBar->addAction(fileOpenAction);
	ui.toolBar->addAction(startAction);
	ui.toolBar->addAction(pauseAction);
	ui.toolBar->addAction(quitAction);
	ui.toolBar->addAction(centerAction);
	ui.toolBar->addAction(zoomOutAction);
	ui.toolBar->addAction(zoomInAction);

	// set focus
	OpenGLWidget->setFocusPolicy(Qt::StrongFocus);


	// set menu bar connect 
	connect(ui.actionFileOpen, SIGNAL(triggered()), this, SLOT(open()));
	connect(ui.actionSimulationParams, SIGNAL(triggered()), this, SLOT(simulationParams()));
	connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
	connect(ui.actionCenter, SIGNAL(triggered()), this, SLOT(center()));
	connect(ui.actionHowDoI, SIGNAL(triggered()), this, SLOT(howDoI()));
	connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(quit()));
	connect(ui.actionStart, SIGNAL(triggered()), this, SLOT(start()));
	connect(ui.actionPause, SIGNAL(triggered()), this, SLOT(pause()));
	connect(ui.actionZoomIn, SIGNAL(triggered()), this, SLOT(zoomIn()));
	connect(ui.actionZoomOut, SIGNAL(triggered()), this, SLOT(zoomOut()));

	// set simulation parameters connect
	connect(simuParamsWidget.horizontalSlidergwStiff, SIGNAL(valueChanged(int)), simuParamsWidget.spinBoxgwStiff, SLOT(setValue(int)));
	connect(simuParamsWidget.horizontalSliderVascularStiff, SIGNAL(valueChanged(int)), simuParamsWidget.spinBoxVascularStiff, SLOT(setValue(int)));

	connect(simuParamsWidget.spinBoxgwStiff, SIGNAL(valueChanged(int)), simuParamsWidget.horizontalSlidergwStiff, SLOT(setValue(int)));
	connect(simuParamsWidget.spinBoxVascularStiff, SIGNAL(valueChanged(int)), simuParamsWidget.horizontalSliderVascularStiff, SLOT(setValue(int)));

	connect(simuParamsWidget.pushButtonApply, SIGNAL(pressed()), this, SLOT(apply()));
	connect(simuParamsWidget.pushButtonCancel, SIGNAL(pressed()), this, SLOT(cancel()));

	// set toolbar and menubar status
	startAction->setEnabled(false);
	ui.actionStart->setEnabled(false);

	// create new phantom omni ptr and initiation
	mDevicePhantomOmni = DevicePhantomOmni::New();
	mDevicePhantomOmni->InitHD();
	mCurrentDisplayState = mDevicePhantomOmni->GetCurrentDisplayState();
	mLastDisplayState = mDevicePhantomOmni->GetLastDisplayState();
	mPhantomOmniVelocity = hduVector3Dd(0, 0, 0);
	mPhantomOmniTwistVelocity = hduVector3Dd(0, 0, 0);
	mTimes = 0;		
}

void guideWireSimulation::open()
{
	QFileDialog* fd = new QFileDialog(this);
	fd->resize(600,400);  
	fd->setFilter( "*.s3m *.S3M *.S3m *.s3M");
	fd->setViewMode(QFileDialog::List);  
	fd->setDirectory(QDir(QString("..//data")));
	if ( fd->exec() == QDialog::Accepted )  
	{
		OpenGLWidget->vascularFile =  (fd->selectedFiles()[0]).toStdString();
		OpenGLWidget->initializeVascularModel();
		OpenGLWidget->centerCount = 0;
		center();
		startAction->setEnabled(true);
		ui.actionStart->setEnabled(false);
	}
	fd->close();
	delete fd;
}

void guideWireSimulation::start()
{
	startAction->setEnabled(false);
	ui.actionStart->setEnabled(false);
	fileOpenAction->setEnabled(false);
	ui.actionFileOpen->setEnabled(false);

	OpenGLWidget->initializeGWModel();
	OpenGLWidget->updateGL();

	timer = new QTimer(this);
	timer->start(20);
	connect(timer, SIGNAL(timeout()), this, SLOT(idle()));

}

void guideWireSimulation::idle()
{
	// 获取Phantom Omni的状态信息，计算Phantom Omni手柄的速度
	mDevicePhantomOmni->GetCurrentDisplayState();

	if (mCurrentDisplayState->button1)
	{
		// 计算用户给导丝的外力
		mPhantomOmniVelocity = (mCurrentDisplayState->position - mLastDisplayState->position) * 200.0; 
		(OpenGLWidget->core)->ApplyExternalForce(1, dtkT3<double>(0, -mPhantomOmniVelocity[2], 0) );
	}
	else
	{
		(OpenGLWidget->core)->ApplyExternalForce(1, dtkT3<double>(0, 0, 0) );
	}

	// 计算用户给导丝的扭曲力
	if (mCurrentDisplayState->button2)
	{
		mTimes ++;
		mPhantomOmniTwistVelocity += mCurrentDisplayState->gimbalAngle - mLastDisplayState->gimbalAngle;
		if (mTimes % 10 == 0)
		{
			(OpenGLWidget->core)->ApplyExternalTwist(1, mPhantomOmniTwistVelocity[2]);
			mTimes = 0;
			mPhantomOmniTwistVelocity = hduVector3Dd(0, 0, 0);
		}
	}
	
	(OpenGLWidget->core)->Update(OpenGLWidget->timeslice, OpenGLWidget->avoid);
	OpenGLWidget->updateGL();

	// 将用户受到的外力实时传递给力反馈Phantom Omni
	dtkT3<double> dtkForce = (OpenGLWidget->core)->GetHapticTranslationForce();
	if (dtkForce.y > 0)
		dtkForce =  dtkForce + (OpenGLWidget->core)->GetHapticCollisionForce();
	
	hduVector3Dd hduForce = hduVector3Dd(dtkForce.x, dtkForce.z, dtkForce.y);
	mDevicePhantomOmni->SetHapticForce(hduForce, 50);
}

void guideWireSimulation::pause()
{

}


void guideWireSimulation::quit()
{
	QMessageBox messagebox(QString("Quit Program?"), QString("Do you real want to quit program?"), QMessageBox::Information, QMessageBox::Yes, QMessageBox::NoButton, QMessageBox::No, this);
	//messagebox.show();
	
	if (messagebox.exec() == QMessageBox::No)
		return;
	else
	{
		this->close();
	}
}

void guideWireSimulation::center()
{

	OpenGLWidget->xRotate = 0;
	OpenGLWidget->yRotate = 0;
	OpenGLWidget->xscale = 1;
	OpenGLWidget->yscale = 1;
	OpenGLWidget->zscale = 1;
	OpenGLWidget->xTranslate = 0;
	OpenGLWidget->yTranslate = 0;

	if (OpenGLWidget->centerCount == 0)
	{
		OpenGLWidget->centerCount = 1;
		// compute center position
		dtkPoints::Ptr vascularPoints = (OpenGLWidget->core->mTriangleMeshes[0])->GetPoints();
		double xmin, xmax; 
		double ymin, ymax;
		double zmin, zmax;
		xmin = ymin = zmin = 10000;
		xmax = ymax = zmax = -10000;
		dtkID size = vascularPoints->GetNumberOfPoints();
		GK::Point3 point;
		for (dtkID i = 0; i < size; i++)
		{
			point = vascularPoints->GetPoint(i);

			if (point.x() < xmin)
				xmin = point.x();

			if (point.x() > xmax)
				xmax = point.x();

			if (point.y() < ymin)
				ymin = point.y();

			if (point.y() > ymax)
				ymax = point.y();

			if (point.z() < zmin)
				zmin = point.z();

			if (point.z() > zmax)
				zmax = point.z();
		}

		double maxAxis = xmax - xmin;
		int indexAxis = 0; // 0:x, 1:y, 2:z
		if (ymax - ymin > maxAxis)
		{
			maxAxis = ymax - ymin;
			indexAxis = 1;
		}
		if (zmax - zmin > maxAxis)
		{
			maxAxis = zmax - zmin;
			indexAxis = 2;
		}

		if (indexAxis == 0)
		{
			OpenGLWidget->upVector = dtkT3<double>(1, 0, 0);
			OpenGLWidget->eyePosition = dtkT3<double>((xmin + xmax) / 2, ymin , (zmin + zmax) / 2);
			OpenGLWidget->centerPosition = dtkT3<double>((OpenGLWidget->eyePosition).x, ymax, (OpenGLWidget-> eyePosition).z);
			
			OpenGLWidget->screenXAxis = dtkT3<double>(0, 0, 1);

		}
		else if (indexAxis == 1)
		{
			OpenGLWidget->upVector = dtkT3<double>(0, 1, 0);
			OpenGLWidget->eyePosition = dtkT3<double>((xmin + xmax) / 2, (ymin + ymax) / 2, zmin);

			OpenGLWidget->centerPosition = dtkT3<double>((OpenGLWidget->eyePosition).x, (OpenGLWidget->eyePosition).y,  zmax);

			OpenGLWidget->screenXAxis = dtkT3<double>(1, 0, 0);

		}
		else
		{
			OpenGLWidget->upVector = dtkT3<double>(0, 0, 1);
			OpenGLWidget->eyePosition = dtkT3<double>(xmin , (ymin + ymax) / 2, (zmin + zmax) / 2);
			OpenGLWidget->centerPosition = dtkT3<double>((xmax, OpenGLWidget->eyePosition).y, (OpenGLWidget->eyePosition).z);

			OpenGLWidget->screenXAxis = dtkT3<double>(0, 1, 0);
		}

	}
	

	OpenGLWidget->updateGL();

}

void guideWireSimulation::zoomOut()
{
	OpenGLWidget->xscale *= 1.2;
	OpenGLWidget->yscale *= 1.2;
	OpenGLWidget->zscale *= 1.2;
	OpenGLWidget->updateGL();
}

void guideWireSimulation::zoomIn()
{
	OpenGLWidget->xscale /= 1.2;
	OpenGLWidget->yscale /= 1.2;
	OpenGLWidget->zscale /= 1.2;
	OpenGLWidget->updateGL();
}

void guideWireSimulation::simulationParams()
{
	SimulationParameterdialog->exec();
}

void guideWireSimulation::about()
{
	QString aboutString("This is real-time simulate guide wire in minimally invasive vascular interventions \n ");
	QMessageBox messagebox(QString("About"), aboutString, QMessageBox::Information, QMessageBox::NoButton, QMessageBox::NoButton, QMessageBox::NoButton, this);
	messagebox.exec();
}

void guideWireSimulation::howDoI()
{
	QString operationGuide = QString("push guide wire :                     w\n");
	operationGuide = operationGuide + QString("pull  guide wire :                      s\n");
	operationGuide = operationGuide + QString("left turn guide wire tip :           a\n");
	operationGuide = operationGuide + QString("right turn guide wire tip :         d\n");
	operationGuide = operationGuide + QString("translate up view frustum :      5\n");
	operationGuide = operationGuide + QString("translate down view frustum : 8\n");
	operationGuide = operationGuide + QString("translate left view frustum :     7\n");
	operationGuide = operationGuide + QString("translate right view frustum :   9\n");

	QMessageBox messagebox(QString("How Do I"), operationGuide, QMessageBox::Information, QMessageBox::NoButton, QMessageBox::NoButton, QMessageBox::NoButton, this);
	messagebox.exec();

}

void guideWireSimulation::apply()
{
	// set simulation parameters

	// close the dialog
	SimulationParameterdialog->close();
}

void guideWireSimulation::cancel()
{
	// close the dialog
	SimulationParameterdialog->close();
}

guideWireSimulation::~guideWireSimulation()
{
	
}

