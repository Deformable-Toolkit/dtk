#ifndef GUIDEWIRESIMULATION_H
#define GUIDEWIRESIMULATION_H

#include <QtGui/QMainWindow>
#include "ui_guidewiresimulation.h"
#include "ui_simulationParameters.h"
#include "DevicePhantomOmni.h"


class GLDrawer;
class QTimer;
class guideWireSimulation : public QMainWindow
{
	Q_OBJECT

public:
	guideWireSimulation(QWidget *parent = 0, Qt::WFlags flags = 0);
	~guideWireSimulation();

	public slots:
		void open();
		void simulationParams();
		void start();
		void pause();
		void quit();
		void center();
		void zoomOut();
		void zoomIn();
		void about();
		void howDoI();
		void apply();
		void cancel();
		void idle(); // equal glutIdleFunc
		
	
private:
	Ui::guideWireSimulationClass ui;
	Ui::SimulationParametersClass simuParamsWidget;
	QDialog  * SimulationParameterdialog;
	GLDrawer * OpenGLWidget ;

	QAction * fileOpenAction ;
	QAction * startAction;
	QAction * pauseAction;
	QAction * quitAction;
	QAction * centerAction ;
	QAction * zoomOutAction;
	QAction * zoomInAction ;
	QTimer * timer;
	int mTimes;

	// Phantom Omni
	DevicePhantomOmni::Ptr mDevicePhantomOmni;
	HapticDisplayState * mCurrentDisplayState;
	HapticDisplayState * mLastDisplayState;
	hduVector3Dd mPhantomOmniVelocity;
	hduVector3Dd mPhantomOmniTwistVelocity;
};

#endif // GUIDEWIRESIMULATION_H
