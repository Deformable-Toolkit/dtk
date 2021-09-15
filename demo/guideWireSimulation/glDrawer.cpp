#include <QtGui/qevent.h>
#include <fstream>
#include "cutHead.h"
#include "glDrawer.h"
#include "dtkStaticTriangleMeshWriter.h"

#define pi 3.1415926
// user operation
dtkT3<double> UpImpulse(0.0, 0.8, 0.0);
dtkT3<double> DownImpulse(0.0, -0.8, -0.0);
dtkT3<double> LeftImpulse(-6.0, 0.0, 0.0);
dtkT3<double> RightImpulse(6.0, 0.0, 0.0);

dtkT3<double> UpForce(0.0, 800.0 * 1.5, 0.0);
dtkT3<double> DownForce(0.0, -800.0 * 1.5, -0.0);
dtkT3<double> LeftForce(-1000.0, 0.0, 0.0);
dtkT3<double> RightForce(1000.0, 0.0, 0.0);

double leftTwist = -pi / 4;
double rightTwist = pi / 4;

// simulation time step
//using namespace dtk;
double timeslice = 0.01;

string avoidPointIDFile = "..\\data\\avoidPoint1.txt";
std::ofstream file;
vector<dtkID> avoidPoint;
dtkStaticTriangleMesh::Ptr triangleMesh;


	void GLDrawer::initializeGL()
	{
	
		// Set up the rendering context, define display lists etc.:
		glClearColor (1.0, 1.0, 1.0, 1.0);

		// 设置光源
		GLfloat light0_ambient [] = {0.2, 0.2, 0.2, 1.0};
		GLfloat light0_diffuse[] = {0.6, 0.6, 0.6, 1.0};
		GLfloat light0_specular[] = {0.4, 0.4, 0.4, 1.0};	

		GLfloat light1_ambient [] = {0.2, 0.2, 0.2, 1.0};
		GLfloat light1_diffuse[] = {0.6, 0.6, 0.6, 1.0};
		GLfloat light1_specular[] = {0.4, 0.4, 0.4, 1.0};	

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glShadeModel(GL_SMOOTH);

		glLightfv(GL_LIGHT0,GL_AMBIENT,light0_ambient);
		glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_diffuse);
		glLightfv(GL_LIGHT0,GL_SPECULAR,light0_specular);

		glLightfv(GL_LIGHT1,GL_AMBIENT,light1_ambient);
		glLightfv(GL_LIGHT1,GL_DIFFUSE,light1_diffuse);
		glLightfv(GL_LIGHT1,GL_SPECULAR,light1_specular);

		// 设置光照模型
		glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

		// 开启光源
		glEnable(GL_LIGHTING);
		//glEnable(GL_LIGHT0);

		// 开启深度测试等
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnableClientState( GL_VERTEX_ARRAY );
		glEnableClientState( GL_NORMAL_ARRAY );
		glEnable(GL_LINE_SMOOTH);
		glHint(GL_LINE_SMOOTH, GL_NICEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_BLEND);

	}

	void GLDrawer::resizeGL( int w, int h )
	{
		// setup viewport, projection etc.:
		glViewport (0, 0, (GLsizei) w, (GLsizei) h);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		gluPerspective(60, (double)w/(double)h,0.1,11000);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	void GLDrawer::paintGL()
	{
		glEnable(GL_NORMALIZE); 

		GLfloat lightPositionVascular[4] = { (eyePosition.x - centerPosition.x ) * xscale, (eyePosition.y - centerPosition.y ) * yscale,   (eyePosition.z - centerPosition.z ) * zscale  , 0};
		GLfloat lightPositionGuideWire[4] = { -(eyePosition.x - centerPosition.x ) * xscale, -(eyePosition.y - centerPosition.y ) * yscale,   -(eyePosition.z - centerPosition.z ) * zscale  , 0};

		glLightfv(GL_LIGHT0, GL_POSITION, lightPositionVascular);
		glLightfv(GL_LIGHT1, GL_POSITION, lightPositionGuideWire);
		
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// draw the scene:
		glPushMatrix();
		gluLookAt(eyePosition.x + 5 * (eyePosition.x - centerPosition.x ), eyePosition.y + 5 * (eyePosition.y - centerPosition.y ), eyePosition.z + 5 * (eyePosition.z - centerPosition.z ) , centerPosition.x, centerPosition.y, centerPosition.z, upVector.x, upVector.y, upVector.z);

		glTranslatef(xTranslate * screenXAxis.x, xTranslate * screenXAxis.y, xTranslate * screenXAxis.z);
		glTranslatef(yTranslate * upVector.x , yTranslate * upVector.y, yTranslate * upVector.z);
		glTranslatef(eyePosition.x, eyePosition.y, eyePosition.z);
		glRotated(yRotate , upVector.x, upVector.y, upVector.z);
		glRotated(xRotate, screenXAxis.x, screenXAxis.y, screenXAxis.z);
		glScaled(xscale, yscale, zscale);
		glTranslatef(-eyePosition.x, -eyePosition.y, -eyePosition.z);

		if (vascularRender != NULL)
			vascularRender->Draw();
		if (guideWireRender != NULL)
		{
			guideWireRender->Draw();
		}
		glPopMatrix();
		glEnd();
	}
	//滑轮滚动 改变视角大小
	void GLDrawer::wheelEvent ( QWheelEvent * e )
	{
		int deltaValue = e->delta() ;
		if (deltaValue > 0)
		{
			xscale *= 1.2;
			yscale *= 1.2;
			zscale *= 1.2;
		}
		else
		{
			xscale /= 1.2;
			yscale /= 1.2;
			zscale /= 1.2;
		}
		update();
		//updateGL();
	}

	void GLDrawer::keyPressEvent(QKeyEvent * e)
	{
		switch (e->key())
		{
		case 'w':
		case 'W':
			if (guideWirePoints != NULL)
				core->ApplyExternalForce(1, UpForce);
			break;
		case 's':
		case 'S':
			if (guideWirePoints != NULL)
				core->ApplyExternalForce(1, DownForce);
			break;
		case 'a':
		case 'A':
			if (guideWirePoints != NULL)
				core->ApplyExternalForce(1, LeftForce);
			break;
		case 'd':
		case 'D':
			if (guideWirePoints != NULL)
				core->ApplyExternalForce(1, RightForce);
			break;
		case '8':
			yTranslate += 4.0;
			update();
//			updateGL();
			break;
		case '5':
			yTranslate -= 4.0;
			update();
//			updateGL();
			break;
		case '4':
			xTranslate += 4.0;
			update();
//			updateGL();
			break;
		case '6':
			xTranslate -= 4.0;
			update();
//			updateGL();
			break;
		case 'l':
		case 'L':
			for (int i = 0; i < 10; i++)
				core->ApplyExternalTwist(1, leftTwist / 10);
			break;
		case 'r':
		case 'R':
			for (int i = 0; i < 10; i++)
				core->ApplyExternalTwist(1, rightTwist / 10);
			break;
		}
	}

	void GLDrawer::mouseMoveEvent ( QMouseEvent * e )
	{
		int changeX = e->x() - lastX;
		int changeY = e->y() - lastY;

		yRotate += changeX / 100.0;
		xRotate += changeY / 100.0;
		update();
//		updateGL();
	}

	void GLDrawer::mousePressEvent(QMouseEvent * e)
	{
		lastX = e->x();
		lastY = e->y();
	}


	void GLDrawer::initializeVascularModel()
	{
		core = controlCore::New(0.15);  // 0.1
		vascularRender = vascularOpenGLRender::New();

		// create vascular mass spring
		//使用四面体构造弹簧质点
		core->CreateTetraMassSpring(vascularFile.c_str(), 0, 4, 7000, 80, 0.99, 0.2, dtkT3<double>( 0, 0, 0 ));
		//crete vascular SurfaceMesh
		//core->CreateTriangleSurfaceMesh(vascularFile.c_str(), 0);
		// render the vascular 
		vascularRender->SetTriangleMesh( core->GetTriangleMesh(0) );

		//dtkStaticTriangleMeshWriter::Ptr triangleMeshWriter = dtkStaticTriangleMeshWriter::New();
		//triangleMeshWriter->SetInput(core->GetTriangleMesh(0));
		//triangleMeshWriter->SetFileName("..\\data\\vein0806version8.s3m");
		//triangleMeshWriter->Write();
	}

	void GLDrawer::initializeGWModel()
	{

 		guideWireRender = guideWireOpenGLRender::New();

		// avoid collision response of vascular's head and tail
		// frist step : read avoid from avoidpoint.txt
		std::ifstream fileReadAvoidPoint(avoidPointIDFile.c_str());
		dtkID avoidPointSize;
		fileReadAvoidPoint >> avoidPointSize;
		avoidPoint.resize(avoidPointSize);
		for (dtkID i = 0; i < avoidPointSize; i++)
		{
			fileReadAvoidPoint >> avoidPoint[i];			// netgen生成结果
			avoidPoint[i] = avoidPoint[i] - 1;             // because netgen pointID is bigger 1 more than .s3m
		}

		// second step: get avoid traingle id from triangleMesh
		cutHead cut(core->GetTriangleMesh(0), avoidPoint);
		const vector<dtkID3> & avoidTriangleRef = cut.AvoidTriangle();
		avoid = avoidTriangleRef;

		// construct guidewire
		// step 1: make the start point of guide wire
		dtkPoints::Ptr points = core->mTriangleMeshes[0]->GetPoints();
		GK::Point3 guidewireStartPoint = points->GetPoint(avoidPoint[0]);

		// step 2: set the guide wire direction
		dtkID triangleID;
		for(triangleID = 0; triangleID < avoid.size(); triangleID++)
		{
			if (avoid[triangleID].a == avoidPoint[0] || avoid[triangleID].a == avoidPoint[0] || avoid[triangleID] == avoidPoint[0])
				break;
		}

		triangleID = 0; //78
		//基尔霍夫杆 数据建立
		GK::Vector3 edge1 = points->GetPoint(avoid[triangleID].a) - points->GetPoint(avoid[triangleID].b);
		GK::Vector3 edge2 = points->GetPoint(avoid[triangleID].b) - points->GetPoint(avoid[triangleID].c);
		GK::Vector3 normal = - GK::Normalize(GK::CrossProduct(edge1, edge2)); 
		//各种需要调节的参数
		double bendAngleInterval = 0;		// pi / 6
		vector<double> tipOriginAngle(2, bendAngleInterval );
		dtkID guidewireSize = tipOriginAngle.size() + 3;  // 2
		double segIntervalLength = 1.0;
		double tipSegIntervalLength = 0.5 ;   // 1.0 / 2.0
		guideWirePoints = dtkPointsVector::New(guidewireSize);

		// create guide wire 
		GK::Vector3 baseVector(0, 1.0, 0);
		GK::Vector3 verticalVector = tipSegIntervalLength * GK::Normalize( GK::CrossProduct(normal, baseVector) );
		// 计算尖端的轴长部分
		double tipAxisLength = 0;
		//被注释掉 尖端没有轴长部分
		//for (dtkID i = 1; i <= tipOriginAngle.size(); i++)
		//{
		//	tipAxisLength += cos (i * bendAngleInterval);
		//}

		// 构造导丝尖端的弯曲部分
		for (dtkID i = tipOriginAngle.size() ; i <= tipOriginAngle.size();  i--)
		{
			if (i == tipOriginAngle.size())
				guideWirePoints->SetPoint(i, guidewireStartPoint - tipAxisLength * tipSegIntervalLength * normal);
			else
			{
				int times = tipOriginAngle.size() - i;
				guideWirePoints->SetPoint(i, guideWirePoints->GetPoint(i + 1)  \
					+ cos(times * bendAngleInterval) * tipSegIntervalLength * normal + sin(times * bendAngleInterval) \
					* verticalVector );
			}
		}

		for (dtkID i = tipOriginAngle.size() + 1; i < guidewireSize; i++)
		{
			// 构造导丝的体部
			guideWirePoints->SetPoint(i, guidewireStartPoint - (i - tipOriginAngle.size() + tipAxisLength * tipSegIntervalLength) * normal );	
		}

		//构造导丝
		core->CreateGuideWire(1, guideWirePoints, tipOriginAngle.size(), segIntervalLength, tipSegIntervalLength, tipOriginAngle);
		(core->mGuideWireMassPoints[1])->SetGuideWireStartPoint(guidewireStartPoint);
		(core->mGuideWireMassPoints[1])->SetGuideWireStartDirection(normal);

		// create collision response
		core->CreateCollisionResponse(0, 1, 2000);

		//set parameters to render the guide wire
		guideWireRender = guideWireOpenGLRender::New();
		guideWireRender->SetGuideWireMassPoints((core->mGuideWireMassPoints)[1]);
		guideWireRender->SetRadius(0.3);
		guideWireRender->SetBezCtrlPtsPercent(0.2);
		guideWireRender->SetBezPtsNumber(3);
	}

