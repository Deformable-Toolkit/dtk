#include "guideWireOpenGLRender.h"
#define pi 3.1415926

namespace dtk
{
	void guideWireOpenGLRender::SetGuideWireMassPoints(guideWire::Ptr ptr)
	{
		mGuideWireMassPoints = ptr;
	}
	void guideWireOpenGLRender::Draw()
	{
		Update();
		if ( ! mGuideWirePoints.get() )
			return ;


		glClearDepth(1.0);
		glClear(GL_DEPTH_BUFFER_BIT);
		//glDepthMask(GL_TRUE);
	
		// 把血管的材质属性修改成导丝的材质属性
		GLfloat mGuidewire_mat_ambient[4] = {0.75, 0.75, 0.75, 1.0};
		GLfloat mGuidewire_mat_diffuse[4] = {0.3, 0.3, 0.3, 1.0};
		GLfloat mGuidewire_mat_specular[4] = {1.0, 1.0, 1.0, 1.0};
		GLfloat mGuidewire_mat_shininess = 1;

		glMaterialfv(GL_FRONT, GL_AMBIENT, mGuidewire_mat_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mGuidewire_mat_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, mGuidewire_mat_specular);
		glMaterialf(GL_FRONT, GL_SHININESS, mGuidewire_mat_shininess);
		
		// enable blend
		//glEnable(GL_BLEND);
		glEnable(GL_LIGHT1);
		glDisable(GL_LIGHT0);
		glShadeModel(GL_SMOOTH);
	
		// define guide wire bject
		GLUquadric * qobj;
		qobj = gluNewQuadric();
		gluQuadricDrawStyle(qobj, GLU_FILL);
		gluQuadricNormals(qobj, GLU_SMOOTH);
		//gluQuadricOrientation(qobj, GLU_INSIDE);


		GK::Point3 point;
		size_t j;
		for (j = 0; j < mGuideWirePoints->GetNumberOfPoints() - 1; j++)
		{
			RenderCylinder(qobj, mGuideWirePoints->GetPoint(j), mGuideWirePoints->GetPoint(j + 1), mGuideWireRadius, 10, 10);
		}

		//glEnable(GL_DEPTH_TEST);
		for (j = 0; j < mGuideWirePoints->GetNumberOfPoints() - 1; j++)
		{
			RenderSphere(qobj, mGuideWirePoints->GetPoint(j), mGuideWireRadius * 0.96, 10, 10);
		}

		//glDisable(GL_BLEND);

	}

	void guideWireOpenGLRender::RenderCylinder(GLUquadricObj * qObj, const GK::Point3 & startCenterPoint, const GK::Point3 & endCenterPoint, double radius, int slices, int stacks)
	{
		// 计算圆柱体的中心位置
		GK::Vector3 cylinderDirection = endCenterPoint - startCenterPoint;
		GK::Vector3 originCylinderDirection = GK::Vector3(0, 0, 1.0);
		GK::Vector3 rotateAxis = GK::CrossProduct(originCylinderDirection, cylinderDirection);
		double rotateAngle = acos(GK::DotProduct(cylinderDirection , originCylinderDirection) / GK::Length(cylinderDirection) ) / pi * 180;
		GK::Point3 midCenterPoint = startCenterPoint + cylinderDirection / 2.0 ;

		// 矩阵变换思路: 首先进行旋转，然后平移 (是利用世界坐标系)
		glPushMatrix();
		glTranslatef( startCenterPoint.x(), startCenterPoint.y(), startCenterPoint.z() );
		glRotatef( rotateAngle, rotateAxis.x(), rotateAxis.y(), rotateAxis.z() );
		gluCylinder(qObj, radius, radius , GK::Length(cylinderDirection), slices, stacks);
		glPopMatrix();
	}

	void guideWireOpenGLRender::RenderSphere(GLUquadricObj * qObj, const GK::Point3 & sphereCenter, double radius, int slices, int stacks)
	{
		glPushMatrix();
		glTranslatef(sphereCenter.x(), sphereCenter.y(), sphereCenter.z());
		gluSphere(qObj, radius, slices, stacks);
		glPopMatrix();
	}

	void guideWireOpenGLRender::Update()
	{	
		mGuideWirePoints = mGuideWireMassPoints->GetPoints();
		if( mGuideWirePoints.get() )
		{
			mPoints.clear();
			for (dtkID i = 0; i < mGuideWirePoints->GetNumberOfPoints(); i++)
			{		
				GK::Point3 point = mGuideWirePoints->GetPoint(i);
				mPoints.push_back(dtkT3<double>(point.x(), point.y(), point.z()));
			}
		}
	}

	void guideWireOpenGLRender::binomialCoeffs (int n, std::vector<int> & coeffs)
	{
		//求二项式系数 用于bezer曲线
		coeffs.resize(n+1);
		int k, j;
		for (k = 0; k <=n; k++)
		{
			coeffs[k] = 1;
			for (j = n; j >= k+1; j--)
				coeffs[k] *= j;
			for (j = n - k; j >= 2; j--)
				coeffs[k] /= j;
		}
	}

	void guideWireOpenGLRender::computeBezPt(int nBeztPts, std::vector<dtkT3<double>> & bezPts, int nCtrlPts,  const std::vector<dtkT3<double>>& ctrlPts)
	{
		int Coeffs[4] = {1, 0, 0, 1};
		if (nCtrlPts == 3)
		{
			Coeffs[1] = 2;
			Coeffs[2] = 1;
			Coeffs[3] = 0;
		}
		if (nCtrlPts == 4)
		{
			Coeffs[1] = 3;
			Coeffs[2] = 3;
		}
		//binomialCoeffs(nCtrlPts - 1, Coeffs);
		bezPts.resize(nBeztPts);
		for (int i = 0; i < nBeztPts; i++)
		{
			bezPts[i] = dtkT3<double>(0, 0, 0);
			for (int j = 0; j < nCtrlPts; j++)
			{
				bezPts[i].x += Coeffs[j] * ctrlPts[j].x * pow(i * 1.0 / nBeztPts, j) * pow(1- i * 1.0 / nBeztPts, nCtrlPts -1 - j);
				bezPts[i].y += Coeffs[j] * ctrlPts[j].y * pow(i * 1.0 / nBeztPts, j) * pow(1- i * 1.0 / nBeztPts, nCtrlPts -1 - j);
				bezPts[i].z += Coeffs[j] * ctrlPts[j].z * pow(i * 1.0 / nBeztPts, j) * pow(1- i * 1.0 / nBeztPts, nCtrlPts -1 - j);
			}
		}
	}

	void guideWireOpenGLRender::computeCtrlPt(double percent, const std::vector<dtkT3<double>> points, std::vector<std::vector<dtkT3<double>> > &ctrlPts)
	{
		int size = points.size();
		ctrlPts.resize(size - 1);
		// compute the first segment with three points
		ctrlPts[0].resize(3);
		ctrlPts[0][0] = points[0];
		ctrlPts[0][1] = points[1] -  percent * (points[2] - points[0]);
		ctrlPts[0][2] = points[1];

		for (int i = 1; i < size - 2; i++)
		{
			ctrlPts[i].resize(4);
			ctrlPts[i][0] = points[i];
			ctrlPts[i][1] = points[i] + percent * (points[i+1] - points[i-1]);
			ctrlPts[i][2] = points[i + 1] - percent * (points[i + 2] - points[i]);
			ctrlPts[i][3] = points[i + 1];
		}

		// compute the last segment with three points
		ctrlPts[size - 2].resize(3);
		ctrlPts[size-2][0] = points[size-2];
		ctrlPts[size-2][1] = points[size-2 ] + percent * (points[size-1] - points[size-3]);
		ctrlPts[size-2][2] = points[size-1];
	}

	guideWireOpenGLRender::guideWireOpenGLRender()
	{
		mGuideWireRadius = 0.0;
		mBezCtrlPtsPercent = 0.0;
		mBezPtsNumber = 0;
	}

	guideWireOpenGLRender::~guideWireOpenGLRender()
	{
		// nothing

	}

	void guideWireOpenGLRender::SetRadius(double radius)
	{
		assert(radius > 0);
		mGuideWireRadius = radius;
	}

	void guideWireOpenGLRender::SetBezPtsNumber(size_t bezPtsNumber)
	{
		mBezPtsNumber = bezPtsNumber;
	}

	void guideWireOpenGLRender::SetBezCtrlPtsPercent(double percent)
	{
		assert(percent > 0 && percent < 1.0);
		mBezCtrlPtsPercent = percent;
	}
}