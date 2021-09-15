#ifndef GUIDEWIREOPENGLRENDER_H
#define  GUIDEWIREOPENGLRENDER_H
#include "guideWire.h"
#include<gl/freeglut.h>
//#include <GL/glew.h>
#include <vector>
#include "dtkPoints.h"
#include "dtkTx.h"



namespace dtk
{
	class guideWireOpenGLRender
	{
	public:
		typedef std::shared_ptr<guideWireOpenGLRender> Ptr;
		static Ptr New ()
		{
			return Ptr(new guideWireOpenGLRender());
		}

		void SetRadius(double radius);
		void SetBezPtsNumber(size_t bezPtsNumber);
		void SetBezCtrlPtsPercent(double percent);
		void SetGuideWireMassPoints(guideWire::Ptr ptr);
		void Draw();
		void RenderCylinder(GLUquadricObj * qObj, const GK::Point3 & startCenterPoint, const GK::Point3 & endCenterPoint, double radius, int slices, int stacks);
		void RenderSphere(GLUquadricObj * qObj, const GK::Point3 & sphereCenter, double radius, int slices, int stacks);


		void Update();
		~guideWireOpenGLRender();

		void binomialCoeffs (int n, std::vector<int> & coeffs);
		void computeBezPt(int nBeztPts, std::vector<dtkT3<double>> & bezPts, int nCtrlPts,  const std::vector<dtkT3<double>>& ctrlPts);
		void computeCtrlPt(double percent, const std::vector<dtkT3<double>> points, std::vector<std::vector<dtkT3<double>> > &ctrlPts);

	private:
		guideWireOpenGLRender();
		dtkPoints::Ptr mGuideWirePoints;
		guideWire::Ptr mGuideWireMassPoints;
		std::vector< dtkT3<double> > mPoints;
		double mGuideWireRadius;
		double mBezCtrlPtsPercent;
		size_t mBezPtsNumber;

	};
}

#endif