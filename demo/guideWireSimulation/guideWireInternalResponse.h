#ifndef GUIDEWIREINTERNALRESPONSE_H
#define  GUIDEWIREINTERNALRESPONSE_H

#include "dtkMatrixOP.h"
#include "dtkPhysMassPoint.h"
#include <windows.h>
using namespace std;

namespace dtk
{
#define pi 3.1415
	class guideWireInternalResponse : public boost::noncopyable
	{
	public :
		typedef std::shared_ptr<guideWireInternalResponse> Ptr;
		static Ptr New(double radius = 0.001, double lineDensity = 7860, double yougModulus = 200, double shearModulus = 100, double stretchModulus = 100, double springConst = 300, double segInterval = 0.1, double length = 0.1,  double defaultDamp = 5880, double defaultPointDamp = 0.8, double defaultPointResistence = 0.0, dtkT3<double> defaultGravityAccel = dtkT3<double>( 0.0,0.0,0.0 ), double defaultCurvature = 0.0)	
		{
			return Ptr (new guideWireInternalResponse(radius, lineDensity, yougModulus, shearModulus, stretchModulus, springConst, segInterval, length, defaultDamp, defaultPointDamp, defaultPointResistence, defaultGravityAccel, defaultCurvature));
		}

		bool Update(double timeslice, ItrMethod method = Euler, bool limitDeformation = false);
		bool UpdateMassPoints(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);

		static dtkT3<double> vector3TodtkT3(const GK::Vector3& v3)
		{
			return dtkT3<double>(v3.x(), v3.y(), v3.z());
		}

		void ConstructGuideWire(dtkPoints::Ptr points, const std::vector<double> & segInterval); 
		void SetPoints(dtkPoints::Ptr points);
		dtkPoints::Ptr GetPoints();

		const GK::Point3& GetPoint(dtkID id) const;
		void SetPoint(dtkID id, const GK::Point3 &coord);

		void ResetContactForces()
		{
			dtkID size = mMassPoints.size();
			for (dtkID i = 0; i < size; i++)
				mContactForces[i] = dtkDouble3(0, 0, 0);
		}
		void SetContactForces(dtkID id, const dtkDouble3 & force)
		{
			mContactForces[id] = force;
		}

		dtkPhysMassPoint* GetMassPoint(dtkID id){ return mMassPoints[id]; };	
		dtkID GetNumberOfMassPoints() { return mMassPoints.size(); }
		dtkID AddMassPoint(dtkID id, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0), double pointDamp = 1.0, double pointResistence = 2.5, dtkT3<double> defaultGravityAccel = dtkT3<double>( 0,0,0 ) );
		void SetPointMass(dtkID id, double newMass);
		void ApplyExternalVelImpulse(const dtkT3<double> & velImpulse);
		void ApplyExternalForce(const dtkT3<double> & force);
		void ApplyExternalTwist(double twist);
		bool ApplyImpulse( double timeslice );

		void AddImpulse(dtkID id, const dtkT3<double> & impulse);
		void AddForce(dtkID id, const dtkT3<double> & force);
		void AddPosition(dtkID id, const GK::Vector3 & positionChange);



		// mainfold projection

		void ResistStretch(double timeslice);
		void ResistOverBend(double bendAngle, double timeslice);
		void AddBendForce(double timeslice);
		void AddTwistForce(double timeslice);
		void AddContactForce();
		

		// rotation
		void RotateMassPoint(dtkPoints::Ptr  points, dtkID rotatePointID, dtkID axisPointID1, dtkID axisPointID2, double angle);

		bool mBaseDirectionTage; 


	private:
		guideWireInternalResponse(double radius, double lineDensity , double yougModulus , double shearModulus , double stretchModulus , double springConst , double segInterval , double length ,   double defaultDamp, double defaultPointDamp, double defaultPointResistence, dtkT3<double> defaultGravityAccel, double defaultCurvature );


	protected:
		// the member of other related class.
		dtkPoints::Ptr mPts;
		std::vector<GK::Point3> nextFramePos;
		std::vector<dtkT3<double>  >nextFrameVel;
		std::vector<dtkT3<double> > externForce;
		std::vector<dtkPhysMassPoint*> mMassPoints;


	private:
		// the coefficient of guideWireInternalResponse.
		double mRadius;
		double mLineDensity;
		double mYoungModulus;
		double mShearModulus;
		double mStretchModulus;
		double mSpringConst;
		vector<double> mSegInterval;
		double mLength;

		double mStiffnessStretch;
		double mStiffnessTensor11;
		double mStiffnessTensor22;
		double mStiffnessTensor33;

		dtkT3<double> mInertiaTensor;

		// the coefficient of mass point.
		double mDefaultMass;
		double mDefaultDamp;
		double mDefaultPointDamp;
		double mDefaultCurvature;
		double mDefaultPointResistence;
		double mRotateDissipationCoef;
		dtkT3<double> mDefaultGravityAccel;
		double mTwist;
		double mTwistInterval;

		// contact force
		vector<dtkDouble3> mContactForces;
	};
}
#endif