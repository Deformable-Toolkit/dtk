#ifndef DTK_PHYSPARTICLE_H
#define DTK_PHYSPARTICLE_H

#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"

namespace dtk
{
	class dtkPhysParticle
	{
	public:
		dtkPhysParticle( const GK::Point3& position, const double& lifetime, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0) );

	public:
		virtual ~dtkPhysParticle();

		bool Update(double timeslice );

		const GK::Point3& GetPoint(){ return mPoint; }
		void SetPoint( const GK::Point3& point ) { mPoint = point; }

		dtkT3<double> GetPosition() { return dtkT3<double>( mPoint[0], mPoint[1], mPoint[2] ); }
		void SetPosition(dtkT3<double> newPos) { mPoint = GK::Point3( newPos[0], newPos[1], newPos[2] ); }

		dtkT3<double> GetVel() { return mVel; }
		void SetVel(dtkT3<double> newVel) { mVel = newVel;}

		void SetMass(const double& mass){ mMass = mass; }
		const double& GetMass(){ return mMass; }

        void AddForce( const dtkT3<double>& f ){
				mForceAccum = mForceAccum + f; 
        }

        const dtkT3<double>& GetForceAccum() { return mForceAccum; }
        void SetForceAccum(dtkT3<double> newForceAccum) { mForceAccum = newForceAccum; }

		void SetActive( bool newActive ) { mActive = newActive; }
        bool IsActive() { return mActive; }

	private:
		GK::Point3 mPoint;

		dtkT3<double> mVel;
		dtkT3<double> mAccel;
		dtkT3<double> mForceAccum;

		double mMass;

		double mResistCoef;

		double mLifetime;

		bool mActive;
	};
}

#endif
