#include "dtkPhysParticle.h"

namespace dtk
{
	dtkPhysParticle::dtkPhysParticle( const GK::Point3& position, const double& lifetime, const double& mass, const dtkT3<double>& vel )
	{
		mPoint = position;
		mMass = mass;
		mVel = vel;
		mLifetime = lifetime;

		mActive = true;

		mResistCoef = 0.0;
	}

	dtkPhysParticle::~dtkPhysParticle()
	{

	}

	bool dtkPhysParticle::Update( double timeslice )
	{
		if(!mActive)
		{
			mForceAccum = dtkT3<double>(0,0,0);
			return true;
		}

		// compute acceleration
		mAccel = ( mForceAccum - mVel * mResistCoef ) / mMass;
		mForceAccum = dtkT3<double>( 0, 0, 0 );

		// get point
		dtkT3<double> p;
		p = GetPosition();

		// Euler step
		p = p + mVel * timeslice;		
		mVel = mVel + mAccel * timeslice;

		// set point
		mPoint = GK::Point3(p.x, p.y, p.z);

		mLifetime -= timeslice;
		if( mLifetime < 0 )
			SetActive(false);

		return true;
	}
}
