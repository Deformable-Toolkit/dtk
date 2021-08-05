#include "dtkPhysParticleSystem.h"

#ifdef DTK_DEBUG
    #define DTK_PHYSPARTICLESYSTEM_DEBUG
#endif //DTK_DEBUG

#ifdef DTK_PHYSPARTICLESYSTEM_DEBUG
    #include <iostream>
    using namespace std;
#endif

#include <string>
using namespace std;
namespace dtk
{
	dtkPhysParticleSystem::dtkPhysParticleSystem( double particleRadius, double particleMass, double particleLifetime )
	{
		mDefaultMass = particleMass;
		mDefaultLifetime = particleLifetime;
		mForceAffector = dtkDouble3( 0, 0, 0 );

		mEmitting = false;
		mDefaultParticleRadius = particleRadius;
	}

	dtkPhysParticleSystem::~dtkPhysParticleSystem()
	{

	}

	const GK::Point3& dtkPhysParticleSystem::GetPoint(dtkID id) const
	{
		return mParticles[id]->GetPoint();
	}

    bool dtkPhysParticleSystem::Update(double timeslice)
    {
		if( mEmitting )
		{//添加发散粒子
			double numOfNewParticles = timeslice * mEmitRate;
#ifdef DTK_PHYSPARTICLESYSTEM_DEBUG
			cout << "Emitting " << numOfNewParticles << endl;
#endif		
			while( numOfNewParticles > 0 )
			{
				if( numOfNewParticles < 1.0 )
				{
					if( rand() < numOfNewParticles * (double)RAND_MAX )
					{
						AddParticle( mEmitPosition, mDefaultLifetime, mDefaultMass, mDefaultVelocity );
#ifdef DTK_PHYSPARTICLESYSTEM_DEBUG
						cout << "Emit 1!" << endl;
#endif		
					}
					numOfNewParticles = -1.0;
				}
				else{
					numOfNewParticles -= 1.0;
#ifdef DTK_PHYSPARTICLESYSTEM_DEBUG
					cout << "Emit 2!" << endl;
#endif		
					AddParticle( mEmitPosition, mDefaultLifetime, mDefaultMass, mDefaultVelocity );
				}
			}
		}

		for( dtkID i = 0; i < mParticles.size(); i++ )
		{//施加力
			mParticles[i]->AddForce( mForceDisturbance * dtkDouble3(
				(double)rand() / (double) RAND_MAX * 2.0 - 1.0, 
				(double)rand() / (double) RAND_MAX * 2.0 - 1.0,
				(double)rand() / (double) RAND_MAX * 2.0 - 1.0 ) );
			mParticles[i]->AddForce( mForceAffector );
			mParticles[i]->Update( timeslice );
		}
		
		dtkID emptyPosition = 0;
		for( dtkID i = 0; i < mParticles.size(); i++ )
		{//去除不活跃点
			if( mParticles[i]->IsActive() )
			{
				mParticles[emptyPosition++] = mParticles[i];
			}
			else
			{
				delete mParticles[i];
			}
		}
		mParticles.erase( mParticles.begin() + emptyPosition, mParticles.end() );

		return true;
	}

	void dtkPhysParticleSystem::StartEmit()
	{
		mEmitting = true;
	}

	void dtkPhysParticleSystem::StopEmit()
	{
		mEmitting = false;
	}

	dtkID dtkPhysParticleSystem::AddParticle( const GK::Point3& position, const double& lifetime, const double& mass, const dtkT3<double>& vel )
	{
		mParticles.push_back(new dtkPhysParticle( position, lifetime, mass, vel ));

		return mParticles.size() - 1;
	}

	void dtkPhysParticleSystem::AddForceAffector( dtkDouble3 force )
	{
		mForceAffector = mForceAffector + force;
	}
}
