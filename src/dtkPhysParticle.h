#ifndef DTK_PHYSPARTICLE_H
#define DTK_PHYSPARTICLE_H

#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"

namespace dtk
{
	class dtkPhysParticle
	{//粒子
	public:
		dtkPhysParticle( const GK::Point3& position, const double& lifetime, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0) );

	public:
		virtual ~dtkPhysParticle();
		
		//更新粒子位置，力,加速度，速度等
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
		GK::Point3 mPoint; //点

		dtkT3<double> mVel; //速度
		dtkT3<double> mAccel; //加速度
		dtkT3<double> mForceAccum; //合力

		double mMass; //质量

		double mResistCoef; //阻力系数

		double mLifetime; //生命周期

		bool mActive; //是否活跃
	};
}

#endif
