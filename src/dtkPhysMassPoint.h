#ifndef dtkPHYSMASSPOINT_H
#define dtkPHYSMASSPOINT_H

#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"
#include "dtkPoints.h"


namespace dtk
{
	enum ItrMethod {Euler, Mid, RK4, Heun, Collision, Verlet};

	class dtkPhysMassPoint
	{
	public:
		dtkPhysMassPoint(dtkID id, dtkPoints::Ptr pts, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0), double pointDamp = 1.0, double pointResistence = 2.5, dtkDouble3 gravityAccel = dtkT3<double>( 0,0,0 ) );

	public:
		virtual ~dtkPhysMassPoint();

		bool Update(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);
		void ApplyImpulse();
		dtkT3<double> GetAndClearImpulse();

		const GK::Point3& GetPoint(){ return mPts->GetPoint(mID); }
		dtkPoints::Ptr GetPoints() { return mPts; }
        dtkID GetPointID() { return mID; }
		void SetPointID(dtkID id) {mID = id;}
		dtkT3<double> GetPosition(ItrMethod method = Euler, dtkID iteration = 0);
        dtkT3<double> GetLastFramePosition() { return mPosLastFrame; }
		void SetPosition(dtkT3<double> newPos, bool passToTwin = true ) 
		{
			mPts->SetPoint(this->mID,GK::Point3(newPos[0],newPos[1],newPos[2]));
			if( passToTwin && mTwins.size() > 0)
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetPosition( newPos, false );
				}
			}
		}
        void SetPosition(dtkT3<double> newPos, dtkID iteration);
		dtkT3<double> GetVel(ItrMethod method = Euler, dtkID iteration = 0);
		dtkT3<double> GetAccel(ItrMethod method = Euler, dtkID iteration = 0);

		// this cannot be used during the update
		void SetVel(dtkT3<double> newVel, bool passToTwin = true ) { 
			mVel = newVel;
			/*if(  passToTwin && mTwins.size() > 0 )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetVel( newVel, false );
				}
			}*/
		}

        void SetVel(dtkT3<double> newVel, dtkID iteration);

		// this cannot be used during the update
		void SetPoint( GK::Point3& newPos, bool passToTwin = true ) {
			mPts->SetPoint( this->mID, newPos );
			/*if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetPoint( newPos, false );
				}
			}*/
		}

		void SetMass(const double& mass){ mMass = mass; }
		const double& GetMass(){ return mMass; }

		void SetResistCoef(const double& resistCoef){ mResistCoef = resistCoef; }
		const double& GetResistCoef(){ return mResistCoef; }

		void SetForceDecorator(const dtkT3<double>& fd){ mForceDecorator = fd; }
		void AddForceDecorator(const dtkT3<double>& newFD) { mForceDecorator = mForceDecorator + newFD; }
		const dtkT3<double>& GetForceDecorator(){ return mForceDecorator; }

        void SetImpulse( const dtkT3<double>& impulse, bool passToTwin = true ) { 
            mImpulse = impulse;
			if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetImpulse( impulse, false );
				}
			}
        }

        void AddImpulse( const dtkT3<double>& newImpulse, bool passToTwin = true ) { 
            mImpulse = mImpulse + newImpulse; 
            mImpulseNum++;
            if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->AddImpulse( newImpulse, false );
				}
			}
        }
        const dtkT3<double>& GetImpulse() { return mImpulse; }

        void AddForce(const dtkT3<double>& f, bool passToTwin = true){ 
			mForceAccum = mForceAccum + f / (double)( mTwins.size() + 1 ); 
			if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->AddForce( f, false );
				}
			}
        }

        const dtkT3<double>& GetForceAccum() { return mForceAccum; }
        void SetForceAccum(dtkT3<double> newForceAccum) { mForceAccum = newForceAccum; }

		void SetActive( bool newActive, bool passToTwin = true );
        bool IsActive() { return mActive; }

		void SetCollide(bool newCollide) { mCollide = newCollide; }
		bool GetCollide() { return mCollide; }

		void SetPointDamp(double newPointDamp) { mPointDamp = newPointDamp; }
		double GetPointDamp() { return mPointDamp; }

		void SetGravity( dtkT3<double> gravity ) { mGravity = gravity; }
        dtkT3<double> GetGravity() { return mGravity; }
		
		// the list of vector represent the mass point state in each iteration. 
        std::vector< dtkT3<double> > GetPosBuffer() { return mPosBuffers; }
        std::vector< dtkT3<double> > GetVelBuffer() { return mVelBuffers; }
        std::vector< dtkT3<double> > GetAccelBuffer() { return mAccelBuffers; }

		std::vector< dtkT3<double> > mPosBuffers;
        dtkT3<double> mImpulse;

        void AddTwin( dtkPhysMassPoint* newTwin ) 
		{ 
			mTwins.push_back( newTwin ); 
		}

		void AbandonTwins()
		{
			mTwins.clear();
		}

  //      dtkPhysMassPoint* GetTwin() 
		//{ 
		//	return mTwin; 
		//}

        bool HasTwin() 
		{ 
            if( mTwins.size() != 0 )
                return true;
            else
                return false;
        }

		dtkID GetLabel() { return mLabel; }

		void SetLabel( dtkID label ) 
		{ 
			// avoid using 0 as label
			assert( label != 0 );
			mLabel = label; 
		}

		void ResetDynamicState()
		{
			mVel = 0;
			mAccel = 0;
			for( dtkID i = 0; i < 3; i++ )
			{
				mVelBuffers[i] = 0;
				mAccelBuffers[i] = 0;
			}
		}

	private:
		
		dtkID mID;
		// the position at the last iteration.
		dtkPoints::Ptr mPts;

		dtkT3<double> mVel;
		dtkT3<double> mAccel;
		dtkT3<double> mForceAccum;
		dtkT3<double> mGravity;
        dtkT3<double> mResistance;

        dtkT3<double> mPosLastFrame;
		double mMass;
        double mResistCoef;
		double mPointDamp;

		std::vector< dtkT3<double> > mVelBuffers;
		std::vector< dtkT3<double> > mAccelBuffers;

		dtkT3<double> mForceDecorator;	

        size_t mImpulseNum;

		bool mActive;
		bool mCollide;

		dtkID mLabel;

	public:
		// Test Feature
		std::vector< dtkPhysMassPoint* > mTwins;
	};
}

#endif
