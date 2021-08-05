#include "dtkPhysMassPoint.h"
#include "dtkTx.h"

namespace dtk
{
	
	dtkPhysMassPoint::dtkPhysMassPoint(dtkID id, dtkPoints::Ptr pts, const double& mass, const dtkT3<double>& vel, double pointDamp, double pointResistence, dtkDouble3 gravityAccel )
	{
		mID = id;
		mPts = pts;
		mMass = mass;
		mVel = vel;

		mPosBuffers.resize(3);
		mVelBuffers.resize(3);
		mAccelBuffers.resize(3);

		mGravity = gravityAccel * mMass;

		mActive = true;
        mForceAccum = dtkT3<double>(0,0,0);
        mResistance = dtkT3<double>(0,0,0);

        mResistCoef = pointResistence;
        GK::Point3 p = pts->GetPoint( id );
        mPosLastFrame = dtkT3<double>( p[0], p[1], p[2] );

        mImpulse = dtkT3<double>(0,0,0);
        mImpulseNum = 0;
		mPointDamp = pointDamp;

		
	}

	
	dtkPhysMassPoint::~dtkPhysMassPoint()
	{

	}

	
	bool dtkPhysMassPoint::Update(double timeslice, ItrMethod method, dtkID iteration)
	{
		// mActive represent the mass point's accel is zero, points don't leave in this iteration.
		dtkT3<double> p;
		if(!mActive) //无速度， 提前返回，实现固定点
		{
		    switch(method)
		    {
		        case Euler:
                {
                    mAccel =  dtkT3<double>(0, 0, 0);
			        mForceAccum = dtkT3<double>(0,0,0);
                    mVel = (GetPosition() - mPosLastFrame) / timeslice;
                    mPosLastFrame = GetPosition();
			        // mPts->SetPoint(mID, GetPosition()); // meaningless but complete
                    //
                    mImpulse = dtkT3<double>(0,0,0);
                    mImpulseNum = 0;
                    return true;
                }
		        case Mid:
		        case Heun:
                case Collision:
                {
                    if(iteration == 0)
                    {
                        mVel = (GetPosition() - mPosLastFrame) / timeslice;
                        mAccel =  dtkT3<double>(0, 0, 0);
			            mForceAccum = dtkT3<double>(0,0,0);
			            mPosBuffers[iteration] = GetPosition();
			            mVelBuffers[iteration] = mVel;//dtkT3<T>(0, 0, 0);
                        mImpulse = dtkT3<double>(0,0,0);
                        mImpulseNum = 0;
                    }
                    else
                    {
                        mVel = (GetPosition() - mPosLastFrame) / timeslice;
                        mPosBuffers[0] = GetPosition();
                        mPosLastFrame = GetPosition();
				        mAccelBuffers[iteration-1] = dtkT3<double>(0, 0, 0);
				        mForceAccum = dtkT3<double>(0,0,0);
			            // mPts->SetPoint(mID, GetPosition()); // meaningless but complete
                        mImpulse = dtkT3<double>(0,0,0);
                        mImpulseNum = 0;
                    }
                    return true;
                }
                case RK4:
                {
                    if(iteration < 3)
                    {
                        mVel = (GetPosition() - mPosLastFrame) / timeslice;
                        mAccel =  dtkT3<double>(0, 0, 0);
			            mForceAccum = dtkT3<double>(0,0,0);
			            mPosBuffers[iteration] = GetPosition();
			            mVelBuffers[iteration] = mVel;//dtkT3<T>(0, 0, 0);
                        mImpulse = dtkT3<double>(0,0,0);
                        mImpulseNum = 0;
                    }
                    else
                    {
                        mVel = (GetPosition() - mPosLastFrame) / timeslice;
				        mAccelBuffers[iteration-1] = dtkT3<double>(0, 0, 0);
				        mForceAccum = dtkT3<double>(0,0,0);
                        mImpulse = dtkT3<double>(0,0,0);
                        mImpulseNum = 0;
			            // mPts->SetPoint(mID, GetPosition()); // meaningless but complete
                    }
                    return true;
                }
            }
		}

		switch(method)
		{
		case Euler:
			// compute acceleration
			mAccel = (mForceAccum + mForceDecorator + mGravity) / mMass;
			mForceAccum = dtkT3<double>(0,0,0);
			
			// get point
			p = GetPosition();
            mPosLastFrame = p;

			// Euler step
			p = p + mVel * timeslice;		
			mVel = mVel + mAccel * timeslice;

			// set point
			mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));

			break;
		case Mid:
			if(iteration == 0)
			{
				// compute acceleration
				mAccel = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mPosBuffers[iteration] = GetPosition() + mVel * timeslice * 0.5;
				mVelBuffers[iteration] = mVel + mAccel * timeslice * 0.5;
			}
			else
			{
				mAccelBuffers[iteration-1] = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

                mPosLastFrame = GetPosition();
				p = GetPosition() + mVelBuffers[iteration-1] * timeslice;
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVel + mAccelBuffers[iteration-1] * timeslice;
			}
			break;
		case Heun:
			if(iteration == 0)
			{
				// compute acceleration
                mResistance = mVel * (-mResistCoef);
				mAccel = (mForceAccum + mForceDecorator + mGravity + mResistance) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mPosBuffers[iteration] = GetPosition() + mVel * timeslice * 1.0;
				mVelBuffers[iteration] = mVel + mAccel * timeslice * 1.0;
			}
			else
			{
                mResistance = mVelBuffers[iteration-1] * (-mResistCoef); 
				mAccelBuffers[iteration-1] = (mForceAccum + mForceDecorator + mGravity + mResistance) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				p = GetPosition() + (mVel + mVelBuffers[iteration-1]) * 0.5 * timeslice;
                mPosLastFrame = GetPosition();
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVel + (mAccel + mAccelBuffers[iteration-1]) * 0.5 * timeslice;
                //mVelBuffers[iteration] = mVel;
			}
			break;
		case Collision:
			if(iteration == 0)
			{
				mPosBuffers[iteration] = GetPosition();
				mVelBuffers[iteration] = mVel;

				// compute acceleration
                mResistance = mVelBuffers[iteration] * (-mResistCoef);
				mAccelBuffers[iteration] = (mForceAccum + mForceDecorator + mGravity + mResistance) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				p = mPosBuffers[iteration] + mVelBuffers[iteration] * timeslice * 1.0;
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVelBuffers[iteration] + mAccelBuffers[iteration] * timeslice * 1.0;
                mVel = (mVel + mVelBuffers[iteration]) * 0.5;
				mVel = mVel * mPointDamp;
			}
			else
			{
                mResistance = mVel * (-mResistCoef); 
				mAccel = (mForceAccum + mForceDecorator + mGravity + mResistance) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				p = mPosBuffers[iteration-1] + mVel * timeslice;
                mPosLastFrame = mPosBuffers[iteration-1];
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVelBuffers[iteration-1] + (mAccel + mAccelBuffers[iteration-1]) * 0.5 * timeslice;
				mVel = mVel * mPointDamp;
                //mVelBuffers[iteration] = mVel;
			}
			break;
		case RK4:
			if(iteration == 0)
			{
				// compute acceleration
				mAccel = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mPosBuffers[iteration] = GetPosition() + mVel * timeslice * 0.5;
				mVelBuffers[iteration] = mVel + mAccel * timeslice * 0.5;
			}
			else if(iteration == 1)
			{
				// compute acceleration
				mAccelBuffers[iteration-1] = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mPosBuffers[iteration] = GetPosition() + mVel * timeslice * 0.5;
				mVelBuffers[iteration] = mVel + mAccel * timeslice * 0.5;
			}
			else if(iteration == 2)
			{
				// compute acceleration
				mAccelBuffers[iteration-1] = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mPosBuffers[iteration] = GetPosition() + mVel * timeslice * 0.5;
				mVelBuffers[iteration] = mVel + mAccel * timeslice * 0.5;
			}
			else
			{
				mAccelBuffers[iteration-1] = (mForceAccum + mForceDecorator + mGravity) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

                mPosLastFrame = GetPosition(); 
				p = GetPosition() + (mVel + mVelBuffers[0] * 2.0 + mVelBuffers[1] * 2.0 + mVelBuffers[2]) / 6.0 * timeslice;
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVel + (mAccel + mAccelBuffers[0] * 2.0 + mAccelBuffers[1] * 2.0 + mAccelBuffers[2]) / 6.0 * timeslice;
			}
			break;
			
		case Verlet:
			if(iteration == 0)
			{
				mPosBuffers[iteration] = GetPosition();
				mVelBuffers[iteration] = mVel;
				mAccelBuffers[iteration] = mAccel;
				p = mPosBuffers[iteration] + mVelBuffers[iteration] * timeslice + mAccelBuffers[iteration] * (timeslice * timeslice * 0.5);
				mPts->SetPoint(mID, GK::Point3(p.x, p.y, p.z));
				mVel = mVelBuffers[iteration] + mAccelBuffers[iteration] * timeslice * 0.5;
				mVel = mVel * mPointDamp;
			}
			else
			{
				mResistance = mVel * (-mResistCoef); 
				mAccel = (mForceAccum + mForceDecorator  + mResistance) / mMass;
				mForceAccum = dtkT3<double>(0,0,0);

				mVel = mVel + mAccel * (timeslice * 0.5);
				mVel = mVel * mPointDamp;
			}
			break;

		}
		return true;
	}

	
	void dtkPhysMassPoint::SetPosition(dtkT3<double> newPos, dtkID iteration)
    {
        mPosBuffers[iteration] = newPos;
    }

	
	dtkT3<double> dtkPhysMassPoint::GetPosition(ItrMethod method, dtkID iteration)
	{
		switch(method)
		{
            case Euler:
            {
                const GK::Point3& vec = mPts->GetPoint(mID);
			    return dtkT3<double>(vec.x(), vec.y(), vec.z());
            }
		    case Mid:
		    case Heun:
		    case RK4:
            {
			    if(iteration == 0)
			    {
				    const GK::Point3& vec = mPts->GetPoint(mID);
				    return dtkT3<double>(vec.x(), vec.y(), vec.z());
			    }
			    else
			    {
				    return mPosBuffers[iteration-1];
			    }
		    }
		    case Collision:
            {
				const GK::Point3& vec = mPts->GetPoint(mID);
				return dtkT3<double>(vec.x(), vec.y(), vec.z());
            }
        }
		return dtkT3<double>(0.0,0.0,0.0);
	}

    
    void dtkPhysMassPoint::SetVel(dtkT3<double> newVel, dtkID iteration)
    {
        mVelBuffers[iteration] = newVel;
    }

	
	dtkT3<double> dtkPhysMassPoint::GetVel(ItrMethod method, dtkID iteration)
	{
		switch(method)
		{
		case Euler:
			return mVel;
		case Mid:
		case Heun:
		case RK4:
			if(iteration == 0)
			{
				return mVel;
			}
			else
			{
				return mVelBuffers[iteration-1];
			}
		case Collision:
			return mVel;
		}
		return dtkT3<double>(0.0,0.0,0.0);
	}

    
	dtkT3<double> dtkPhysMassPoint::GetAccel(ItrMethod method, dtkID iteration)
	{
		switch(method)
		{
		case Euler:
			return mAccel;
		case Mid:
		case Heun:
		case RK4:
			if(iteration == 0)
			{
				return mAccel;
			}
			else
			{
				return mAccelBuffers[iteration-1];
			}
		case Collision:
			return mAccel;
		}
		return dtkT3<double>(0.0,0.0,0.0);
	}

	
	void dtkPhysMassPoint::SetActive(bool newActive, bool passToTwin) 
	{
		mActive = newActive;
		if(!mActive)
		{
			mVel = dtkT3<double>(0, 0, 0);
			mAccel = dtkT3<double>(0, 0, 0);

			for(unsigned int i = 0; i < 3; i++)
			{
				mPosBuffers[i] = GetPosition();
				mVelBuffers[i] = dtkT3<double>(0, 0, 0);
				mAccelBuffers[i] = dtkT3<double>(0, 0, 0);
			}
		}
		if( mTwins.size() > 0 && passToTwin )
		{
			for( dtkID i = 0; i < mTwins.size(); i++ )
			{
				mTwins[i]->SetActive( newActive, false );
			}
		}
	}
		
	
    void dtkPhysMassPoint::ApplyImpulse()
    {
        if(mImpulseNum != 0)
        {
            mImpulseNum = 1;
            if( mActive )
                mVel = mVel + mImpulse * (1.0 / (mMass * (double)mImpulseNum));
            mImpulse = dtkT3<double>(0,0,0);
            mImpulseNum = 0;
        }
    }

	
	dtkT3<double> dtkPhysMassPoint::GetAndClearImpulse()
	{
		mImpulseNum = 1;
		dtkT3<double> temp = mImpulse * (1.0 / (double)mImpulseNum);
		mImpulse = dtkT3<double>( 0, 0, 0 );
		mImpulseNum = 0;
		return temp;
	}
}
