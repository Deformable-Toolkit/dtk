#ifndef DTK_PHYSMASSSPRINGTHREADIMPL_H
#define DTK_PHYSMASSSPRINGTHREADIMPL_H

#include "dtkPhysMassSpringThread.h"

//#ifdef DTK_DEBUG
    #define DTK_PHYSMASSSPRINGTHREADIMPL_DEBUG
//#endif //DTK_DEBUG

#ifdef DTK_PHYSMASSSPRINGTHREADIMPL_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{
	
	dtkPhysMassSpringThread::dtkPhysMassSpringThread(double interval, int length, dtkT3<double> firstPos, Orientation ori, double mass, double edgeStiff, double bendStiff,
		double torsionStiff, double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp)
		:dtkPhysTetraMassSpring( true, mass, edgeStiff, edgeDamp, 0.99, 0 )
	{
		mTetraMeshPtr = dtkStaticTetraMesh::New();

		mFirstPos = firstPos;
		mInterval = interval;//2.0;//5.0; // mm
		mRotateInterval = 0.4;
		mLength = length;
		if(length < 3)
			mLength = 3;

		mEdgeStiff = edgeStiff;//8000;
		mExtraEdgeStiff = edgeStiff;//8000;
		mBendStiff = bendStiff;//4000;
		mTorsionStiff = torsionStiff;//4000;

		mEdgeDamp = edgeDamp;//20;
		mExtraEdgeDamp = extraEdgeDamp;//20;
		mBendDamp = bendDamp;//20;
		mTorsionDamp = torsionDamp;//20;

		mOrientation = ori;

		constructThreadMesh();
		constructTetraMesh();
		addExtraEdgeSpring();
		addEdgeSpring();
		addBendSpring();
		addTorsionSpring();
	}

	
	dtkPhysMassSpringThread::~dtkPhysMassSpringThread()
	{

	}

	
	void dtkPhysMassSpringThread::constructThreadMesh()
	{
		dtkPoints::Ptr mPointsPtr = dtkPointsVector::New();
		// compute the height of triangle
		double tempRadius = tan( 60.0 / 180.0 * dtkPI ) * mInterval / 2.0;
		dtkT3<double> oriX;
		dtkT3<double> oriY;
		dtkT3<double> oriZ;
		switch(mOrientation)
		{
		case RIGHT:
			oriX = dtkT3<double>(1,0,0);
			oriY = dtkT3<double>(0,1,0);
			oriZ = dtkT3<double>(0,0,1);
			break;
		case LEFT:
			oriX = dtkT3<double>(-1,0,0);
			oriY = dtkT3<double>(0,-1,0);
			oriZ = dtkT3<double>(0,0,-1);
			break;
		case UP:
			oriX = dtkT3<double>(0,1,0);
			oriY = dtkT3<double>(0,0,1);
			oriZ = dtkT3<double>(1,0,0);
			break;
		case DOWN:
			oriX = dtkT3<double>(0,-1,0);
			oriY = dtkT3<double>(0,0,-1);
			oriZ = dtkT3<double>(-1,0,0);
			break;
		case FRONT:
			oriX = dtkT3<double>(0,0,1);
			oriY = dtkT3<double>(1,0,0);
			oriZ = dtkT3<double>(0,1,0);
			break;
		case BACK:
			oriX = dtkT3<double>(0,0,-1);
			oriY = dtkT3<double>(-1,0,0);
			oriZ = dtkT3<double>(0,-1,0);
			break;
		default:
			assert(false);
			oriX = dtkT3<double>(0,0,1);
			oriY = dtkT3<double>(1,0,0);
			oriZ = dtkT3<double>(0,1,0);
			break;
		}

		// add the rotateInterval, present the the thread as x-coordinate.
		// rotate angle around of x-coordinate.
		for(dtkID i = 0;i < mLength * 2 + 1;i++)
		{
			// set the position of the mass ,that is not extra mid point. 
			if(i % 2 == 0)
			{
				dtkT3<double> temp = mFirstPos + oriX * ( mInterval * (double)i / 2.0); 
				mPointsPtr->SetPoint(i,GK::Point3(temp.x, temp.y, temp.z));
			}
			else
			{
				double angle = mRotateInterval * i;
				if(angle > 2.0 * dtkPI)
					angle = angle - 2.0 * dtkPI;
				dtkT3<double> tempPoint;
				tempPoint = mFirstPos +
					oriY * tempRadius * cos(angle) + 
					oriZ * tempRadius * sin(angle) + 
					oriX * ( mInterval * (float)((i-1) / 2) + mInterval / 2.0 ); 

				mPointsPtr->SetPoint(i,GK::Point3(tempPoint.x,tempPoint.y,tempPoint.z));
			} 
		}

		this->SetPoints(mPointsPtr);
		mTetraMeshPtr->SetPoints(mPointsPtr);
	}

	
	void dtkPhysMassSpringThread::constructTetraMesh()
	{
		for(dtkID i = 0;i < mLength * 2 - 1;i += 2)
		{
			if(i == 0)
			{
				mTetraMeshPtr->InsertTetra(i,i+1,i+2,i+3);
				//           mTetraMeshPtr->InsertTetra(i,i+1,i+2,i+4);
			}
			else if(i != 0 && i != mLength*2 - 2)
			{
				mTetraMeshPtr->InsertTetra(i-1,i,i+1,i+2);
				mTetraMeshPtr->InsertTetra(i,i+1,i+2,i+3);
				//            if(mTetraMeshPtr->InsertTetra(i-2,i,i+1,i+2) == dtkErrorID)
				//               exit(1);
				//           if(mTetraMeshPtr->InsertTetra(i,i+1,i+2,i+4) == dtkErrorID)
				//               exit(1);
			}
			else if(i == mLength*2 - 2)
			{
				mTetraMeshPtr->InsertTetra(i-1,i,i+1,i+2);
				//         if(mTetraMeshPtr->InsertTetra(i-2,i,i+1,i+2) == dtkErrorID)
				//             exit(1);
			}
		}

		this->SetTetraMesh(mTetraMeshPtr);
	}

	
	void dtkPhysMassSpringThread::addExtraEdgeSpring()
	{
		dtkPhysSpring* tempSpring;
		for(dtkID i = 0;i < mLength * 2 + 1 - 1;i++)
		{
			tempSpring = this->GetSpringByPoints(dtkID2(i,i+1));
			if( tempSpring != 0 )
			{
				tempSpring->SetStiffness(mExtraEdgeStiff);
				tempSpring->SetDamp(mExtraEdgeDamp);
			}
		}
	}

	
	void dtkPhysMassSpringThread::addEdgeSpring()
	{
		dtkPhysSpring* tempSpring;
		for(dtkID i = 0;i < mLength * 2 - 1;i += 2)
		{
			tempSpring = this->GetSpringByPoints(dtkID2(i,i+2));
			if( tempSpring != 0 )
			{
				tempSpring->SetStiffness(mEdgeStiff);
				tempSpring->SetDamp(mEdgeDamp);
			}
		}
	}

	
	void dtkPhysMassSpringThread::addBendSpring()
	{
		dtkPhysSpring* tempSpring;
		for( dtkID i = 0; i < mLength * 2 + 1 - 4; i += 2 )
		{
			this->AddSpring(i,i+4,mBendStiff,mBendDamp);
		}
		for( dtkID i = 1; i < mLength * 2 + 1 - 3; i += 2 )
		{
			tempSpring = this->GetSpringByPoints(dtkID2(i,i+2));
			if( tempSpring != 0 )
			{
				tempSpring->SetStiffness(mBendStiff);
				tempSpring->SetDamp(mBendDamp);
			}
		}
	}

	
	void dtkPhysMassSpringThread::addTorsionSpring()
	{
		dtkPhysSpring* tempSpring;
		for(dtkID i = 0;i < mLength * 2 + 1 - 3;i++)
		{
			tempSpring = this->GetSpringByPoints(dtkID2(i,i+3));
			if( tempSpring != 0 )
			{
				tempSpring->SetStiffness(mTorsionStiff);
				tempSpring->SetDamp(mTorsionDamp);
			} 
		}
	}


	// the total of impulse doesn't increase, just divide them into some mass point. 
	void dtkPhysMassSpringThread::ImpulsePropagate(dtkT3<double> impulse, dtkID pointID, dtkID opt, size_t range)
	{
		dtkDouble3 tempPos1, tempPos2, e, tempImpulse1, tempImpulse2, tempImpulseBuffer1, tempImpulseBuffer2;
		tempImpulseBuffer1 = tempImpulseBuffer2 = tempImpulse1 = tempImpulse2 = impulse;
		for( dtkID i = 0; i < range; i++ )
		{
			if( opt != 1 && (pointID + (i + 1) * 2) <= mLength * 2 && (i + 1) < range)
			{
				tempPos1 = this->GetMassPoint(pointID + i * 2)->GetPosition();
				tempPos2 = this->GetMassPoint(pointID + (i + 1) * 2)->GetPosition();
				e = normalize(tempPos1 - tempPos2);
				tempImpulse1 = e * dot(tempImpulse1,e);
				this->GetMassPoint(pointID + (i + 1) * 2)->AddImpulse(tempImpulse1);
				this->GetMassPoint(pointID + i * 2 + 1)->AddImpulse((tempImpulse1 + tempImpulseBuffer1) * 0.5);
				tempImpulseBuffer1 = tempImpulse1;
			}
			if( opt != 2 && pointID >= (i + 1) * 2 && (i + 1) < range)
			{
				tempPos1 = this->GetMassPoint(pointID - i * 2)->GetPosition();
				tempPos2 = this->GetMassPoint(pointID - (i + 1) * 2)->GetPosition();
				e = normalize(tempPos1 - tempPos2);
				tempImpulse2 = e * dot(tempImpulse2,e);
				this->GetMassPoint(pointID - (i + 1) * 2)->AddImpulse(tempImpulse2);
				this->GetMassPoint(pointID - i * 2 - 1)->AddImpulse((tempImpulse2 + tempImpulseBuffer2) * 0.5);
				tempImpulseBuffer2 = tempImpulse2;
			}
		}
	}

	
	void dtkPhysMassSpringThread::ControlEndPropagate(double timeslice, int range)
	{
		for(dtkID j = 0;j < this->GetNumberOfMassPoints();j = j + 2)
		{
			if( !this->GetMassPoint(j)->IsActive() )
			{
				dtkDouble3 lastPos = this->GetMassPoint(j)->GetLastFramePosition();
				dtkDouble3 curPos = this->GetMassPoint(j)->GetPosition();

				dtkDouble3 vel = (curPos - lastPos) * (1.0 / timeslice);
				dtkDouble3 impulse = vel * this->GetMassPoint(j)->GetMass();
                ImpulsePropagate( impulse, j, 0, range );
			}
		}
	}

	
	bool dtkPhysMassSpringThread::ApplyImpulse( double timeslice )
	{
		//ControlEndPropagate( timeslice );
		return dtkPhysMassSpring::ApplyImpulse( timeslice );
	}

}

#endif

