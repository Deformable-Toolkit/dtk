#include "guideWireInternalResponse.h"
#include "dtkMatrixOP.h"
#include <windows.h>

using namespace std;
using namespace boost;
#define errorFactor 0.000000000001
namespace dtk
{
	guideWireInternalResponse::guideWireInternalResponse(double radius, double lineDensity , double yougModulus , double shearModulus , double stretchModulus , double springConst , double segInterval , double length,  double defaultDamp, double defaultPointDamp, double defaultPointResistence, dtkT3<double> defaultGravityAccel, double defaultCurvature )
	{
		mRadius = radius;
		mLineDensity = lineDensity;
		mYoungModulus = yougModulus * 5000;
		mShearModulus = shearModulus*0;
		mStretchModulus = stretchModulus*10000 / 25 * 0;
		mSpringConst = springConst / 2000 * 0;
		//mSpringConst = 0;
		mLength = length;

		//mDefaultMass = lineDensity * length * pi * pow(mRadius, 2);
		mDefaultMass = 2.0;
		mDefaultDamp = defaultDamp;
		mDefaultPointDamp = 1.0;
		mDefaultPointResistence = 6.0 * mDefaultMass ;  // 1.0  3.0 
		mDefaultGravityAccel = defaultGravityAccel;
		mDefaultCurvature = defaultCurvature;

		mStiffnessStretch =  mStretchModulus * pi * pow(mRadius, 2);
		mStiffnessTensor11 =  mYoungModulus * pi * pow(mRadius, 2) / 4.0;
		mStiffnessTensor22 = mStiffnessTensor11;
		mStiffnessTensor33 = mShearModulus * pi * pow(mRadius, 2) / 2.0;

		mInertiaTensor.x = mLineDensity * pi * pow(mRadius, 2) / 4.0;
		mInertiaTensor.y= mInertiaTensor.x;
		mInertiaTensor.z = mInertiaTensor.x * 2.0;

		mRotateDissipationCoef = 0.0001 ;
		mTwist = 0.0;
		mBaseDirectionTage = true;
		mTwistInterval = 1.0;

	}

	bool guideWireInternalResponse::Update(double timeslice, ItrMethod method, bool limitDeformation)
	{
		switch (method)
		{
		case Euler:
			UpdateMassPoints(timeslice, method, 0);
			break;

		case Collision:
			for (int i = 0; i < 20; i++)
			{
				if (i >= 5)
					ResetContactForces();
				UpdateMassPoints(timeslice / 20, Collision, 0);
				UpdateMassPoints(timeslice / 20, Collision, 1);
			}

			double splitAngle = 20;
			double mergeAngle = 5;
			double maxSegLength = 4;
			double minSegLength = 1;
			/*dynamicSuit::Update(mMassPoints, mSegInterval, splitAngle, mergeAngle, maxSegLength, minSegLength, mDefaultMass, mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel );*/
			ResistStretch(timeslice);

			for (unsigned int i = 0; i < mMassPoints.size() - 1; i++)
			{
				double length = GK::Length(mMassPoints[i]->GetPoint() - mMassPoints[i+1]->GetPoint());
				if (length < 0.001)
					std::cout << "length is less than 0.1" << std::endl;
			}

			break;

		/*case Collision:

			for (int i = 0; i < 50; i++)
			{
				UpdateMassPoints(timeslice / 50, method, 0);
				UpdateMassPoints(timeslice / 50, method, 1);
			}*/
			
			//double splitAngle = 20;
			//double mergeAngle = 5;
			//double maxSegLength = 4;
			//double minSegLength = 1;
			///*dynamicSuit::Update(mMassPoints, mSegInterval, splitAngle, mergeAngle, maxSegLength, minSegLength, mDefaultMass, mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel );*/
			//ResistStretch(timeslice);
			////ResistOverBend(30, timeslice);         // 3 20


			//for (unsigned int i = 0; i < mMassPoints.size() - 1; i++)
			//{
			//	double length = GK::Length(mMassPoints[i]->GetPoint() - mMassPoints[i+1]->GetPoint());
			//	if (length < 0.001)
			//		std::cout << "length is less than 0.1" << std::endl;
			//	
			//}

		/*	break;*/
		}

		return true;
	}
	bool guideWireInternalResponse::UpdateMassPoints(double timeslice, ItrMethod method, dtkID iteration)
	{
		/*for (dtkID i = 0; i <  mMassPoints.size() - 1 ; i++)
		{
			computeInternalForce(i, timeslice, method, iteration);		
		}
		*/
		ApplyImpulse(timeslice);
		//ResistOverBend(30, timeslice); 
		AddBendForce(timeslice);
		AddContactForce();
		
		for (dtkID i = 0; i <= mMassPoints.size() - 1; i++)
		{
			mMassPoints[i]->Update(timeslice, method, iteration);
		}

		return true;	
	}

	void guideWireInternalResponse::ResistStretch(double timeslice)
	{
		GK::Vector3 edge;
		double length;
		double lambad;
		dtkID size = mMassPoints.size();
		vector <GK::Vector3> sumDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));
		GK::Vector3 differPosition;

		dtkT3<double> addVel;
		int count = 0;
		int constraintTimes = 10;

		while (count < constraintTimes)
		{
			count++;

			vector <GK::Vector3> eachDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));
			for (dtkID i = 0; i < size-1; i++)                                                                                                                        
			{
				edge = mPts->GetPoint(i+1) -mPts->GetPoint(i);
				length = GK::Length(edge);
				lambad = 0.5 * (length - mSegInterval[i]) / length;
				if (lambad < 0.0001 && lambad > -0.0001)
					continue;

				differPosition = edge * lambad;
				sumDifferPosition[i] = sumDifferPosition[i] + differPosition;
				sumDifferPosition[i+1] = sumDifferPosition[i+1] - differPosition;
				eachDifferPosition[i] = eachDifferPosition[i] + differPosition;
				eachDifferPosition[i+1] = eachDifferPosition[i+1] - differPosition;
			}

			for (dtkID i = 0; i < size - 1; i++)                                                                                                                        
			{
				mMassPoints[i]->SetPoint(mMassPoints[i]->GetPoint() + eachDifferPosition[i] , false);
				//mMassPoints[i]->AddForce(vector3TodtkT3(eachDifferPosition[i]) / timeslice / timeslice * mMassPoints[i]->GetMass() );
				//mMassPoints[i]->Update(timeslice , Verlet, 0);
				//mMassPoints[i]->Update(timeslice, Collision, 1);
			}


		}

		for (dtkID i = 0; i < size - 1; i++)
		{
			addVel.x = sumDifferPosition[i].x() / timeslice;
			addVel.y = sumDifferPosition[i].y()/  timeslice;
			addVel.z = sumDifferPosition[i].z()/ timeslice;
			mMassPoints[i]->SetVel(mMassPoints[i]->GetVel() + addVel, false);
			//mMassPoints[i]->AddForce(vector3TodtkT3(sumDifferPosition[i]) / timeslice / timeslice * mMassPoints[i]->GetMass());
		}
	}

	void guideWireInternalResponse::ResistOverBend(double bendAngle, double timeslice)
	{
		GK::Vector3 differPosition1;
		GK::Vector3 differPosition2;
		GK::Vector3 differPosition3;

		dtkT3<double> addVel;
		double constranitValue;

		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		double leftEdgeLength;
		double rightEdgeLength;
		double dotProd;
		double cosTheta; 
		dtkID size = mMassPoints.size();
		vector <GK::Vector3> sumDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));

		int count = 0;
		int constraintTimes = 1;
		while (count < constraintTimes)
		{
			count ++;
			vector <GK::Vector3> eachDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));

			for (dtkID id = 1; id < size-1; id++)
			{
				cosTheta = cos(bendAngle / 180 * pi);
				leftEdge = mMassPoints[id]->GetPoint() - mMassPoints[id-1]->GetPoint();
				rightEdge = mMassPoints[id+1]->GetPoint() - mMassPoints[id]->GetPoint();
				leftEdgeLength = GK::Length(leftEdge);
				rightEdgeLength = GK::Length(rightEdge);
				dotProd = GK::DotProduct(leftEdge, rightEdge);

				constranitValue = dotProd  - leftEdgeLength * rightEdgeLength * cosTheta;

				// exceed the protected angle: bendAngle
				if (constranitValue < 0)
				{
					GK::Vector3 twoEdgeDiff = rightEdge - leftEdge;
					double lamda =constranitValue / ( pow(rightEdgeLength, 2) + GK::DotProduct(twoEdgeDiff, twoEdgeDiff) + pow(leftEdgeLength, 2) );
					differPosition1 = lamda* rightEdge;

					differPosition2 = - lamda * twoEdgeDiff ;

					differPosition3 = - lamda * leftEdge;

					eachDifferPosition[id-1] = eachDifferPosition[id-1] + differPosition1;
					eachDifferPosition[id] = eachDifferPosition[id] + differPosition2;
					eachDifferPosition[id+1] = eachDifferPosition[id+1] + differPosition3;

					sumDifferPosition[id-1] = sumDifferPosition[id-1] + differPosition1;
					sumDifferPosition[id] = sumDifferPosition[id] + differPosition2;
					sumDifferPosition[id+1] = sumDifferPosition[id+1] + differPosition3;
				}
			}

			for (dtkID i = 0; i < size; i++)                                                                                                                        
			{
				mMassPoints[i]->SetPoint(mMassPoints[i]->GetPoint() + eachDifferPosition[i] , false);
				//mMassPoints[i]->AddForce(vector3TodtkT3(eachDifferPosition[i] ) *( mMassPoints[i]->GetMass() / 100.0 / timeslice / timeslice ));
			}
		}

		for (dtkID i = 0; i < size; i++)
		{
			addVel.x = sumDifferPosition[i].x() / timeslice;
			addVel.y = sumDifferPosition[i].y()/  timeslice;
			addVel.z = sumDifferPosition[i].z()/ timeslice;
			//mMassPoints[i]->SetVel(mMassPoints[i]->GetVel() + addVel);
		}
	}

	void  guideWireInternalResponse::AddBendForce(double timeslice)
	{
		double mBendModulus = 5000.0 ;  // the max value:60000  6000 * 0.02;

		//GK::Vector3 bendForce1(0.0, 0.0, 0.0);
		//GK::Vector3 bendForce2(0.0, 0.0, 0.0);
		//GK::Vector3 bendForce3(0.0, 0.0, 0.0);
		//GK::Vector3 leftEdge;
		//GK::Vector3 rightEdge;
		//GK::Vector3 kb;
		//GK::Vector3 crossProd;
		//double dotProd;

		//for (dtkID id = 1; id < mMassPoints.size() - 1; id++)
		//{
		//	leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
		//	rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
		//	dotProd = GK::DotProduct(leftEdge, rightEdge);
		//	crossProd = GK::CrossProduct(leftEdge, rightEdge);
		//	kb = crossProd / (0.5 * (mSegInterval[id-1] * mSegInterval[id] + dotProd));

		//	if (GK::DotProduct(kb, kb) < 0.0000001)
		//		continue;

		//	bendForce1 = - 2 * GK::CrossProduct(rightEdge, kb) +  rightEdge * GK::DotProduct(kb, kb);                                       // left
		//	bendForce2 = 2 * GK::CrossProduct(leftEdge + rightEdge, kb) - (rightEdge - leftEdge)*GK::DotProduct(kb, kb);      // middle
		//	bendForce3 = -(bendForce1 + bendForce2);																														 // right

		//	double coefficient = (-2*mBendModulus / (mSegInterval[id-1] + mSegInterval[id]) ) / (mSegInterval[id-1]*mSegInterval[id] + dotProd) ;
		//	bendForce1 = bendForce1 * coefficient;
		//	bendForce2 = bendForce2 * coefficient;
		//	bendForce3 = bendForce3 * coefficient;

		//	dtkT3<double> bendForceT1 = vector3TodtkT3(bendForce1);
		//	dtkT3<double> bendForceT2 = vector3TodtkT3(bendForce2);
		//	dtkT3<double> bendForceT3 = vector3TodtkT3(bendForce3);

		//	mMassPoints[id-1]->AddForce(bendForceT1 );
		//	mMassPoints[id]->AddForce(bendForceT2 );
		//	mMassPoints[id+1]->AddForce(bendForceT3 );
		//}

		/*GK::Vector3 rightEdge1, rightEdge2;
		GK::Vector3 crossProduct1, crossProduct2;
		double lengthRightEdge1, lengthRightEdge2;
		double theta;
		GK::Vector3 bendForceDirection;
		dtkT3<double> bendForce;
		for (dtkID i = 0; i <= mMassPoints.size() - 3; i++)
		{
			rightEdge1 = mPts->GetPoint(i) - mPts->GetPoint(i + 1);
			rightEdge2 = mPts->GetPoint(i + 1) - mPts->GetPoint(i + 2);
			lengthRightEdge1 = GK::Length(rightEdge1);
			lengthRightEdge2 = GK::Length(rightEdge2);
			theta = acos(GK::DotProduct(rightEdge1, rightEdge2) / (lengthRightEdge1 * lengthRightEdge2) );


			crossProduct1 = GK::CrossProduct(rightEdge1, rightEdge2);
			if (GK::Length(crossProduct1)  > 0.00001)
			{
				bendForceDirection = GK::Normalize(GK::CrossProduct(GK::CrossProduct(rightEdge1, rightEdge2), rightEdge1) );
				bendForce = vector3TodtkT3(mBendModulus * theta * bendForceDirection);
				mMassPoints[i]->AddForce(bendForce,false);
			}
		}*/

		GK::Vector3 bendForce1(0.0, 0.0, 0.0);
		GK::Vector3 bendForce2(0.0, 0.0, 0.0);
		GK::Vector3 bendForce3(0.0, 0.0, 0.0);
		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		GK::Vector3 crossProd;
		double dotProd;
		double theta;


		for (dtkID id = 1; id < mMassPoints.size() - 1; id++)
		{
			leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
			rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
			dotProd = GK::DotProduct(leftEdge, rightEdge);
			crossProd = GK::CrossProduct(leftEdge, rightEdge);

			if (GK::DotProduct(crossProd, crossProd) < 0.0000001)
				continue;
			theta = acos(dotProd / (GK::Length(leftEdge) * GK::Length(rightEdge)) );

			bendForce1 = (- mBendModulus * theta) * GK::Normalize(GK::CrossProduct(crossProd, leftEdge) ) ;                                       // left
			bendForce3 = (- mBendModulus * theta) * GK::Normalize(GK::CrossProduct(crossProd, rightEdge) );																													 // right
			bendForce2 = -(bendForce1 + bendForce3);      // middle

			dtkT3<double> bendForceT1 = vector3TodtkT3(bendForce1);
			dtkT3<double> bendForceT2 = vector3TodtkT3(bendForce2);
			dtkT3<double> bendForceT3 = vector3TodtkT3(bendForce3);

			mMassPoints[id-1]->AddForce(bendForceT1 );
			mMassPoints[id]->AddForce(bendForceT2 );
			mMassPoints[id+1]->AddForce(bendForceT3 );
		}
	}


	void guideWireInternalResponse::AddTwistForce(double timeslice)
	{
		if (mTwist == 0)
			return ;

		double mTwistModulus = 1.0 * 50000 * 0.001 ;

		GK::Vector3 TwistForce1(0.0, 0.0, 0.0);
		GK::Vector3 TwistForce2(0.0, 0.0, 0.0);
		GK::Vector3 TwistForce3(0.0, 0.0, 0.0);
		GK::Vector3 TwistForce(0.0, 0.0, 0.0);
		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		GK::Vector3 kb;
		GK::Vector3 crossProd;
		double dotProd;

		for (dtkID id = 1; id < mMassPoints.size() - 1; id++)
		{
			leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
			rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
			dotProd = GK::DotProduct(leftEdge, rightEdge);
			crossProd = GK::CrossProduct(leftEdge, rightEdge);
			kb = crossProd / (0.5 * (mSegInterval[id-1] * mSegInterval[id] + dotProd));

			if (GK::DotProduct(kb, kb) < 0.0000001)
				continue;

			TwistForce1 = kb / (-2*mSegInterval[id-1]);
			TwistForce3 = -kb / (-2*mSegInterval[id]);
			TwistForce2 = -(TwistForce1 + TwistForce2); 

			double coefficient = mTwistModulus * mTwist / ( (mSegInterval[id-1] + mSegInterval[id] ) * mPts->GetNumberOfPoints()) ;
			TwistForce1 = TwistForce1 * coefficient;
			TwistForce2 = TwistForce2 * coefficient;
			TwistForce3 = TwistForce3 * coefficient;

			dtkT3<double> TwistForceT1 = vector3TodtkT3(TwistForce1);  // left
			dtkT3<double> TwistForceT3 = vector3TodtkT3(TwistForce3);  // right
			dtkT3<double> TwistForceT2 = vector3TodtkT3(TwistForce2);  // middle

			mMassPoints[id-1]->AddForce(TwistForceT1);
			mMassPoints[id]->AddForce(TwistForceT2);
			mMassPoints[id+1]->AddForce(TwistForceT3);
		}


		//TwistForce = mTwistModulus * mTwistInterval / (mSegInterval * mPts->GetNumberOfPoints()) * TwistForce;
		/*	double TwistLength = GK::Length(TwistForce);
		dtkT3<double> TwistForceT3 = vector3TodtkT3(TwistForce);*/

		/*dtkT3<double> dir = TwistForceT3 / TwistLength;
		dtkT3<double> v = mMassPoints[id]->GetVel();
		if (TwistLength > 1000)
		TwistForceT3 = dir * 1000.0;

		dtkT3<double> dampForce = -dir * ( dot( v, dir ) * (timeslice * mTwistModulus * mDefaultMass * TwistLength * 0.5 ) );
		*/
		if (mTwist > mTwistInterval)
			mTwist -= mTwistInterval;
		else if (mTwist < -mTwistInterval)
			mTwist += mTwistInterval;
		else
			mTwist = 0;

		return ;
	}

	void guideWireInternalResponse::AddContactForce()
	{
		dtkID size = mMassPoints.size();
		for (dtkID i = 0; i < size; i++)
			mMassPoints[i]->AddForce(mContactForces[i]);
	}

	// encapsulated impulse function, as to transparency impulse because the guide wire inextensible
	void guideWireInternalResponse::AddImpulse(dtkID id, const dtkT3<double> & impulse)
	{
		// transmit to left
		GK::Vector3 leftEdge,rightEdge;
		dtkT3<double> leftEdgeT3,rightEdgeT3;
		dtkT3<double> leftImpulse, rightImpulse;
		double leftDot,rightDot;
		dtkID tempLeftID, tempRightID;
		tempLeftID = id;
		tempRightID = id;

		leftImpulse = impulse;
		rightImpulse = impulse;

		//transmit to left
		while (tempLeftID > 0)
		{
			leftEdge = mPts->GetPoint(tempLeftID) - mPts->GetPoint(tempLeftID - 1);
			leftEdgeT3.x = leftEdge.x();
			leftEdgeT3.y = leftEdge.y();
			leftEdgeT3.z = leftEdge.z();

			leftDot = dot(leftEdgeT3, leftImpulse);
			if (leftDot <= 0.0001 &&  leftDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempLeftID]->AddImpulse(leftImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				leftImpulse = dot(leftImpulse, leftEdgeT3) / dot(leftEdgeT3, leftEdgeT3) * leftEdgeT3 ;
				mMassPoints[tempLeftID-1]->AddImpulse(leftImpulse, false);
			}

			tempLeftID -- ;
		}

		// transmit to right
		while (tempRightID + 1 < mPts->GetMaxID()) 
		{
			rightEdge = mPts->GetPoint(tempRightID+1) - mPts->GetPoint(tempRightID);
			rightEdgeT3.x = rightEdge.x();
			rightEdgeT3.y = rightEdge.y();
			rightEdgeT3.z = rightEdge.z();

			rightDot = dot(rightEdgeT3, rightImpulse);
			if (rightDot <= 0.0001&&  rightDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempRightID]->AddImpulse(rightImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				rightImpulse = dot(rightImpulse, rightEdgeT3) / dot(rightEdgeT3, rightEdgeT3) * rightEdgeT3 ;
				mMassPoints[tempRightID + 1]->AddImpulse(rightImpulse, false);

			}

			tempRightID ++;
		}

		mMassPoints[id]->AddImpulse(impulse, false);
		return ;
	}

	// encapsulated force function, as to transparency force because the guide wire inextensible
	void guideWireInternalResponse::AddForce(dtkID id, const dtkT3<double> & force)
	{
		// transmit to left
		GK::Vector3 leftEdge,rightEdge;
		dtkT3<double> leftEdgeT3,rightEdgeT3;
		dtkT3<double> leftForce, rightForce;
		double leftDot,rightDot;
		dtkID tempLeftID, tempRightID;
		tempLeftID = id;
		tempRightID = id;

		leftForce = force;
		rightForce = force;

		//transmit to left
		while (tempLeftID > 0)
		{
			leftEdge = mPts->GetPoint(tempLeftID) - mPts->GetPoint(tempLeftID - 1);
			leftEdgeT3.x = leftEdge.x();
			leftEdgeT3.y = leftEdge.y();
			leftEdgeT3.z = leftEdge.z();

			leftDot = dot(leftEdgeT3, leftForce);
			if (leftDot <= 0.0001 &&  leftDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempLeftID]->AddImpulse(leftImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				leftForce = dot(leftForce, leftEdgeT3) / dot(leftEdgeT3, leftEdgeT3) * leftEdgeT3 ;
				mMassPoints[tempLeftID-1]->AddForce(leftForce, false);
			}

			tempLeftID -- ;
		}

		// transmit to right
		while (tempRightID < mPts->GetMaxID()) 
		{
			rightEdge = mPts->GetPoint(tempRightID+1) - mPts->GetPoint(tempRightID);
			rightEdgeT3.x = rightEdge.x();
			rightEdgeT3.y = rightEdge.y();
			rightEdgeT3.z = rightEdge.z();

			rightDot = dot(rightEdgeT3, rightForce);
			if (rightDot <= 0.0001&&  rightDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempRightID]->AddImpulse(rightImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				rightForce = dot(rightForce, rightEdgeT3) / dot(rightEdgeT3, rightEdgeT3) * rightEdgeT3 ;
				mMassPoints[tempRightID + 1]->AddForce(rightForce, false);

			}

			tempRightID ++;
		}

		mMassPoints[id]->AddForce(force, false);
		return ;
	}

	// collision response: position changes transparency
	void guideWireInternalResponse::AddPosition(dtkID id, const GK::Vector3 & positionChange)
	{
		// transmit to left
		GK::Vector3 leftEdge,rightEdge;
		GK::Vector3 leftPositionChange, rightPositionChange;
		double leftDot,rightDot;
		dtkID tempLeftID, tempRightID;
		tempLeftID = id;
		tempRightID = id;

		leftPositionChange = positionChange;
		rightPositionChange = positionChange;

		//transmit to left
		while (tempLeftID > 0)
		{
			leftEdge = mPts->GetPoint(tempLeftID) - mPts->GetPoint(tempLeftID - 1);
		
			leftDot = GK::DotProduct(leftEdge, leftPositionChange);
			if (leftDot <= 0.0001 &&  leftDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempLeftID]->AddImpulse(leftImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				leftPositionChange = GK::DotProduct(leftPositionChange, leftEdge) / GK::DotProduct(leftEdge, leftEdge) * leftEdge ;
				mMassPoints[tempLeftID-1]->SetPoint(mMassPoints[tempLeftID-1]->GetPoint() + leftPositionChange);
			}

			tempLeftID -- ;
		}

		// transmit to right
		while (tempRightID + 1 < mPts->GetMaxID()) 
		{
			rightEdge = mPts->GetPoint(tempRightID+1) - mPts->GetPoint(tempRightID);
		
			rightDot = GK::DotProduct(rightEdge, rightPositionChange);
			if (rightDot <= 0.0001&&  rightDot >= -0.0001)
			{
				// stop transmit
				//mMassPoints[tempRightID]->AddImpulse(rightImpulse, false);
				break;
			}
			else
			{
				// transmit, continue
				rightPositionChange = GK::DotProduct(rightPositionChange, rightEdge) / GK::DotProduct(rightEdge, rightEdge) * rightEdge ;
				mMassPoints[tempRightID + 1]->SetPoint(mMassPoints[tempRightID+1]->GetPoint() + rightPositionChange); 

			}

			tempRightID ++;
		}

		mMassPoints[id]->SetPoint(mMassPoints[id]->GetPoint() + positionChange); 

	}

	void guideWireInternalResponse::ConstructGuideWire(dtkPoints::Ptr points, const std::vector<double> & segInterval)
	{
		this->SetPoints(points);
		for( dtkID i = 0; i < points->GetNumberOfPoints(); i++ )
		{
			this->AddMassPoint( i, mDefaultMass, dtkT3<double>(0,0,0), mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel );
		}
		mSegInterval = segInterval;
	}

	void guideWireInternalResponse::SetPoints(dtkPoints::Ptr points)
	{
		dtkAssert(points.get() != NULL, NULL_POINTER);
		mPts = points;
		// inition last frame vector

		nextFramePos.resize(mPts->GetNumberOfPoints());
		nextFrameVel.resize(mPts->GetNumberOfPoints());

		externForce.resize(mPts->GetNumberOfPoints());
		mContactForces.resize(mPts->GetNumberOfPoints(), dtkDouble3(0, 0, 0));

		for (dtkID i = 0; i < mPts->GetNumberOfPoints(); i++)
		{
			externForce[i] = dtkT3<double>(0.0, 0.0, 0.0);
		}
	}
	dtkPoints::Ptr guideWireInternalResponse::GetPoints()
	{
		return mPts;
	}


	const GK::Point3& guideWireInternalResponse::GetPoint(dtkID id) const
	{
		dtkAssert(mPts.get() != NULL, NULL_POINTER);
		return mPts->GetPoint(id);
	}

	void guideWireInternalResponse::SetPoint(dtkID id, const GK::Point3 &coord)
	{
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);
		dtkAssert(mPts->SetPoint(id, coord), OUT_OF_RANGE);
	}

	dtkID guideWireInternalResponse::AddMassPoint(dtkID id, const double& mass , const dtkT3<double>& vel , double pointDamp, double pointResistence , dtkT3<double> defaultGravityAccel  )
	{
		mMassPoints.push_back(new dtkPhysMassPoint(id, mPts, mass, vel, pointDamp, pointResistence, defaultGravityAccel ));
		return mMassPoints.size() - 1;
	}



	void guideWireInternalResponse::SetPointMass(dtkID id, double newMass)
	{
		if(id >= mMassPoints.size() || id < 0)
			return;
		(mMassPoints[id])->SetMass(newMass);
	}


	void guideWireInternalResponse::ApplyExternalVelImpulse(const dtkT3<double> & velImpulse)
	{
		//AddImpulse(mMassPoints.size() - 1, velImpulse);
		if (velImpulse.y > 0 && velImpulse.x == 0)
		{
			double velImpulseLength = length(velImpulse);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[0]->GetPoint() - mMassPoints[1]->GetPoint());
			dtkT3<double> verticalEdgeImpulse = velImpulseLength * normalize(edge);

			AddImpulse(0, verticalEdgeImpulse);
			mBaseDirectionTage = true;
		}

		else if (velImpulse.x != 0)
		{
			//mMassPoints[60]->AddImpulse(velImpulse, false);
			double velImpulseLength = length(velImpulse);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[0]->GetPoint() - mMassPoints[1]->GetPoint());
			dtkT3<double> crossProd = cross(edge, velImpulse);
			dtkT3<double> verticalEdgeImpulse = velImpulseLength * cross(crossProd, edge);

			AddImpulse(0, verticalEdgeImpulse);
			//externForce[0] = velImpulse;
			mBaseDirectionTage = true;
		}

		else if (velImpulse.y < 0)
		{
			double velImpulseLength = length(velImpulse);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[mMassPoints.size() - 1]->GetPoint() - mMassPoints[mMassPoints.size() - 2]->GetPoint());
			dtkT3<double> verticalEdgeImpulse = velImpulseLength * normalize(edge);

			AddImpulse(mMassPoints.size() - 1, verticalEdgeImpulse);
			mBaseDirectionTage = false;
		}
	}

	void guideWireInternalResponse::ApplyExternalForce(const dtkT3<double> & force)
	{
		//AddImpulse(mMassPoints.size() - 1, velImpulse);
		if (force.y > 0 && force.x == 0)
		{
			double forceLength = length(force);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[0]->GetPoint() - mMassPoints[1]->GetPoint());
			dtkT3<double> verticalEdgeForce = forceLength * normalize(edge);

			AddForce(0, verticalEdgeForce);
			mBaseDirectionTage = true;
		}

		else if (force.x != 0)
		{
			//mMassPoints[60]->AddImpulse(velImpulse, false);
			double forceLength = length(force);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[0]->GetPoint() - mMassPoints[1]->GetPoint());
			dtkT3<double> crossProd = cross(edge, force);
			dtkT3<double> verticalEdgeForce = forceLength * cross(crossProd, edge);

			AddForce(0, verticalEdgeForce);
			//externForce[0] = velImpulse;
			mBaseDirectionTage = true;
		}

		else if (force.y < 0)
		{
			double forceLength = length(force);
			dtkT3<double> edge = vector3TodtkT3(mMassPoints[mMassPoints.size() - 1]->GetPoint() - mMassPoints[mMassPoints.size() - 2]->GetPoint());
			dtkT3<double> verticalEdgeForce = forceLength * normalize(edge);

			AddForce(mMassPoints.size() - 1, verticalEdgeForce);
			mBaseDirectionTage = false;
		}
	}


	void guideWireInternalResponse::ApplyExternalTwist(double twist)
	{
		//mTwist = mTwist + twist;
	
	}

	bool guideWireInternalResponse::ApplyImpulse( double timeslice )
	{
		for(dtkID i = 0; i < mMassPoints.size(); i++)
			mMassPoints[i]->ApplyImpulse();

		return true;
	}
	
	void guideWireInternalResponse::RotateMassPoint(dtkPoints::Ptr  points, dtkID rotatePointID, dtkID axisPointID1, dtkID axisPointID2, double angle)
	{
		// 平移旋转轴的前一个顶点到原点
		GK::Point3 point = points->GetPoint(rotatePointID);
		GK::Point3 axisPoint1 = points->GetPoint(axisPointID1);
		GK::Point3 axisPoint2 = points->GetPoint(axisPointID2);

		GK::Point3 translatedPoint = point + (GK::Point3(0, 0, 0) - axisPoint1);
		// 构造旋转矩阵
		GK::Vector3 rotateUnitAxis = GK::Normalize(axisPoint2 - axisPoint1);
		double cosAngle = cos(angle);
		double sinAngle = sin(angle);
		double oneMinusCosAngle = 1 - cosAngle;

		double u = rotateUnitAxis[0];
		double v = rotateUnitAxis[1];
		double w = rotateUnitAxis[2];

		double rotateMatrix[3][3];
		rotateMatrix[0][0] = u * u * oneMinusCosAngle + cosAngle;
		rotateMatrix[0][1] = u * v * oneMinusCosAngle - w * sinAngle;
		rotateMatrix[0][2] = u * w * oneMinusCosAngle + v * sinAngle;

		rotateMatrix[1][0] = u * v * oneMinusCosAngle + w * sinAngle;
		rotateMatrix[1][1] = v * v * oneMinusCosAngle + cosAngle;
		rotateMatrix[1][2] = v * w * oneMinusCosAngle - u * sinAngle;

		rotateMatrix[2][0] = u * w * oneMinusCosAngle - v * sinAngle;
		rotateMatrix[2][1] = v * w * oneMinusCosAngle + u * sinAngle;
		rotateMatrix[2][2] = w * w * oneMinusCosAngle + cosAngle;
		
		// 根据旋转矩阵对质点进行旋转
		double x = translatedPoint.x();
		double y = translatedPoint.y();
		double z = translatedPoint.z();

		double rotatedPointx = x * rotateMatrix[0][0] + y * rotateMatrix[0][1] + z * rotateMatrix[0][2];
		double rotatedPointy = x * rotateMatrix[1][0] + y * rotateMatrix[1][1] + z * rotateMatrix[1][2];
		double rotatedPointz = x * rotateMatrix[2][0] + y * rotateMatrix[2][1] + z * rotateMatrix[2][2];
		
		// 进行相应的反平移
		points->SetPoint(rotatePointID, GK::Point3(rotatedPointx, rotatedPointy, rotatedPointz) +  (axisPoint1 - GK::Point3(0, 0, 0)) );
	}
}
