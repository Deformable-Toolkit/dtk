#include "guideWire.h"

namespace dtk
{
	guideWire::guideWire()
	{
		mDefaultGravityAccel = 0.0;
		mDefaultPointDamp = 1.0;
		mIncompleteSegLength = 0.0;
	}

	// 导丝的动态更新
	void guideWire::Update(double timeslice,  ItrMethod method, dtkID iteration)
	{
		switch (method)
		{
		case Euler:
			UpdateMassPoints(timeslice, method, iteration);
			break;

		case Collision:
			UpdateMassPoints(timeslice , Verlet, iteration);
			//yzk add
			ResetContactForces();
			// yzk
			ResistStretch(timeslice);
			break;
		}
	}
	
	void  guideWire::UpdateMassPoints(double timeslice, ItrMethod  method, dtkID iteration)
	{
		for (dtkID i = 0; i <= mMassPoints.size() - 1; i++)
		{
			mMassPoints[i]->Update(timeslice, method, iteration);
		}
	}

	void guideWire::PreUpdate(double timeslice)
	{
		AddBendForce(timeslice);
		// add yzk
		Add3DBendForce(timeslice);

		AddContactForce();
	}

	// 导丝的功能函数
	void guideWire::RemovePoint()
	{
		mPts->DeletePoint(mPts->GetMaxID());
	}

	void guideWire::AddPoint(const GK::Point3 & point)
	{
		mPts->InsertPoint(mPts->GetNumberOfPoints() , point);
	}

	// 导丝的模型函数
	void guideWire::ConstructGuideWireMassPoints() 
	{
		// 根据已有的导丝点的数目构造质点
		for( dtkID i = 0; i < mPts->GetNumberOfPoints(); i++ )
		{
			this->AddMassPoint( dtkT3<double>(0,0,0) );
		}
		mContactForces.resize(mPts->GetNumberOfPoints(), dtkT3<double>(0, 0, 0));
		mMassPointsCollisionFlag.resize(mPts->GetNumberOfPoints(), false);
	}

	dtkPhysMassPoint* guideWire::GetMassPoint(dtkID id)  const
	{
		return mMassPoints[id];
	}

	dtkID guideWire::AddMassPoint(const dtkT3<double>& vel)
	{
		// 导丝的尖端与导丝的末端在血管中的摩擦系数不同
		if (mMassPoints.size() <= mLastTipID)
			mMassPoints.push_back(new dtkPhysMassPoint(mMassPoints.size(), mPts, mMass, vel, mDefaultPointDamp, mDefaultTipPointResistence, mDefaultGravityAccel ));
		else
			mMassPoints.push_back(new dtkPhysMassPoint(mMassPoints.size(), mPts, mMass, vel, mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel ));

		mContactForces.push_back(dtkDouble3(0, 0, 0));
		mMassPointsCollisionFlag.push_back(false);
		return mMassPoints.size() - 1;
	}

	bool guideWire::RemoveMassPoint()
	{
		if (mMassPoints.size())
		{
			delete mMassPoints[mMassPoints.size() - 1];
			mMassPoints.pop_back();
			mContactForces.pop_back();
			return true;
		}

		return false;
	}

	void guideWire::DynamicGuideWirePoint()
	{
		// 判断是否需要增删导丝点
		GK::Point3 currentLastPoint = mPts->GetPoint(mPts->GetMaxID());
		GK::Vector3 lastDirection = mPts->GetPoint(mPts->GetMaxID() - 1) - mPts->GetPoint(mPts->GetMaxID() );
		mIncompleteSegLength =  GK::DotProduct(mGuideWireStartPoint - currentLastPoint, lastDirection) ;
		mIncompleteSegLength = mIncompleteSegLength - (mLastTipID  + 2) * mSegInterval;
		while (mIncompleteSegLength > mSegInterval)
		{
			// 移除一个点
			RemoveMassPoint();
			RemovePoint();
			currentLastPoint = mPts->GetPoint(mPts->GetMaxID());
			mIncompleteSegLength =  GK::DotProduct(mGuideWireStartPoint - currentLastPoint, lastDirection) ;
			mIncompleteSegLength = mIncompleteSegLength - (mLastTipID / 2 + 2) * mSegInterval;
		}

		while (mIncompleteSegLength < 0)
		{
			// 增加一个点
			AddPoint(currentLastPoint - mSegInterval * lastDirection);
			AddMassPoint(dtkT3<double>(0, 0, 0));
			currentLastPoint = mPts->GetPoint(mPts->GetMaxID());
			mIncompleteSegLength =  GK::DotProduct(mGuideWireStartPoint - currentLastPoint, lastDirection) ;
			mIncompleteSegLength = mIncompleteSegLength - (mLastTipID / 2+ 2) * mSegInterval;
		}
	}

	void guideWire::ResistStretch(double timeslice)
	{
		GK::Vector3 edge;
		double length;
		double lambad;
		vector <GK::Vector3> sumDifferPosition(mMassPoints.size(), GK::Vector3(0.0, 0.0, 0.0));
		GK::Vector3 differPosition;

		dtkT3<double> addVel;
		int count = 0;
		int constraintTimes = 10;   // 10

		while (count < constraintTimes)
		{
			count++;

			vector <GK::Vector3> eachDifferPosition(mMassPoints.size(), GK::Vector3(0.0, 0.0, 0.0));
			for (dtkID i = 0; i < mLastTipID; i++)                                                                                                                        
			{
				edge = mPts->GetPoint(i+1) -mPts->GetPoint(i);
				length = GK::Length(edge);
				lambad = 0.5 * (length - mTipSegInterval) / length;
				if ( abs(lambad) < 0.001)
					continue;

				differPosition = edge * lambad;
				sumDifferPosition[i] = sumDifferPosition[i] + differPosition;
				sumDifferPosition[i+1] = sumDifferPosition[i+1] - differPosition;
				eachDifferPosition[i] = eachDifferPosition[i] + differPosition;
				eachDifferPosition[i+1] = eachDifferPosition[i+1] - differPosition;
			}

			for (dtkID i = mLastTipID;  i < mMassPoints.size() - 1; i++ )
			{
				edge = mPts->GetPoint(i+1) -mPts->GetPoint(i);
				length = GK::Length(edge);
				lambad = 0.5 * (length - mSegInterval) / length;
				if (abs(lambad) < 0.0001)
					continue;

				differPosition = edge * lambad;
				sumDifferPosition[i] = sumDifferPosition[i] + differPosition;
				sumDifferPosition[i+1] = sumDifferPosition[i+1] - differPosition;
				eachDifferPosition[i] = eachDifferPosition[i] + differPosition;
				eachDifferPosition[i+1] = eachDifferPosition[i+1] - differPosition;
			}

			for (dtkID i = 0; i < mMassPoints.size() ; i++)                                                                                                                        
			{
				mMassPoints[i]->SetPoint(mMassPoints[i]->GetPoint() + eachDifferPosition[i] , false);
			}
		}

		for (dtkID i = 0; i < mMassPoints.size() ; i++)
		{
			addVel.x = sumDifferPosition[i].x()/ timeslice;
			addVel.y = sumDifferPosition[i].y()/ timeslice;
			addVel.z = sumDifferPosition[i].z()/ timeslice;
			mMassPoints[i]->SetVel(mMassPoints[i]->GetVel() + addVel, false);
		}
	}

	void guideWire::AddBendForce(double timeslice)
	{
		GK::Vector3 bendForce1(0.0, 0.0, 0.0);
		GK::Vector3 bendForce2(0.0, 0.0, 0.0);
		GK::Vector3 bendForce3(0.0, 0.0, 0.0);
		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		GK::Vector3 kb;
		GK::Vector3 crossProd;
		double dotProd;
		double theta;

		// 导丝尖端弯曲力的计算
	
		//for (dtkID id = mLastTipID + 1; id >= 1; id--)
		//{
		//	leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
		//	rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
		//	dotProd = GK::DotProduct(leftEdge, rightEdge);
		//	crossProd = GK::CrossProduct(leftEdge, rightEdge);

		//	theta = acos(dotProd / (GK::Length(leftEdge) * GK::Length(rightEdge)) );
		//
		//	// 判断是否会出现Z字型
		//	if (id == mLastTipID)
		//		theta -= mTipOriginAngle[id - 1];
		//	else if (id == mLastTipID + 1);
		//	else
		//	{
		//		// 导丝尖端的弯曲部分要防止出现Z字型
		//		GK::Vector3 rightRightEdge = mPts->GetPoint(id + 2) - mPts->GetPoint(id + 1);
		//		// 用点积来预防Z字型
		//		/*if (acos(GK::DotProduct(leftEdge, rightRightEdge) / (GK::Length(leftEdge)  * GK::Length(rightRightEdge) ) ) <  mTipOriginAngle[id] )
		//			theta += mTipOriginAngle[id - 1];*/
		//		
		//		// 用差积来预防Z字型
		//		if (GK::DotProduct(GK::CrossProduct(rightRightEdge, rightEdge), GK::CrossProduct(rightEdge, leftEdge)) < 0)
		//			theta -= mTipOriginAngle[id - 1];
		//		else
		//			theta -= mTipOriginAngle[id - 1];
		//	}
		//	
		//	if (std::abs(theta) < 0.00001)
		//		continue;

		//	//bendForce1 = (-mTipBendModulus * theta) * GK::Normalize(rightEdge) ;											// left
		//	//bendForce3 = (mTipBendModulus * theta) * GK::Normalize(leftEdge);												// right
		//	//bendForce2 = -(bendForce1 + bendForce3);																							// middle

		//	bendForce1 = (mTipBendModulus * theta) * GK::Normalize(GK::CrossProduct(leftEdge, crossProd));
		//	bendForce3 = (mTipBendModulus * theta) * GK::Normalize(GK::CrossProduct(rightEdge, crossProd));
		//	bendForce2 = - (bendForce1 + bendForce3);

		//	dtkT3<double> bendForceT1 = vector3TodtkT3(bendForce1);
		//	dtkT3<double> bendForceT2 = vector3TodtkT3(bendForce2);
		//	dtkT3<double> bendForceT3 = vector3TodtkT3(bendForce3);

		//	mMassPoints[id-1]->AddForce(bendForceT1 );
		//	mMassPoints[id]->AddForce(bendForceT2 );
		//	mMassPoints[id+1]->AddForce(bendForceT3 );
		//}

		 //导丝体部弯曲力的计算
		for (dtkID id = 1; id < mMassPoints.size() - mLastTipID; id++)
		{
			leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
			rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
			dotProd = GK::DotProduct(leftEdge, rightEdge);
			crossProd = GK::CrossProduct(leftEdge, rightEdge);
			kb = crossProd / (0.5 * (mSegInterval * mSegInterval + dotProd));

			if (GK::DotProduct(kb, kb) < 0.0001 && dotProd > 0)
				continue;
			else if (GK::DotProduct(kb, kb) < 0.0001 && dotProd < 0)
			{
				mMassPoints[id - 1]->SetPoint(mPts->GetPoint(id - 1) + 2 * leftEdge);
				continue;
			}

			bendForce1 = - 2 * GK::CrossProduct(rightEdge, kb) +  rightEdge * GK::DotProduct(kb, kb);                         // left
			bendForce2 = 2 * GK::CrossProduct(leftEdge + rightEdge, kb) - (rightEdge - leftEdge)*GK::DotProduct(kb, kb);      // middle
			bendForce3 = -(bendForce1 + bendForce2);																          // right
			
			double coefficient = (-2*mBendModulus / (mSegInterval + mSegInterval) ) / (mSegInterval*mSegInterval + dotProd) ;
		
			bendForce1 = bendForce1 * coefficient;
			bendForce2 = bendForce2 * coefficient;
			bendForce3 = bendForce3 * coefficient;

			dtkT3<double> bendForceT1 = vector3TodtkT3(bendForce1);
			dtkT3<double> bendForceT2 = vector3TodtkT3(bendForce2);
			dtkT3<double> bendForceT3 = vector3TodtkT3(bendForce3);

			mMassPoints[id-1]->AddForce(bendForceT1 );
			mMassPoints[id]->AddForce(bendForceT2 );
			mMassPoints[id+1]->AddForce(bendForceT3 );
		}
	}

	void guideWire::ResistOverBend(double timeslice)
	{
		GK::Vector3 differPosition1;
		GK::Vector3 differPosition2;
		GK::Vector3 differPosition3;

		dtkT3<double> addVel;
		double constranitValue;

		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		GK::Vector3 constraintGradient1;
		GK::Vector3 constraintGradient2;
		GK::Vector3 constraintGradient3;
		double leftEdgeLength;
		double rightEdgeLength;
		double dotProd;
		double cosTheta; 
		dtkID size = mMassPoints.size();
		vector <GK::Vector3> sumDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));

		int count = 0;
		int constraintTimes =3;
		while (count < constraintTimes)
		{
			count ++;
			vector <GK::Vector3> eachDifferPosition(size, GK::Vector3(0.0, 0.0, 0.0));

			for (dtkID id = 1; id <= mLastTipID + 1; id++)
			{
				if (id == mLastTipID + 1)
					cosTheta = 1;
				else
					cosTheta = cos(mTipOriginAngle[id - 1]);

				leftEdge = mMassPoints[id]->GetPoint() - mMassPoints[id-1]->GetPoint();
				rightEdge = mMassPoints[id+1]->GetPoint() - mMassPoints[id]->GetPoint();
				leftEdgeLength = GK::Length(leftEdge);
				rightEdgeLength = GK::Length(rightEdge);
				dotProd = GK::DotProduct(leftEdge, rightEdge);

				constranitValue = dotProd  - leftEdgeLength * rightEdgeLength * cosTheta;
				constraintGradient1 = - rightEdge - leftEdge * (rightEdgeLength * cosTheta / leftEdgeLength);
				constraintGradient3 = leftEdge + rightEdge * (leftEdgeLength * cosTheta / rightEdgeLength);
				constraintGradient2 = -(constraintGradient1 + constraintGradient3);

				// the angle of tip mass point should be constrainted: bendAngle
				//	

				if (abs(constranitValue) > 0.01)
				{
					double lamda =constranitValue / ( GK::DotProduct(constraintGradient1, constraintGradient1)  \
						+ GK::DotProduct(constraintGradient2, constraintGradient2) +  GK::DotProduct(constraintGradient3, constraintGradient3) );
					differPosition1 = - lamda* constraintGradient1;

					differPosition2 = - lamda * constraintGradient2 ;

					differPosition3 = - lamda * constraintGradient3;

					eachDifferPosition[id-1] = eachDifferPosition[id-1] + differPosition1;
					eachDifferPosition[id] = eachDifferPosition[id] + differPosition2;
					eachDifferPosition[id+1] = eachDifferPosition[id+1] + differPosition3;

					sumDifferPosition[id-1] = sumDifferPosition[id-1] + differPosition1;
					sumDifferPosition[id] = sumDifferPosition[id] + differPosition2;
					sumDifferPosition[id+1] = sumDifferPosition[id+1] + differPosition3;
				}

			}

			for (dtkID i = 0; i <= mLastTipID + 1; i++)                                                                                                                        
			{
				//mMassPoints[i]->SetPoint(mMassPoints[i]->GetPoint() + eachDifferPosition[i] , false);
				mMassPoints[i]->AddForce(vector3TodtkT3(eachDifferPosition[i]  * 100) );
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

	void guideWire::Add3DBendForce(double timeslice)
	{
		// 计算导丝体部的第一段和导丝尖端的最后一段
		GK::Vector3 guideWireBodyFirstVector = mPts->GetPoint(mLastTipID) - mPts->GetPoint(mLastTipID + 1);
		GK::Vector3 guideWireTipLastVector     = mPts->GetPoint(mLastTipID - 1) - mPts->GetPoint(mLastTipID);
		GK::Vector3 planeNormal = GK::CrossProduct(guideWireBodyFirstVector, guideWireTipLastVector);
		
		// 计算3D弯曲力
		GK::Vector3 leftEdge , rightEdge, tipPlaneNormal;
		double theta;
		GK::Vector3 threeDBendForceDirection;
		dtkT3<double> threeDBendForce;
		for (dtkID i = 1; i < mLastTipID; i++)
		{
			leftEdge = mPts->GetPoint(i - 1) - mPts->GetPoint(i);
			rightEdge = mPts->GetPoint(i) - mPts->GetPoint(i + 1);
			tipPlaneNormal = GK::CrossProduct(rightEdge, leftEdge);
			theta = acos(GK::DotProduct(planeNormal, tipPlaneNormal) );
			threeDBendForceDirection = GK::CrossProduct(guideWireBodyFirstVector, GK::CrossProduct(planeNormal, tipPlaneNormal));
			threeDBendForceDirection = GK::Normalize(threeDBendForceDirection);
			threeDBendForce = vector3TodtkT3( m3DBendModulus * theta * threeDBendForceDirection ); 
		}
	}


	void guideWire::AddTwistForce(double twistAngle)
	{
		double twistStiffness = 20000.0;                            // 20000.0 ,  40000.0
		double totalLength = 2 * mPts->GetMaxID() * mSegInterval;
		double coefficient = twistStiffness * twistAngle / (2 * mSegInterval);

		dtkT3<double> twistForce1(0.0, 0.0, 0.0);
		dtkT3<double> twistForce3(0.0, 0.0, 0.0);
		GK::Vector3 leftEdge;
		GK::Vector3 rightEdge;
		GK::Vector3 kb;
		GK::Vector3 crossProd;
		double dotProd;

		for (dtkID id = mLastTipID + 1; id < mPts->GetMaxID() - mLastTipID - 2; id++)
		{
			leftEdge = mPts->GetPoint(id) - mPts->GetPoint(id-1);
			rightEdge = mPts->GetPoint(id+1) - mPts->GetPoint(id);
			dotProd = GK::DotProduct(leftEdge, rightEdge);
			crossProd = GK::CrossProduct(leftEdge, rightEdge);
			kb = crossProd / (0.5 * (mSegInterval * mSegInterval + dotProd));
			twistForce1 = vector3TodtkT3( coefficient * kb );
			twistForce3 = - twistForce1;

			mMassPoints[id - 1]->AddForce(twistForce1);
			mMassPoints[id +1]->AddForce(twistForce3);
		}
	}

	void guideWire::AddContactForce()
	{
		assert(mMassPoints.size() <= mContactForces.size(), "contact force subscript");
		for (dtkID i = 0; i < mMassPoints.size(); i++)
			mMassPoints[i]->AddForce(mContactForces[i]);
	}

	// 对导丝的受力进行操作
	void guideWire::AddForce(const dtkT3<double> & force, dtkID directionID, bool isPush)
	{
		if (isPush)   // 用户在向前推导丝
		{
			// 为导丝的尖端赋值相同的force
			for (dtkID id = 0; id <= directionID; id++)
			{
				mMassPoints[id]->AddForce(force, false);
			}
			// 向导丝后面的体部传播force
			GK::Vector3 rightEdge;
			dtkT3<double> rightEdgeT3;
			dtkT3<double> rightForce;
			double rightDot;
			dtkID tempRightID;
			tempRightID = directionID; 
			rightForce = force;

			while (tempRightID < mPts->GetMaxID())
			{
				rightEdge = mPts->GetPoint(tempRightID+1) - mPts->GetPoint(tempRightID);
				rightEdgeT3.x = rightEdge.x();
				rightEdgeT3.y = rightEdge.y();
				rightEdgeT3.z = rightEdge.z();

				rightDot = dot(rightEdgeT3, rightForce);
				if (rightDot <= 0.0001&&  rightDot >= -0.0001)
				{
					break;
				}
				else
				{
					rightForce = dot(rightForce, rightEdgeT3) / dot(rightEdgeT3, rightEdgeT3) * rightEdgeT3 ;
					mMassPoints[tempRightID + 1]->AddForce(rightForce, false);
				}

				tempRightID ++;
			}
		}

		else				// 用户在向后拉导丝
		{
			// 向导丝前面的体部传播force
			GK::Vector3 leftEdge;
			dtkT3<double> leftEdgeT3;
			dtkT3<double> leftForce;
			double leftDot;
			dtkID tempLeftID;
			tempLeftID = mPts->GetMaxID(); 
			leftForce = force;

			while (tempLeftID > directionID)
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

			// 为导丝的尖端赋值相同的leftForce
			/*for (dtkID j = 0; j <= directionID ; j++)
			{
				mMassPoints[j]->AddForce(leftForce, false);
			}*/
			
			// 为导丝体部的末端赋值force
			mMassPoints[mMassPoints.size() - 1]->AddForce(force, false); 
		}
	}
	

	// 用户对导丝的外力操作
	void guideWire::ApplyExternalForce(const dtkT3<double> & force)
	{
		// 向前推导丝
		double forceLength = length(force);
		dtkT3<double> edge;
		dtkT3<double> verticalEdgeForce(0, 0, 0);
		if (force.y > 0)
		{
			// 首先将力从导丝体部的末端传递到导丝尖端的最后一段
			dtkID id;
			//检查是否有某个点发生碰撞
			for (id = 0; id < mLastTipID; id++)
			{
				if (mMassPointsCollisionFlag[id])
				{
					break;
				}
			}
			edge = vector3TodtkT3(mPts->GetPoint(0) - mPts->GetPoint(1));
			verticalEdgeForce =  forceLength * normalize(edge);
			AddForce(verticalEdgeForce, 0 , true);
		}
		else  // 向后拉导丝
		{
			edge = vector3TodtkT3(mPts->GetPoint(mPts->GetMaxID()) - mPts->GetPoint(mPts->GetMaxID() - 1));
			verticalEdgeForce = forceLength * normalize(edge);
			AddForce(verticalEdgeForce, 0, false);
		}
	}

	void guideWire::RotateMassPoint(dtkPoints::Ptr  points, dtkID rotatePointID, dtkID axisPointID1, dtkID axisPointID2, double angle)
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

	// 导丝的参数设置
	void guideWire::SetPoints(dtkPoints::Ptr points)
	{
		assert(points);
		mPts = points;
	}

	void guideWire::SetSegInterval(double segInterval)
	{
		assert(segInterval > 0);
		mSegInterval = segInterval;
	}

	void guideWire::SetTipSegInterval(double tipSegInterval)
	{
		assert(tipSegInterval > 0);
		mTipSegInterval = tipSegInterval;
	}

	void guideWire::SetLastTipID(dtkID lastTipID)
	{
		assert(lastTipID > 0);
		mLastTipID = lastTipID;
	}

	void guideWire::SetBendModulus(double bendModulus)
	{
		assert(bendModulus > 0);
		mBendModulus = bendModulus;
	}

	void guideWire::SetTipBendModulus(double tipBendModulus)
	{
		mTipBendModulus = tipBendModulus;
	}

	void guideWire::Set3DBendModulus(double threeDBendModulus)
	{
		assert(threeDBendModulus > 0);
		m3DBendModulus = threeDBendModulus;
	}


	void guideWire::SetTipOriginAngle(const vector<double>& tipOriginAngle)
	{
		mTipOriginAngle = tipOriginAngle;
	}

	void guideWire::SetMass(double mass)
	{
		assert(mass > 0);
		mMass = mass;
	}

	void guideWire::SetPointResistence(double pointResistence)
	{
		mDefaultPointResistence = pointResistence;
	}

	void guideWire::SetTipPointResistence(double tipPointResistence)
	{
		mDefaultTipPointResistence = tipPointResistence;
	}

	void guideWire::SetContactForces(dtkID id, const dtkDouble3 & force)
	{
		if ( id >= mContactForces.size() )
		{
			for (dtkID i = mContactForces.size(); i <= id; i++)
				mContactForces.push_back(dtkDouble3(0, 0, 0));
		}
		mContactForces[id] = force;
	}

	void guideWire::SetCollisionFlag(dtkID id, bool flag)
	{
		if (id >= mMassPointsCollisionFlag.size() )
		{
			for (dtkID i = mMassPointsCollisionFlag.size(); i <= id; i++)
				mMassPointsCollisionFlag.push_back(flag);
		}
		mMassPointsCollisionFlag[id] = flag;
	}

	void guideWire::ResetContactForces()
	{
		for (dtkID i = 0; i < mContactForces.size(); i++)
			mContactForces[i] = dtkDouble3(0, 0, 0);
	}

	void guideWire::ResetCollisionFlag()
	{
		for (dtkID i = 0; i < mMassPointsCollisionFlag.size(); i++)
			mMassPointsCollisionFlag[i] = false;
	}

	void guideWire::SetGuideWireStartPoint(const GK::Point3 & point)
	{
		mGuideWireStartPoint = point;
	}

	void guideWire::SetGuideWireStartDirection(const GK::Vector3 & direction)
	{
		mGuideWireStartDirection = direction;
	}

	// 获取导丝的参数
	dtkPoints::Ptr guideWire::GetPoints() const
	{
		return mPts;
	}

	double guideWire::GetSegInterval() const
	{
		return mSegInterval;
	}

	double guideWire::GetPointResistence() const
	{
		return mDefaultPointResistence;
	}

	double guideWire::GetTipPointResistence() const
	{
		return mDefaultTipPointResistence;
	}

	double guideWire::GetTipSegInterval() const
	{
		return mTipSegInterval;
	}

	dtkID guideWire::GetLastTipID() const
	{
		return mLastTipID;
	}

	double guideWire::GetBendModulus() const
	{
		return mBendModulus;
	}

	double guideWire::GetTipBendModulus() const
	{
		return mTipBendModulus;
	}

	double guideWire::Get3DBendModulus() const
	{
		return m3DBendModulus;
	}

	vector<double> guideWire::GetTipOriginAngle() const
	{
		return mTipOriginAngle;
	}

	double guideWire::GetMass() const
	{
		return mMass;
	}

}