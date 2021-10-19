#include "collisionResponse.h"
#include "dtkCollisionDetectPrimitive.h"

namespace dtk
{
	void collisionResponse::Update( double timeslice, std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, 
		const std::vector< dtkID3>& avoid_1 , double stiffness)
	{
		dtkID size = intersectResults.size();
		for (dtkID i = 0; i < size; i++)
		{
			dtkIntersectTest::IntersectResult::Ptr result = intersectResults[i];
			dtkCollisionDetectPrimitive* pri_1;
			dtkCollisionDetectPrimitive* pri_2;
			result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1 );
			result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2 );

			dtkCollisionDetectPrimitive::Type type1 = pri_1->GetType();
			dtkCollisionDetectPrimitive::Type type2 = pri_2->GetType();

			bool needAvoid = false;

			for (int i = 0; i < avoid_1.size(); i++)
			{
				if (avoid_1[i].a == pri_1->mDetailIDs[0] && avoid_1[i].b == pri_1->mDetailIDs[1] && avoid_1[i].c == pri_1->mDetailIDs[2] )
				{
					needAvoid = true;
					break;
				}
				if (avoid_1[i].a == pri_1->mDetailIDs[0] && avoid_1[i].b == pri_1->mDetailIDs[2] && avoid_1[i].c == pri_1->mDetailIDs[1] )
				{
					needAvoid = true;
					break;
				}
				if (avoid_1[i].a == pri_1->mDetailIDs[1] && avoid_1[i].b == pri_1->mDetailIDs[0] && avoid_1[i].c == pri_1->mDetailIDs[2] )
				{
					needAvoid = true;
					break;
				}
				if (avoid_1[i].a == pri_1->mDetailIDs[1] && avoid_1[i].b == pri_1->mDetailIDs[2] && avoid_1[i].c == pri_1->mDetailIDs[0] )
				{
					needAvoid = true;
					break;
				}
				if (avoid_1[i].a == pri_1->mDetailIDs[2] && avoid_1[i].b == pri_1->mDetailIDs[0] && avoid_1[i].c == pri_1->mDetailIDs[1] )
				{
					needAvoid = true;
					break;
				}
				if (avoid_1[i].a == pri_1->mDetailIDs[2] && avoid_1[i].b == pri_1->mDetailIDs[1] && avoid_1[i].c == pri_1->mDetailIDs[0] )
				{
					needAvoid = true;
					break;
				}
			}
			
			if (needAvoid)
				continue;

			if ( type1 == dtkCollisionDetectPrimitive::TRIANGLE )
				if (type2 == dtkCollisionDetectPrimitive::SPHERE )
				{
					GK::Vector3 normal;
					result->GetProperty( dtkIntersectTest::INTERSECT_NORMAL, normal );
					double lengthPuncture = GK::Length(normal); 

					dtkPhysMassPoint* massPoint21 = mGuideWireMassPoints[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] );

					dtkT3<double> impulse( normal[0], normal[1], normal[2] );

					//------------------------------------------------
					double impulseSquare = dot(impulse, impulse);
					dtkT3<double> proj1 = dot(massPoint21->GetVel(), impulse) / impulseSquare * impulse;
					//-------------------------------------------------
		
					// 剔除碰撞检测的假点
				/*	GK::Point3 trianglePoint = (pri_1->mPts)->GetPoint(pri_1->mDetailIDs[0]);
					dtkID sphereCenterID = pri_2->mDetailIDs[0];
					double distanceLast1, distanceLast2, distanceCenter, distanceNext1, distanceNext2; 
					distanceLast1 = distanceLast2 = 0;
					if ( sphereCenterID >= 2 )
					{
						GK::Point3 last1SphereCenter = (pri_2->mPts)->GetPoint(sphereCenterID - 2);
						distanceLast1 = GK::DotProduct(normal, last1SphereCenter - trianglePoint);
					}
					else if ( sphereCenterID >= 1 )
					{
						GK::Point3 last2SphereCenter = (pri_2->mPts)->GetPoint(sphereCenterID - 1);
						distanceLast2 = GK::DotProduct(normal, last2SphereCenter - trianglePoint);
					}	
						
					GK::Point3 sphereCenter	 = (pri_2->mPts)->GetPoint(pri_2->mDetailIDs[0]);
					GK::Point3 nextSphereCenter = (pri_2->mPts)->GetPoint(pri_2->mDetailIDs[0] + 1);
					GK::Point3 nextNextSphereCenter = (pri_2->mPts)->GetPoint(pri_2->mDetailIDs[0] + 2);

					distanceCenter = GK::DotProduct(normal, sphereCenter - trianglePoint);
					distanceNext1 = GK::DotProduct(normal, nextSphereCenter - trianglePoint);
					distanceNext2 = GK::DotProduct(normal, nextNextSphereCenter - trianglePoint);


					if (distanceLast2 < 0 && distanceCenter < 0 && distanceNext1 < 0)
						return;*/
					
					if (lengthPuncture <=0) 
						return ;


					dtkT3<double> impulse0 = -proj1 * (massPoint21->GetMass() ) * 1.0;

					// compute contact force
					dtkT3<double> contactForce = impulse / timeslice / timeslice * 8.0;
					// 如果是导丝的尖端碰到血管壁，则使每个尖端都受到这样的接触力

					dtkPoints::Ptr points = mGuideWireMassPoints[1]->GetPoints();
					if (massPoint21->GetPointID() <= mGuideWireMassPoints[1]->GetLastTipID())
					{
						//if (length(impulse / timeslice / timeslice * 4.0) > length(mGuideWireMassPoints[1]->mContactForces[0]) )
						//{
						//	for (dtkID i = 0; i < mGuideWireMassPoints[1]->GetLastTipID(); i++)
						//	{
						//		mGuideWireMassPoints[1]->SetContactForces(massPoint21->GetPointID(), \
						//			impulse / timeslice / timeslice * 4.0);	// 8.0
						//	}
						//}
						mGuideWireMassPoints[1]->SetContactForces(massPoint21->GetPointID(), \
										impulse / timeslice / timeslice * 4.0);	// 8.0

					}
					else
						mGuideWireMassPoints[1]->SetContactForces(massPoint21->GetPointID(), \
						impulse / timeslice / timeslice * 6.0);    // 16.0

					// 设置导丝质点的碰撞标志
					mGuideWireMassPoints[1]->SetCollisionFlag(massPoint21->GetPointID(), true);
				}
				else
					assert(false);
			else
				assert(false);
		}
	}

	collisionResponse::collisionResponse(dtkStaticTriangleMesh::Ptr ptr)
	{
		mVascularSurfaceMesh = ptr;
		// smooth vascular surface normals
		const std::vector<dtkID3>& triFacets = mVascularSurfaceMesh->GetECTable(); 
		mVascularSurfaceNormals.clear();
		dtkPoints::Ptr points= mVascularSurfaceMesh->GetPoints();
		mVascularSurfaceNormals.resize( mVascularSurfaceMesh->GetPoints()->GetNumberOfPoints(), GK::Vector3(0, 0, 0) );
		// initial the contact forces


		GK::Vector3 normal;

		for( dtkID i = 0; i < triFacets.size(); i++ )
		{
			GK::Point3 p1 = points->GetPoint(triFacets[i][0]);
			GK::Point3 p2 = points->GetPoint(triFacets[i][1]);
			GK::Point3 p3 = points->GetPoint(triFacets[i][2]);

			normal = GK::CrossProduct( p1 - p2, p3 - p2 );
			mVascularSurfaceNormals[triFacets[i][0]] = mVascularSurfaceNormals[triFacets[i][0]] + normal;
			mVascularSurfaceNormals[triFacets[i][1]] = mVascularSurfaceNormals[triFacets[i][1]] + normal;
			mVascularSurfaceNormals[triFacets[i][2]] = mVascularSurfaceNormals[triFacets[i][2]] + normal;
		}

		for( dtkID i = 0; i < mVascularSurfaceNormals.size(); i++ )
		{
			if (mVascularSurfaceNormals[i].x() == 0 && mVascularSurfaceNormals[i].y() == 0 && mVascularSurfaceNormals[i].z() == 0)
			{
				double nValue = std::sqrt(1/3.0);
				mVascularSurfaceNormals[i] = GK::Vector3(nValue, nValue, nValue); 
			}
			else
				mVascularSurfaceNormals[i] = GK::Normalize( mVascularSurfaceNormals[i] );
		}
	}

	collisionResponse::~collisionResponse()
	{
		// nothing
	}
}