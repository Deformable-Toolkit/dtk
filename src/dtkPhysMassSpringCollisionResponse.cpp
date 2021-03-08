#include "dtkPhysMassSpringCollisionResponse.h"
#include "dtkCollisionDetectPrimitive.h"

using namespace std;

namespace dtk
{
	dtkPhysMassSpringCollisionResponse::dtkPhysMassSpringCollisionResponse()
	{
	}

	dtkPhysMassSpringCollisionResponse::~dtkPhysMassSpringCollisionResponse()
	{
	}
        
	// the function compute the mass points get impluse after collision.
    void dtkPhysMassSpringCollisionResponse::Update( double timeslice, 
            vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults,
            const std::vector< dtkInterval<int> >& avoid_1,
            const std::vector< dtkInterval<int> >& avoid_2,
			double stiffness)
    {
		for( dtkID i = 0; i < intersectResults.size(); i++ )
		{
			dtkIntersectTest::IntersectResult::Ptr result = intersectResults[i];
			dtkCollisionDetectPrimitive* pri_1;
			dtkCollisionDetectPrimitive* pri_2;
			result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1 );
			result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2 );

			// eliminate some primitive pair that need to avoid collision responsing.
            bool needAvoid = false;
            for( dtkID avoidID = 0; avoidID < avoid_1.size(); avoidID++ )
            {
                if( avoid_1[avoidID].Contain( pri_1->mMinorID ) )
                {
                    needAvoid = true;
                    break;
                }
            }
            if( needAvoid )
                continue;

            for( dtkID avoidID = 0; avoidID < avoid_2.size(); avoidID++ )
            {
                if( avoid_2[avoidID].Contain( pri_2->mMinorID ) )
                {
                    needAvoid = true;
                    break;
                }
            }
            if( needAvoid )
                continue;

			dtkCollisionDetectPrimitive::Type type1 = pri_1->GetType();
			dtkCollisionDetectPrimitive::Type type2 = pri_2->GetType();

			ResponseType responseType;
			if( type1 == dtkCollisionDetectPrimitive::SEGMENT )
			{
				if( type2 == dtkCollisionDetectPrimitive::SEGMENT )
				{
					responseType = SEGMENT_SEGMENT;
				}
				else if( type2 == dtkCollisionDetectPrimitive::TRIANGLE )
				{
                    assert(false);// only tri - seg pairs are possible
				}
				else
					assert(false);
			}
			else if( type1 == dtkCollisionDetectPrimitive::TRIANGLE )
			{
				if( type2 == dtkCollisionDetectPrimitive::SEGMENT )
				{
					responseType = TRIANGLE_SEGMENT;
				}
				else if( type2 == dtkCollisionDetectPrimitive::TRIANGLE )
				{
					responseType = TRIANGLE_TRIANGLE;
				}
				else
					assert(false);
			}

			switch( responseType )
			{
			case SEGMENT_SEGMENT:
				{
					dtkDouble2 uv1;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_1, uv1 );

					dtkDouble2 uv2;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_2, uv2 );

					GK::Vector3 normal;
					result->GetProperty( dtkIntersectTest::INTERSECT_NORMAL, normal );

					// get four mass point of two segs.
					dtkPhysMassPoint* massPoint11 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint12 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[1] );
					dtkPhysMassPoint* massPoint21 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint22 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[1] );
                    
					// try 
					dtkDouble3 oriPos11 = massPoint11->mPosBuffers[0];
					dtkDouble3 oriPos12 = massPoint12->mPosBuffers[0];
					dtkDouble3 oriPos21 = massPoint21->mPosBuffers[0];
					dtkDouble3 oriPos22 = massPoint22->mPosBuffers[0];

					// compute vertical distance of two segments.
					double penetrate1 = dot( oriPos21 - oriPos11, normalize(cross( oriPos12 - oriPos11, oriPos22 - oriPos21 )) );

					dtkDouble3 Pos11 = massPoint11->GetPosition();
					dtkDouble3 Pos12 = massPoint12->GetPosition();
					dtkDouble3 Pos21 = massPoint21->GetPosition();
					dtkDouble3 Pos22 = massPoint22->GetPosition();

					double penetrate2 = dot( Pos21 - Pos11, normalize(cross( Pos12 - Pos11, Pos22 - Pos21 )) );

					//cout << penetrate1 << " " << penetrate2 << endl;
					// relative relationship of two segments invert.
					// if relationship of two segments changes, normal direction and length also changed. 
					if( penetrate1 * penetrate2 < 0 )
					{
						//cout << normal << endl;
						//cout << "invert- " << endl;
						normal = GK::Normalize(-normal) * ( pri_1->GetExtend()*2.0 + pri_2->GetExtend()*2.0 - GK::Length(normal)/2.0 ) * 2.0;
						//cout << normal << endl;
					}
					dtkDouble3 impulse( normal[0], normal[1], normal[2] );
					
					// ??? 
					impulse = impulse * ( stiffness * timeslice ); 

					// try vel impulse
					dtkDouble3 vel11 = massPoint11->GetVel();
					dtkDouble3 vel12 = massPoint12->GetVel();
					dtkDouble3 vel21 = massPoint21->GetVel();
					dtkDouble3 vel22 = massPoint22->GetVel();

					dtkDouble3 velRelative = vel21 * uv2[0] + vel22 * uv2[1] - vel11 * uv1[0] - vel12 * uv1[1];

					dtkDouble3 impulse_normal = normalize( impulse );
					double relativeVel = dot( velRelative, impulse_normal);
					double candidate1 = - relativeVel 
                        * ( massPoint11->GetMass() + massPoint12->GetMass() + massPoint21->GetMass() + massPoint22->GetMass() );
					//double candidate2 = stiffness * timeslice * GK::Length(normal);
	
					// compute the total impulse
					impulse = impulse + impulse_normal * candidate1;

					//double impulse_value = max( candidate1, candidate2 );

					//impulse = impulse_normal * impulse_value / ( 1 + pow( uv1[0], 2 ) + pow( uv1[1], 2 ) + pow( uv2[0], 2 ) + pow( uv2[1], 2 ) );

					// the process is that assign impluse to each mass point. 
					impulse = impulse / ( 1 + pow( uv1[0], 2 ) + pow( uv1[1], 2 ) + pow( uv2[0], 2 ) + pow( uv2[1], 2 ) );

					massPoint11->AddImpulse( impulse * (-uv1[0] / massPoint11->GetMass() ) );
					massPoint12->AddImpulse( impulse * (-uv1[1] / massPoint12->GetMass() ) );
					massPoint21->AddImpulse( impulse * ( uv2[0] / massPoint21->GetMass() ) );
					massPoint22->AddImpulse( impulse * ( uv2[1] / massPoint22->GetMass() ) );

					//mMassSprings[pri_1->mMajorID]->ImpulsePropagate( impulse * (-uv1[0] / 1.0), pri_1->mDetailIDs[0], 0 );
					//mMassSprings[pri_1->mMajorID]->ImpulsePropagate( impulse * (-uv1[1] / 1.0), pri_1->mDetailIDs[1], 0 );
					//mMassSprings[pri_2->mMajorID]->ImpulsePropagate( impulse * (uv2[0] / 1.0), pri_2->mDetailIDs[0], 0 );
					//mMassSprings[pri_2->mMajorID]->ImpulsePropagate( impulse * (uv2[1] / 1.0), pri_2->mDetailIDs[1], 0 );
					
					// four mass points to two mass points???
					dtkPhysMassPoint* massPoint1112 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[0] + 1 );
					dtkPhysMassPoint* massPoint2122 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] + 1);
					massPoint1112->AddImpulse( impulse * (-0.5 / massPoint1112->GetMass() ) );
					massPoint2122->AddImpulse( impulse * ( 0.5 / massPoint2122->GetMass() ) );

					// ???
					if( pri_1->mDetailIDs[0] > 0 )
					{
						dtkPhysMassPoint* massPoint1011 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[0] - 1 );
						massPoint1011->AddImpulse( impulse * (-uv1[0] * 0.5 / massPoint1011->GetMass() ) );
					}
					if( pri_1->mDetailIDs[1] + 1 < mMassSprings[pri_1->mMajorID]->GetNumberOfMassPoints() )
					{
						dtkPhysMassPoint* massPoint1213 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[1] + 1 );
						massPoint1213->AddImpulse( impulse * (-uv1[1] * 0.5 / massPoint1213->GetMass() ) );
					}

					if( pri_2->mDetailIDs[0] > 0 )
					{
						dtkPhysMassPoint* massPoint2021 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] - 1 );
						massPoint2021->AddImpulse( impulse * (uv2[0] * 0.5 / massPoint2021->GetMass() ) );
					}
					if( pri_2->mDetailIDs[1] + 1 < mMassSprings[pri_2->mMajorID]->GetNumberOfMassPoints() )
					{
						dtkPhysMassPoint* massPoint2223 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[1] + 1 );
						massPoint2223->AddImpulse( impulse * (uv2[1] * 0.5 / massPoint2223->GetMass() ) );
					}
					break;
				}
			case TRIANGLE_SEGMENT:
				{
					GK::Vector3 normal;
					result->GetProperty( dtkIntersectTest::INTERSECT_NORMAL, normal );

					dtkDouble3 uvw1;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_1, uvw1 );

					dtkDouble2 uv2;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_2, uv2 );

                    bool isPiercing = false;
                    for( dtkID pierceID = 0; pierceID < mPierceSegments.size(); pierceID++ )
                    {
                        if( pri_2->mMajorID == mPierceSegments[pierceID][0] 
                                && pri_2->mMinorID == mPierceSegments[pierceID][1] 
                                && ( GK::Length(normal) ) * uv2[0] > ( pri_1->GetExtend() + pri_2->GetExtend() ) )
                        {
                            mPiercingResults.push_back( result );
                            isPiercing = true;
                            break;
                        }
                    }
                    
                    if( isPiercing )
                        break;

                    dtkPhysMassPoint* massPoint11 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint12 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[1] );
					dtkPhysMassPoint* massPoint13 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[2] );
					dtkPhysMassPoint* massPoint21 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint22 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[1] );

					dtkDouble3 impulse( normal[0], normal[1], normal[2] );
					impulse = impulse * ( stiffness * timeslice ); 

					impulse = impulse * 2.0 / ( pow( uvw1[0], 2 ) + pow( uvw1[1], 2 ) + pow( uvw1[2], 2 ) + pow( uv2[0], 2 ) + pow( uv2[1], 2 ) );

					massPoint11->AddImpulse( impulse * ( -uvw1[0] / massPoint11->GetMass() ) );
					massPoint12->AddImpulse( impulse * ( -uvw1[1] / massPoint12->GetMass() ) );
					massPoint13->AddImpulse( impulse * ( -uvw1[2] / massPoint13->GetMass() ) );
					massPoint21->AddImpulse( impulse * ( uv2[0] / massPoint21->GetMass() ) );
					massPoint22->AddImpulse( impulse * ( uv2[1] / massPoint22->GetMass() ) );

					//dtkDouble3 impulse_normal = normalize( impulse );

					//double coe = 0.1;
					//dtkDouble3 vel11 = massPoint11->GetVel();
					//{
					//	dtkDouble3 temp = vel11 - impulse_normal * dot( vel11, impulse_normal );
					//	dtkDouble3 impulse11 = impulse * ( -uvw1[0] / massPoint11->GetMass() );
					//	if( length(temp) > 0.01 )
					//	{							
					//		if( length(impulse11) * coe > length( temp ) )
					//		{
					//			impulse11 = impulse11 - temp * massPoint11->GetMass();
					//		}
					//		else
					//		{
					//			impulse11 = impulse11 - normalize( temp ) * length(impulse11) * coe * massPoint11->GetMass();
					//		}
					//	}
					//	massPoint11->AddImpulse( impulse11 );
					//}

					//dtkDouble3 vel12 = massPoint12->GetVel();
					//{
					//	dtkDouble3 temp = vel12 - impulse_normal * dot( vel12, impulse_normal );
					//	dtkDouble3 impulse12 = impulse * ( -uvw1[1] / massPoint12->GetMass() );
					//	if( length(temp) > 0.01 )
					//	{	
					//		if( length(impulse12) * coe > length( temp ) )
					//		{
					//			impulse12 = impulse12 - temp * massPoint12->GetMass();
					//		}
					//		else
					//		{
					//			impulse12 = impulse12 - normalize( temp ) * length(impulse12) * coe * massPoint12->GetMass();
					//		}
					//	}
					//	massPoint12->AddImpulse( impulse12 );
					//}

					//dtkDouble3 vel13 = massPoint13->GetVel();
					//{
					//	dtkDouble3 temp = vel13 - impulse_normal * dot( vel13, impulse_normal );
					//	dtkDouble3 impulse13 = impulse * ( -uvw1[2] / massPoint13->GetMass() );
					//	if( length(temp) > 0.01 )
					//	{	
					//		if( length(impulse13) * coe > length( temp ) )
					//		{
					//			impulse13 = impulse13 - temp * massPoint13->GetMass();
					//		}
					//		else
					//		{
					//			impulse13 = impulse13 - normalize( temp ) * length(impulse13) * coe * massPoint13->GetMass();
					//		}
					//	}
					//	massPoint13->AddImpulse( impulse13 );
					//}

					//dtkDouble3 vel21 = massPoint21->GetVel();
					//{
					//	dtkDouble3 temp = vel21 - impulse_normal * dot( vel21, impulse_normal );
					//	dtkDouble3 impulse21 = impulse * ( uv2[0] / massPoint21->GetMass() );
					//	if( length(temp) > 0.01 )
					//	{	
					//		if( length(impulse21) * coe > length( temp ) )
					//		{
					//			impulse21 = impulse21 - temp * massPoint21->GetMass();
					//		}
					//		else
					//		{
					//			impulse21 = impulse21 - normalize( temp ) * length(impulse21) * coe * massPoint21->GetMass();
					//		}
					//	}
					//	massPoint21->AddImpulse( impulse21 );
					//}

					//dtkDouble3 vel22 = massPoint22->GetVel();
					//{
					//	dtkDouble3 temp = vel22 - impulse_normal * dot( vel22, impulse_normal );
					//	dtkDouble3 impulse22 = impulse * ( uv2[1] / massPoint22->GetMass() );
					//	if( length(temp) > 0.01 )
					//	{	
					//		if( length(impulse22) * coe > length( temp ) )
					//		{
					//			impulse22 = impulse22 - temp * massPoint22->GetMass();
					//		}
					//		else
					//		{
					//			impulse22 = impulse22 - normalize( temp ) * length(impulse22) * coe * massPoint22->GetMass();
					//		}
					//	}
					//	massPoint22->AddImpulse( impulse22 );
					//}
        
					dtkPhysMassPoint* massPoint2122 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] + 1);
					massPoint2122->AddImpulse( impulse * ( 0.5 / massPoint2122->GetMass() ) );

					if( pri_2->mDetailIDs[0] > 0 )
					{
						dtkPhysMassPoint* massPoint2021 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] - 1 );
						massPoint2021->AddImpulse( impulse * (uv2[0] * 0.5 / massPoint2021->GetMass() ) );
					}
					if( pri_2->mDetailIDs[1] + 1 < mMassSprings[pri_2->mMajorID]->GetNumberOfMassPoints() )
					{
						dtkPhysMassPoint* massPoint2223 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[1] + 1 );
						massPoint2223->AddImpulse( impulse * (uv2[1] * 0.5 / massPoint2223->GetMass() ) );
					}
					break;
				}
			case TRIANGLE_TRIANGLE:
				{
					GK::Vector3 normal;
					result->GetProperty( dtkIntersectTest::INTERSECT_NORMAL, normal );

					dtkDouble3 uvw1;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_1, uvw1 );

					dtkDouble3 uvw2;
					result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_2, uvw2 );

                    dtkPhysMassPoint* massPoint11 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint12 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[1] );
					dtkPhysMassPoint* massPoint13 = mMassSprings[pri_1->mMajorID]->GetMassPoint( pri_1->mDetailIDs[2] );
					dtkPhysMassPoint* massPoint21 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[0] );
					dtkPhysMassPoint* massPoint22 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[1] );
					dtkPhysMassPoint* massPoint23 = mMassSprings[pri_2->mMajorID]->GetMassPoint( pri_2->mDetailIDs[2] );

					dtkDouble3 impulse( normal[0], normal[1], normal[2] );
					impulse = impulse * ( stiffness * timeslice ); 

					impulse = impulse * 2.0 / ( pow( uvw1[0], 2 ) + pow( uvw1[1], 2 ) + pow( uvw1[2], 2 ) + pow( uvw2[0], 2 ) + pow( uvw2[1], 2 ) + pow( uvw2[2], 2 ) );

					massPoint11->AddImpulse( impulse * (-uvw1[0] / massPoint11->GetMass() ) );
					massPoint12->AddImpulse( impulse * (-uvw1[1] / massPoint12->GetMass() ) );
					massPoint13->AddImpulse( impulse * (-uvw1[2] / massPoint13->GetMass() ) );
					massPoint21->AddImpulse( impulse * ( uvw2[0] / massPoint21->GetMass() ) );
					massPoint22->AddImpulse( impulse * ( uvw2[1] / massPoint22->GetMass() ) );
					massPoint23->AddImpulse( impulse * ( uvw2[2] / massPoint23->GetMass() ) );
        
					break;
				}

			default:
				{
					break;
				}
			}
		}
    }
}

