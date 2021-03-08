#include "dtkPhysMassSpringThreadCollisionResponse.h"
#include "dtkCollisionDetectPrimitive.h"

#include <iostream>
using namespace std;

namespace dtk
{
	dtkPhysMassSpringThreadCollisionResponse::dtkPhysMassSpringThreadCollisionResponse( dtkPhysMassSpringCollisionResponse::Ptr priorResponse )
	{
        mPriorResponse = priorResponse;
	}

	dtkPhysMassSpringThreadCollisionResponse::~dtkPhysMassSpringThreadCollisionResponse()
	{

	}

    pair< dtkDouble3, dtkDouble3 > dtkPhysMassSpringThreadCollisionResponse::GetVirtualPair( dtkID threadID, dtkID id )
    {
        PiercedResult::Ptr result = mPiercedResults[threadID][id];
		dtkCollisionDetectPrimitive* pri_1;
        dtkCollisionDetectPrimitive* pri_2;

		result->GetProperty( PIERCED_PRIMITIVE_1, pri_1 );
        result->GetProperty( PIERCED_PRIMITIVE_2, pri_2 );

		dtkPhysMassSpring::Ptr targetMS = mPriorResponse->GetMassSpring( pri_1->mMajorID );
        dtkPhysMassSpring::Ptr threadMS = mPriorResponse->GetMassSpring( pri_2->mMajorID );

		dtkPhysMassPoint* massPoint11 = targetMS->GetMassPoint( pri_1->mDetailIDs[0] );
		dtkPhysMassPoint* massPoint12 = targetMS->GetMassPoint( pri_1->mDetailIDs[1] );
		dtkPhysMassPoint* massPoint13 = targetMS->GetMassPoint( pri_1->mDetailIDs[2] );

        int segmentID;
        result->GetProperty( PIERCED_SEGMENTID, segmentID );

        dtkPhysMassPoint* massPoint21 = threadMS->GetMassPoint( segmentID * 2 );
		dtkPhysMassPoint* massPoint22 = threadMS->GetMassPoint( segmentID * 2 + 2 );
        
		dtkDouble3 p11 = massPoint11->GetPosition();
		dtkDouble3 p12 = massPoint12->GetPosition();
		dtkDouble3 p13 = massPoint13->GetPosition();
        dtkDouble3 p21 = massPoint21->GetPosition();
        dtkDouble3 p22 = massPoint22->GetPosition();

		dtkDouble3 uvw;
		result->GetProperty( PIERCED_WEIGHT_1, uvw );

        dtkDouble2 uv;
        result->GetProperty( PIERCED_WEIGHT_2, uv );

		dtkDouble3 pointOnTriangle = p11 * uvw[0] + p12 * uvw[1] + p13 * uvw[2]; 
        dtkDouble3 pointOnSegment = p21 * uv[0] + p22 * uv[1]; 
        return pair< dtkDouble3, dtkDouble3 >(pointOnTriangle,pointOnSegment);
    }

    void dtkPhysMassSpringThreadCollisionResponse::Update( double timeslice, vector<dtkIntersectTest::IntersectResult::Ptr>& internalPiercingResults )
    {
        std::vector< dtkIntersectTest::IntersectResult::Ptr >& surfacePiercingResults = mPriorResponse->GetPiercingResults();

        for( dtkID i = 0; i < surfacePiercingResults.size(); i++ )
        {
			cout<<"try surface piercing results"<<endl;

            dtkIntersectTest::IntersectResult::Ptr result = surfacePiercingResults[i];
            dtkCollisionDetectPrimitive* pri_1;
            dtkCollisionDetectPrimitive* pri_2;

            result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1 );
            result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2 );

            PiercedResult::Ptr newResult = PiercedResult::New();

            newResult->SetProperty( PIERCED_PRIMITIVE_1, pri_1 );
            newResult->SetProperty( PIERCED_PRIMITIVE_2, pri_2 );

			bool already_pierced = false;
			for( dtkID j = 0; j < mPiercedResults[pri_2->mMajorID].size(); j++ )
			{
				dtkCollisionDetectPrimitive* f_pri_1;
				mPiercedResults[pri_2->mMajorID][j]->GetProperty( PIERCED_PRIMITIVE_1, f_pri_1 );

				if( f_pri_1->mMinorID == pri_1->mMinorID )
				{
					already_pierced = true;
					break;
				}
			}
			if( already_pierced )
				continue;

			cout<<"add surface piercing results"<<endl;

            dtkDouble3 uvw1;
            result->GetProperty( dtkIntersectTest::INTERSECT_WEIGHT_1, uvw1 );
            newResult->SetProperty( PIERCED_WEIGHT_1, uvw1 );

            dtkDouble2 uv2;
            uv2[0] = 1;
            uv2[1] = 0;
            newResult->SetProperty( PIERCED_WEIGHT_2, uv2 );

            int segmentID = 0;
            newResult->SetProperty( PIERCED_SEGMENTID, segmentID );

            bool valid = true;
            newResult->SetProperty( PIERCED_VALID, valid );

            bool surface = true;
            newResult->SetProperty( PIERCED_SURFACE, surface );

            //mAvoidIntervals[pri_2->mMajorID].push_back( dtkInterval<int> ( -1, -1 ) );
            mThreadHeadInsides[pri_2->mMajorID] = !mThreadHeadInsides[pri_2->mMajorID];
            mPiercedResults[pri_2->mMajorID].push_back( newResult );

			mNumerOfSurfacePiercedResults[pri_2->mMajorID] = mNumerOfSurfacePiercedResults[pri_2->mMajorID] + 1;

			break;
        }

        surfacePiercingResults.clear();
    
        for( dtkID i = 0; i < internalPiercingResults.size(); i++ )
        {
            dtkIntersectTest::IntersectResult::Ptr result = internalPiercingResults[i];
            dtkCollisionDetectPrimitive* pri_1;
            dtkCollisionDetectPrimitive* pri_2;

            result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1 );
            result->GetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2 );

			bool already_pierced = false;
			for( dtkID j = 0; j < mPiercedResults[pri_2->mMajorID].size(); j++ )
			{
				dtkCollisionDetectPrimitive* f_pri_1;
				mPiercedResults[pri_2->mMajorID][j]->GetProperty( PIERCED_PRIMITIVE_1, f_pri_1 );

				if( f_pri_1->mMinorID == pri_1->mMinorID )
				{
					already_pierced = true;
					break;
				}
			}
			if( already_pierced )
				continue;

            //cout<<"add internal piercing results"<<endl;

            PiercedResult::Ptr newResult = PiercedResult::New();

            newResult->SetProperty( PIERCED_PRIMITIVE_1, pri_1 );
            newResult->SetProperty( PIERCED_PRIMITIVE_2, pri_2 );

            GK::Object object;
            result->GetProperty( dtkIntersectTest::INTERSECT_OBJECT, object );

			dtkDouble3 p4;
            if( const GK::Point3* ipoint = CGAL::object_cast< GK::Point3 >( &object ) )
			{
				p4 = dtkDouble3( ipoint->x(), ipoint->y(), ipoint->z() );
			}
			else if( const GK::Segment3* isegment = CGAL::object_cast< GK::Segment3 >( &object ) )
			{
				continue;
			}
			else
				assert( false );

            dtkPhysMassSpring::Ptr targetMS = mPriorResponse->GetMassSpring( pri_1->mMajorID );
            dtkPhysMassPoint* massPoint11 = targetMS->GetMassPoint( pri_1->mDetailIDs[0] );
		    dtkPhysMassPoint* massPoint12 = targetMS->GetMassPoint( pri_1->mDetailIDs[1] );
			dtkPhysMassPoint* massPoint13 = targetMS->GetMassPoint( pri_1->mDetailIDs[2] );

            dtkDouble3 p1 = massPoint11->GetPosition();
            dtkDouble3 p2 = massPoint12->GetPosition();
            dtkDouble3 p3 = massPoint13->GetPosition();


            dtkDouble3 uvw1 = barycentricWeight( p4, p1, p2, p3 );

            newResult->SetProperty( PIERCED_WEIGHT_1, uvw1 );

            dtkDouble2 uv2;
            uv2[0] = 1;
            uv2[1] = 0;
            newResult->SetProperty( PIERCED_WEIGHT_2, uv2 );

            int segmentID = 0;
            newResult->SetProperty( PIERCED_SEGMENTID, segmentID );

            bool valid = true;
            newResult->SetProperty( PIERCED_VALID, valid );

            bool surface = false;
            newResult->SetProperty( PIERCED_SURFACE, surface );

            //mInternalPiercingTriangleIDs[pri_2->mMajorID] = pri_1->mMinorID;

            mPiercedResults[pri_2->mMajorID].push_back( newResult );
        }

        for( std::map< dtkID, std::vector< PiercedResult::Ptr > >::iterator itr = mPiercedResults.begin();
                itr != mPiercedResults.end(); itr++ )
        {
            dtkID threadID = itr->first;
        
            for( dtkID i = 0; i < mPiercedResults[threadID].size(); i++ )
            {
                PiercedResult::Ptr result = mPiercedResults[threadID][i];

                bool valid;
                result->GetProperty( PIERCED_VALID, valid );

                if( !valid )
                    continue;

                dtkCollisionDetectPrimitive* pri_1;
                dtkCollisionDetectPrimitive* pri_2;

                result->GetProperty( PIERCED_PRIMITIVE_1, pri_1 );
                result->GetProperty( PIERCED_PRIMITIVE_2, pri_2 );

                dtkDouble3 uvw1;
                result->GetProperty( PIERCED_WEIGHT_1, uvw1 );
            
                dtkPhysMassSpring::Ptr targetMS = mPriorResponse->GetMassSpring( pri_1->mMajorID );
                dtkPhysMassSpring::Ptr threadMS = mPriorResponse->GetMassSpring( pri_2->mMajorID );

                dtkPhysMassPoint* massPoint11 = targetMS->GetMassPoint( pri_1->mDetailIDs[0] );
		        dtkPhysMassPoint* massPoint12 = targetMS->GetMassPoint( pri_1->mDetailIDs[1] );
			    dtkPhysMassPoint* massPoint13 = targetMS->GetMassPoint( pri_1->mDetailIDs[2] );

                int segmentID;
                result->GetProperty( PIERCED_SEGMENTID, segmentID );
			    dtkPhysMassPoint* massPoint21 = threadMS->GetMassPoint( segmentID * 2 );
			    dtkPhysMassPoint* massPoint22 = threadMS->GetMassPoint( segmentID * 2 + 2 );
        
                dtkDouble3 p11 = massPoint11->GetPosition();
                dtkDouble3 p12 = massPoint12->GetPosition();
                dtkDouble3 p13 = massPoint13->GetPosition();
                dtkDouble3 p21 = massPoint21->GetPosition();
                dtkDouble3 p22 = massPoint22->GetPosition();

                dtkDouble2 uv;
                result->GetProperty( PIERCED_WEIGHT_2, uv );
                dtkDouble3 pointOnSegment = p21 * uv[0] + p22 * uv[1]; 
                dtkDouble3 pointOnTriangle = p11 * uvw1[0] + p12 * uvw1[1] + p13 * uvw1[2];

                dtkDouble3 impulseVec( pointOnTriangle - pointOnSegment );
 
                dtkDouble3 p2221_normal = normalize(p22 - p21);
                double cosangle = dot( p2221_normal, impulseVec );

                dtkDouble3 proj = p2221_normal * cosangle;
                double percent = length(proj) / length( p22 - p21);
                if( cosangle < 0 )
                {
                    percent = -percent;
                }
                double pointOnSegmentID = segmentID + uv[1] + percent;
                segmentID = (int)floor( pointOnSegmentID );

                if( segmentID + 1 > (int)mThreads[pri_2->mMajorID]->GetNumberOfSegments() || segmentID < 0)
                {
                   // cout<<"set PIERCED_VALID false"<<endl;
                    result->SetProperty( PIERCED_VALID, false );
                    continue;
                }

                dtkDouble3 impulse;
                result->SetProperty( PIERCED_SEGMENTID, segmentID );

                impulse = impulseVec * ( 200000 * timeslice * 5.0 );
                massPoint11->AddForce( impulse * ( -uvw1[0] / massPoint11->GetMass() ) );
			    massPoint12->AddForce( impulse * ( -uvw1[1] / massPoint12->GetMass() ) );
			    massPoint13->AddForce( impulse * ( -uvw1[2] / massPoint13->GetMass() ) );

                massPoint21->AddForce( impulse * ( uv[0] / massPoint21->GetMass() * 4.0 ) );
                massPoint22->AddForce( impulse * ( uv[1] / massPoint22->GetMass() * 4.0 ) );

                uv[1] = pointOnSegmentID - (int)pointOnSegmentID; 
                uv[0] = 1.0 - uv[1];

                result->SetProperty( PIERCED_WEIGHT_2, uv );
            }

			int numOfPiercedResults = mPiercedResults[threadID].size();
            //delete
            for( int i = mPiercedResults[threadID].size() - 1; i >= 0; i-- )
            {
                PiercedResult::Ptr result = mPiercedResults[threadID][i];
                bool valid;
                result->GetProperty( PIERCED_VALID, valid );
                if( !valid )
                {
                    cout<<"delete pierced result: "<<i<<endl;
					bool surface;
					result->GetProperty( PIERCED_SURFACE, surface );
					if( surface )
					{
						mNumerOfSurfacePiercedResults[threadID] = mNumerOfSurfacePiercedResults[threadID] - 1;
					}
                    mPiercedResults[threadID].erase( mPiercedResults[threadID].begin() + i );
                }
            }

			mAvoidIntervals[threadID].clear();
			for( dtkID i = 0; i < mPiercedResults[threadID].size(); i++ )
			{
				PiercedResult::Ptr result = mPiercedResults[threadID][i];
				bool surface;
				result->GetProperty( PIERCED_SURFACE, surface );
				if( surface )
				{
					int segmentID;
					result->GetProperty( PIERCED_SEGMENTID, segmentID );
					//cout<<"avoid intervals: "<<segmentID - 2<<" "<<segmentID + 2<<endl;
					mAvoidIntervals[threadID].push_back( dtkInterval<int>( segmentID - 2, segmentID + 2 ) );
				}
			}
			
			if( mPiercedResults[threadID].size() == 0 )
				mThreadHeadInsides[threadID] = false;

            mInternalIntervals[threadID].clear();

            std::vector< dtkInterval<int> >& intervals = mAvoidIntervals[threadID];
            int i = intervals.size() - 1;

            int numSegments = mThreads[threadID]->GetNumberOfSegments();

            int lower, upper;
            if( mThreadHeadInsides[threadID] )
            {
                lower = 0;
                upper = intervals[i][1];
                if( upper + 1 > numSegments )
                    upper = numSegments - 1;
                mInternalIntervals[threadID].push_back(dtkInterval<int>( lower, upper ));
                i--;
            }

            for( ; i > 0; i -= 2 )
            {
                lower = intervals[i][0];
                upper = intervals[i - 1][1];
                if( lower < 0 )
                    lower = 0;
                if( upper + 1 > numSegments )
                    upper = numSegments - 1;
                mInternalIntervals[threadID].push_back(dtkInterval<int>( lower, upper ));
            }

            if( i == 0 )
            {
                lower = intervals[i][0];
                upper = numSegments - 1;
                if( lower < 0 )
                    lower = 0;
                mInternalIntervals[threadID].push_back(dtkInterval<int>( lower, upper ));
            }
        }
    }

	void dtkPhysMassSpringThreadCollisionResponse::PostProcess(double range)
	{
		for( std::map< dtkID, std::vector< PiercedResult::Ptr > >::iterator itr = mPiercedResults.begin();
			itr != mPiercedResults.end(); itr++ )
		{
			dtkID threadID = itr->first;

			for( dtkID i = 0; i < mPiercedResults[threadID].size(); i++ )
			{
				PiercedResult::Ptr result = mPiercedResults[threadID][i];

				bool valid;
				result->GetProperty( PIERCED_VALID, valid );

				if( !valid )
					continue;

				dtkCollisionDetectPrimitive* pri_1;
				dtkCollisionDetectPrimitive* pri_2;

				result->GetProperty( PIERCED_PRIMITIVE_1, pri_1 );
				result->GetProperty( PIERCED_PRIMITIVE_2, pri_2 );

				dtkDouble3 uvw;
				result->GetProperty( PIERCED_WEIGHT_1, uvw );

				dtkPhysMassSpring::Ptr targetMS = mPriorResponse->GetMassSpring( pri_1->mMajorID );
				dtkPhysMassSpring::Ptr threadMS = mPriorResponse->GetMassSpring( pri_2->mMajorID );

				dtkPhysMassPoint* massPoint11 = targetMS->GetMassPoint( pri_1->mDetailIDs[0] );
				dtkPhysMassPoint* massPoint12 = targetMS->GetMassPoint( pri_1->mDetailIDs[1] );
				dtkPhysMassPoint* massPoint13 = targetMS->GetMassPoint( pri_1->mDetailIDs[2] );
				
				int segmentID;
				result->GetProperty( PIERCED_SEGMENTID, segmentID );
				dtkPhysMassPoint* massPoint21 = threadMS->GetMassPoint( segmentID * 2 );
				dtkPhysMassPoint* massPoint2_mid = threadMS->GetMassPoint( segmentID * 2 + 1 );
				dtkPhysMassPoint* massPoint22 = threadMS->GetMassPoint( segmentID * 2 + 2 );

				dtkDouble3 p11 = massPoint11->GetPosition();
				dtkDouble3 p12 = massPoint12->GetPosition();
				dtkDouble3 p13 = massPoint13->GetPosition();
				dtkDouble3 p21 = massPoint21->GetPosition();
				dtkDouble3 p22 = massPoint22->GetPosition();
				dtkDouble3 p2_mid = massPoint2_mid->GetPosition();

				dtkDouble2 uv;
				result->GetProperty( PIERCED_WEIGHT_2, uv );
				dtkDouble3 pointOnSegment = p21 * uv[0] + p22 * uv[1]; 
				dtkDouble3 pointOnTriangle = p11 * uvw[0] + p12 * uvw[1] + p13 * uvw[2];

				dtkDouble3 impulseVec( pointOnTriangle - pointOnSegment );

				if( length( impulseVec ) > range)
				{
					massPoint21->SetPosition( p21 + impulseVec * 0.2 );
					massPoint22->SetPosition( p22 + impulseVec * 0.2);
					massPoint2_mid->SetPosition( p2_mid + impulseVec * 0.2 );
					/*massPoint11->SetPosition( p11 - impulseVec * 0.1 );
					massPoint12->SetPosition( p12 - impulseVec * 0.1 );
					massPoint13->SetPosition( p13 - impulseVec * 0.1 );*/
				}	
			}
		}			
	}
}

