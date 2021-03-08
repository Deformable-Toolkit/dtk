#include "dtkCollisionDetectPrimitive.h"

#include <cstdarg>

using namespace std;

namespace dtk
{
	// the object of behind pts is vertex list of objects.
    dtkCollisionDetectPrimitive::dtkCollisionDetectPrimitive( Type type, dtkPoints::Ptr pts, ... )
    {
        mType = type;
        mPts = pts;
        mModified = true;
        mIntersected = false;

        va_list arguments;
        dtkID i;

        switch( mType )
        {
            case TRIANGLE:
                mNumberOfPoints = 3;
                break;
            case SEGMENT:
                mNumberOfPoints = 2;
                break;
			case SPHERE:
				mNumberOfPoints = 1;
				break;
            default:
                dtkAssert( false, NOT_IMPLEMENTED );
                break;
        }

        va_start( arguments, pts );
        for( i = 0; i < mNumberOfPoints; i++ )
            mIDs.push_back( va_arg( arguments, dtkID ) );
        va_end( arguments );

		mExtend = 0;

        mInvert = 0;

		mActive = true;

		Update();
    }
        
    void dtkCollisionDetectPrimitive::Update()
    {
		if( !mActive )
			return;

        mIntersected = false;

        switch( mType )
        {
            case TRIANGLE:
            {
                const GK::Point3& p0 = mPts->GetPoint( mIDs[0] );
                const GK::Point3& p1 = mPts->GetPoint( mIDs[1] );
                const GK::Point3& p2 = mPts->GetPoint( mIDs[2] );
                mObject = make_object( GK::Triangle3( p0, p1, p2 ) );
                mCentroid = GK::Centroid( p0, p1, p2 );
                break;
            }
            case SEGMENT:
            {
                const GK::Point3& p0 = mPts->GetPoint( mIDs[0] );
                const GK::Point3& p1 = mPts->GetPoint( mIDs[1] );
                mObject = make_object( GK::Segment3( p0, p1 ) );
                mCentroid = GK::Midpoint( p0, p1 );
                break;
            }
			case SPHERE:
			{
				const GK::Point3& p0 = mPts->GetPoint( mIDs[0] );
				if( mExtend > 0 )
					mObject = make_object( GK::Sphere3( p0, mExtend ) );
				else
					mObject = make_object( GK::Sphere3( p0, 0 ) );
				mCentroid = p0;
				break;
			}
            default:
                dtkAssert( false, NOT_IMPLEMENTED );
                break;
        }
    }
}

