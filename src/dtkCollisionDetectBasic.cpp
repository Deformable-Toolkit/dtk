#include "dtkCollisionDetectBasic.h"
#include "dtkCollisionDetectNodeKDOPS.h"

#include "dtkAssert.h"

#include <algorithm>

using namespace std;

#ifdef DTK_DEBUG
    #define DTK_COLLISIONDETECTBASIC_DEBUG
#endif //DTK_DEBUG

#ifdef DTK_COLLISIONDETECTBASIC_DEBUG
#include <iostream>
using namespace std;
#endif

namespace dtk
{
    bool dtkCollisionDetectBasic::DoIntersect( 
            dtkCollisionDetectPrimitive* pri_1, 
            dtkCollisionDetectPrimitive* pri_2, 
            IntersectResult::Ptr& result, 
            bool self, bool ignore_extend )
    {
        if( self )
        {
            for( dtkID i = 0; i < pri_1->mIDs.size(); i++ )
            {
                for( dtkID j = 0; j < pri_2->mIDs.size(); j++ )
                {
                    if( pri_1->mIDs[i] == pri_2->mIDs[j] )
                    {
                        return false;
                    }
                }
            }
        }

        bool exchanged = false;

		// according primitives' vertex to make geometry object.
        const GK::Object& obj_1 = pri_1->GetObject();
        const GK::Object& obj_2 = pri_2->GetObject();

        bool intersected = false;
        double distance = pri_1->GetExtend() + pri_2->GetExtend();
        assert( distance >= 0 );
		// ignore_extend represent considering the thickness of the two primitives.
        if( const GK::Triangle3* tri_1 = CGAL::object_cast< GK::Triangle3 >( &obj_1 ) )
        {
            if( const GK::Triangle3* tri_2 = CGAL::object_cast< GK::Triangle3 >( &obj_2 ) )
            {
                if( ignore_extend || distance == 0 )
                    intersected = dtkIntersectTest::DoIntersect( *tri_1, *tri_2, result );
                else
                    intersected = dtkIntersectTest::DoDistanceIntersect( *tri_1, *tri_2, distance, result, max( pri_1->mInvert, pri_2->mInvert ) );
            }
            else if( const GK::Segment3* seg_2 = CGAL::object_cast< GK::Segment3 >( &obj_2 ) )
            {
                if( ignore_extend || distance == 0 )
                    intersected = dtkIntersectTest::DoIntersect( *tri_1, *seg_2, result );
                else
                    intersected = dtkIntersectTest::DoDistanceIntersect( *tri_1, *seg_2, distance, result, pri_2->mInvert );
            }
			else if( const GK::Sphere3* sphere = CGAL::object_cast< GK::Sphere3 >( &obj_2 ) )
			{
				if( ignore_extend || distance == 0 )
					assert( false ); //intersected = dtkIntersectTest::DoIntersect( *tri_1, *sphere, result );
				else
					intersected = dtkIntersectTest::DoDistanceIntersect( *tri_1, *sphere, distance, result);
			}
        }
        else if( const GK::Segment3* seg_1 = CGAL::object_cast< GK::Segment3 >( &obj_1 ) )
        {
            if( const GK::Triangle3* tri_2 = CGAL::object_cast< GK::Triangle3 >( &obj_2 ) )
            {
                exchanged = true;
                if( ignore_extend || distance == 0 )
                    intersected = dtkIntersectTest::DoIntersect( *tri_2, *seg_1, result );
                else
                    intersected = dtkIntersectTest::DoDistanceIntersect( *tri_2, *seg_1, distance, result, pri_1->mInvert );
            }
            else if( const GK::Segment3* seg_2 = CGAL::object_cast< GK::Segment3 >( &obj_2 ) )
            {
                if( ignore_extend || distance == 0 )
                    intersected = dtkIntersectTest::DoIntersect( *seg_1, *seg_2, result );
                else
                    intersected = dtkIntersectTest::DoDistanceIntersect( *seg_1, *seg_2, distance, result );
            }
        }
        else
        {
            dtkAssert( false, NOT_IMPLEMENTED );
            return false;
        }

        if( intersected )
        {
            if( exchanged )
            {
                result->SetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_1 );
                result->SetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_2 );
            }
            else 
            {
                result->SetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1 );
                result->SetProperty( dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2 );
            }
            pri_1->SetIntersected( true );
            pri_2->SetIntersected( true );
        }

        return intersected;
    }

    bool dtkCollisionDetectBasic::DoIntersect( const dtkCollisionDetectNode* node_1, const dtkCollisionDetectNode* node_2 )
    {
        if( const dtkCollisionDetectNodeKDOPS* node_kdops_1 = 
                dynamic_cast< const dtkCollisionDetectNodeKDOPS* >( node_1 ) )
        {
            if( const dtkCollisionDetectNodeKDOPS* node_kdops_2 = 
                    dynamic_cast< const dtkCollisionDetectNodeKDOPS* >( node_2 ) )
            {
                return dtkIntersectTest::DoIntersect( node_kdops_1->GetKDOP(), node_kdops_2->GetKDOP() );
            }
            else
            {
                dtkAssert( false, NOT_IMPLEMENTED );
                return false;
            }
        }
        else
        {
            dtkAssert( false, NOT_IMPLEMENTED );
            return false;
        }
    }
}

