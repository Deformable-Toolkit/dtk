#include "dtkGraphicsKernel.h"

#include "dtkSign.h"
#include "dtkAssert.h"
#include "dtkIDTypes.h"

#include <CGAL/number_utils.h>
#include <algorithm>

namespace dtk
{   
    /**
     * @brief K-Dops axises
     */
    const GK::Vector3 preAxis_0( 1, 0, 0 );
    const GK::Vector3 preAxis_1( 0, 1, 0 );
    const GK::Vector3 preAxis_2( 0, 0, 1 ); /**< 6-dops */
    const GK::Vector3 preAxis_3( 1, 1, 1 );
    const GK::Vector3 preAxis_4( -1, 1, 1 );
    const GK::Vector3 preAxis_5( 1, -1, 1 );
    const GK::Vector3 preAxis_6( 1, 1, -1 ); /**< 14-dops */
    const GK::Vector3 preAxis_7( 0, 1, 1 );
    const GK::Vector3 preAxis_8( 1, 0, 1 ); /**< 18-dops */
    const GK::Vector3 preAxis_9( 1, 1, 0 );
    const GK::Vector3 preAxis_10( 0, 1, -1 );
    const GK::Vector3 preAxis_11( -1, 0, 1 );
    const GK::Vector3 preAxis_12( 1, -1, 0 ); /**< 26-dops */
    

    const GK::Vector3 GK::KDOP::mPredefinedAxis[13] = {
        preAxis_0, preAxis_1, preAxis_2, 
        preAxis_3, preAxis_4, preAxis_5, preAxis_6,
        preAxis_7, preAxis_8, preAxis_9, preAxis_10, preAxis_11, preAxis_12 };

		    bool
    dtkGraphicsKernel::IsZero(
            const dtkGraphicsKernel::Float &v)
    {
        return CGAL::is_zero(v);
    }

    float
    dtkGraphicsKernel::ToFloat(
            const dtkGraphicsKernel::Float &v)
    {
        return (float)CGAL::to_double(v);
    }

    dtkFloat3
    dtkGraphicsKernel::ToFloat3(
            const dtkGraphicsKernel::Point3 &pt)
    {
        return dtkFloat3(ToFloat(pt.x()),
                ToFloat(pt.y()), ToFloat(pt.z()));
    }

    dtkFloat3
    dtkGraphicsKernel::ToFloat3(
            const dtkGraphicsKernel::Vector3 &v)
    {
        return dtkFloat3(
                (float)CGAL::to_double(v.x()),
                (float)CGAL::to_double(v.y()),
                (float)CGAL::to_double(v.z()));
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::SquaredDistance(
            const dtkGraphicsKernel::Point3 &pt,
            const dtkGraphicsKernel::Plane3 &plane)
    {
        return CGAL::squared_distance(pt, plane);
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::SquaredDistance(
            const dtkGraphicsKernel::Plane3 &plane,
            const dtkGraphicsKernel::Point3 &pt)
    {
        return SquaredDistance(pt, plane);
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::SquaredDistance(
            const GK::Point3 &pt,
            const GK::Line3 &line)
    {
        return CGAL::squared_distance(pt, line);
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::SquaredDistance(
            const GK::Line3 &line,
            const GK::Point3 &pt)
    {
        return SquaredDistance(pt, line);
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::SquaredArea(
            const dtkGraphicsKernel::Point3 &p0,
            const dtkGraphicsKernel::Point3 &p1,
            const dtkGraphicsKernel::Point3 &p2)
    {
        return dtkGraphicsKernel::Triangle3(p0, p1, p2).squared_area();
    }

    dtkGraphicsKernel::Vector3
    dtkGraphicsKernel::CrossProduct(
            const dtkGraphicsKernel::Vector3 &lhs,
            const dtkGraphicsKernel::Vector3 &rhs)
    {
        return CGAL::cross_product(lhs, rhs);
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::DotProduct(
            const dtkGraphicsKernel::Vector3 &lhs,
            const dtkGraphicsKernel::Vector3 &rhs)
    {
        return lhs * rhs;
    }


    dtkGraphicsKernel::Float
    dtkGraphicsKernel::DotProduct(
            const dtkGraphicsKernel::Vector2 &lhs,
            const dtkGraphicsKernel::Vector2 &rhs)
    {
        return lhs * rhs;
    }

    dtkGraphicsKernel::Float
    dtkGraphicsKernel::Sqrt(const dtkGraphicsKernel::Float &v)
    {
        return CGAL::sqrt(v);
    }

    dtkGraphicsKernel::Vector3
    dtkGraphicsKernel::Normalize(
            const dtkGraphicsKernel::Vector3 &v)
    {
        dtkGraphicsKernel::Float len = Sqrt(v.squared_length());
        assert(len != 0);

        return dtkGraphicsKernel::Vector3(
                v.x() / len, v.y() / len, v.z() / len);
    }

    dtkGraphicsKernel::Vector2
    dtkGraphicsKernel::Normalize(
            const Vector2 &v)
    {
        Float len = Sqrt(v.squared_length());
        return Vector2(v.x() / len, v.y() / len);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Midpoint(
            const dtkGraphicsKernel::Point3 &p0,
            const dtkGraphicsKernel::Point3 &p1)
    {
        return CGAL::midpoint(p0, p1);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Centroid(
            const dtkGraphicsKernel::Point3 &p0,
            const dtkGraphicsKernel::Point3 &p1,
            const dtkGraphicsKernel::Point3 &p2)
    {
        return CGAL::centroid(p0, p1, p2);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Centroid(
            const dtkGraphicsKernel::Point3 &p0,
            const dtkGraphicsKernel::Point3 &p1,
            const dtkGraphicsKernel::Point3 &p2,
            const dtkGraphicsKernel::Point3 &p3)
    {
        return CGAL::centroid(p0, p1, p2, p3);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Centroid(
            const dtkGraphicsKernel::Triangle3 &triangle)
    {
        return CGAL::centroid(triangle);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Centroid(
            const dtkGraphicsKernel::Tetrahedron3 &tetra)
    {
        return CGAL::centroid(tetra);
    }
    
    
	// Any point  located on this triangle may then be written as a weighted sum of these three vertices.
	// detail : http://en.wikipedia.org/wiki/Barycentric_coordinate_system_(mathematics).
	// at first: p0 is projected to the bottom triangle, then, compute the weight.

    dtkDouble3    
    dtkGraphicsKernel::BarycentricWeight(
            const Type::Point3 &p0, 
            const Type::Point3 &p1, 
            const Type::Point3 &p2, 
            const Type::Point3 &p3)
    {
        dtkDouble3 uvw;

        GK::Vector3 v03 = p0 - p3;
        GK::Vector3 v13 = p1 - p3;
        GK::Vector3 v23 = p2 - p3;

        double dot1313 = DotProduct( v13, v13 );
        double dot1323 = DotProduct( v13, v23 );
        double dot2323 = DotProduct( v23, v23 );
        double dot1303 = DotProduct( v13, v03 );
        double dot2303 = DotProduct( v23, v03 );

        assert( dot1313 > dtkDoubleEpslon );
        assert( dot2323 > dtkDoubleEpslon );

        /*
        if( dot1323 < dtkDoubleEpslon )
        {
            uvw[0] = dot1303 / dot1313;
            uvw[1] = dot2303 / dot2323;
        }
        else
        {
        */
        double det = dot1313 * dot2323 - dot1323 * dot1323;
        double det0 = dot1303 * dot2323 - dot1323 * dot2303;
        double det1 = dot1313 * dot2303 - dot1303 * dot1323;
        uvw[0] = det0 / det;
        uvw[1] = det1 / det;
            /*
        }
        */
        uvw[2] = 1.0 - uvw[0] - uvw[1];

        return uvw;
    }

    dtkGraphicsKernel::BBox3
    dtkGraphicsKernel::BoundingBox(
            const dtkGraphicsKernel::Triangle3 &triangle)
    {
        return triangle.bbox();
    }

    dtkGraphicsKernel::BBox3
    dtkGraphicsKernel::BoundingBox(
            const dtkGraphicsKernel::Sphere3 &sphere)
    {
        return sphere.bbox();
    }

    dtkGraphicsKernel::BBox3
    dtkGraphicsKernel::BoundingBox(
            const dtkGraphicsKernel::Tetrahedron3 &tetra)
    {
        return tetra.bbox();
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Circumcenter(
            const Type::Triangle3 &triangle)
    {
        return CGAL::circumcenter(triangle);
    }

    dtkGraphicsKernel::Point3
    dtkGraphicsKernel::Circumcenter(
            const Type::Tetrahedron3 &tetra)
    {
        return CGAL::circumcenter(tetra);
    }

    dtkSign
    dtkGraphicsKernel::Orient2D(
            const Point2 &a,
            const Point2 &b,
            const Point2 &p)
    {
        CGAL::Orientation orient = CGAL::orientation(a, b, p);
        if (orient == CGAL::RIGHT_TURN) return dtkSign::POSITIVE;
        if (orient == CGAL::ZERO) return dtkSign::ZERO;
        return dtkSign::NEGATIVE;
    }

    dtkSign
    dtkGraphicsKernel::Orient3D(
            const Point3 &a,
            const Point3 &b,
            const Point3 &p)
    {
        CGAL::Orientation orient = CGAL::coplanar_orientation(a, b, p);
        if (orient == CGAL::COLLINEAR)   return dtkSign::ZERO;
        if (orient == CGAL::POSITIVE) return dtkSign::POSITIVE;
        return dtkSign::NEGATIVE;
    }

    dtkSign
    dtkGraphicsKernel::Orient3D(
            const Point3 &a, 
            const Point3 &b, 
            const Point3 &c, 
            const Point3 &p)
    {
        CGAL::Orientation orient = CGAL::orientation(a, b, c, p);
        if (orient == CGAL::POSITIVE)
            return dtkSign::POSITIVE;
        if (orient == CGAL::COPLANAR)
            return dtkSign::ZERO;
        return dtkSign::NEGATIVE;
    }

    dtkSign
    dtkGraphicsKernel::Orient3D(
            const Plane3 &plane,
            const Point3 &p)
    {
        CGAL::Oriented_side side = plane.oriented_side(p);
        if (side == CGAL::ON_POSITIVE_SIDE) return dtkSign::POSITIVE;
        if (side == CGAL::ON_NEGATIVE_SIDE) return dtkSign::NEGATIVE;
        return dtkSign::ZERO;
    }

    dtkSign
    dtkGraphicsKernel::InSphere(
            const Point3 &a, 
            const Point3 &b, 
            const Point3 &c, 
            const Point3 &d, 
            const Point3 &p)
    {
        CGAL::Bounded_side side;
        side = CGAL::side_of_bounded_sphere(a, b, c, d, p);

        if (side == CGAL::ON_BOUNDED_SIDE) return dtkSign::POSITIVE;
        if (side == CGAL::ON_BOUNDARY)     return dtkSign::ZERO;
        return dtkSign::NEGATIVE;
    }

    bool
    dtkGraphicsKernel::IsDegenerate(
            const Segment3 &seg)
    {
        return seg.is_degenerate();
    }

    bool
    dtkGraphicsKernel::IsDegenerate(
            const Tetrahedron3 &tetra)
    {
        return tetra.is_degenerate();
    }

    dtkSign
    dtkGraphicsKernel::BoundedSide(
            const Tetrahedron3 &tetra,
            const Point3 &pt)
    {
        CGAL::Bounded_side side = tetra.bounded_side(pt);

        if (side == CGAL::ON_UNBOUNDED_SIDE) return dtkSign::NEGATIVE;
        if (side == CGAL::ON_BOUNDED_SIDE) return dtkSign::POSITIVE;
        return dtkSign::ZERO;
    }

    dtkSign
    dtkGraphicsKernel::BoundedSide(
            const Sphere3 &sphere,
            const Point3 &pt)
    {
        CGAL::Bounded_side side = sphere.bounded_side(pt);

        if (side == CGAL::ON_UNBOUNDED_SIDE) return dtkSign::NEGATIVE;
        if (side == CGAL::ON_BOUNDED_SIDE) return dtkSign::POSITIVE;
        return dtkSign::ZERO;
    }
        
    GK::Interval GK::Merge(const GK::Interval& int_1, const GK::Interval& int_2)
    {
        GK::Interval merge_int;
        
        merge_int[0] = std::min( int_1[0], int_2[0] );
        merge_int[1] = std::max( int_1[1], int_2[1] );

        return merge_int;
    }
    
    GK::KDOP GK::Merge(const GK::KDOP& kdop_1, const GK::KDOP& kdop_2)
    {
        assert( kdop_1.mHalfK == kdop_2.mHalfK );

        GK::KDOP merge_kdop( kdop_1.mHalfK );
        for( dtkID i = 0; i < kdop_1.mHalfK; i++ )
        {
            merge_kdop.mIntervals[i] = GK::Merge( kdop_1.mIntervals[i], kdop_2.mIntervals[i] );
        }

        return merge_kdop;
    }

    void GK::Merge(GK::Interval& int_r, const GK::Interval& int_1, const GK::Interval& int_2)
    {
        int_r[0] = std::min( int_1[0], int_2[0] );
        int_r[1] = std::max( int_1[1], int_2[1] );
    }
    
    void GK::Merge(GK::KDOP& kdop_r, const GK::KDOP& kdop_1, const GK::KDOP& kdop_2)
    {
        for( dtkID i = 0; i < kdop_1.mHalfK; i++ )
        {
            GK::Merge( kdop_r.mIntervals[i], kdop_1.mIntervals[i], kdop_2.mIntervals[i] );
        }
    }
        
    GK::Float GK::Length( const Type::Vector3 &vec )
    {
        return GK::Sqrt( GK::DotProduct( vec, vec ) );
    }
}
