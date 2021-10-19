#ifndef DTK_GRAPHICSKERNEL_H
#define DTK_GRAPHICSKERNEL_H

#include "dtkConfig.h"

#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include <CGAL/basic.h>
#include <CGAL/Polyhedron_3.h>

#include "dtkTx.h"
#include "dtkSign.h"

namespace dtk
{
    template < typename Float >
    class dtkInterval
    {
        public:
            /*
            typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
            typedef K::FT                   Float;
            */

            dtkInterval()
            {
                mLowerBound = true;
                mUpperBound = true;
                mLower = 0;
                mUpper = 0;
            }

            dtkInterval(const Float& l, const Float& u, bool lb = true, bool ub = true)
            {
                mLowerBound = lb;
                mUpperBound = ub;
                mLower = l;
                mUpper = u;
            }
        
            const Float& Lower() const { return mLower; }

            const Float& Upper() const { return mUpper; }

            bool IsLowerClosed() const { return mLowerBound; }

            bool IsUpperClosed() const { return mUpperBound; }

            const Float& operator[](const int& n) const
		    {
			    switch (n)
			    {
                    case 0: return mLower;
				    case 1: return mUpper;
				    default: // OUT_OF_RANGE
					    assert(false);
                        return mLower;
			    }
		    }

		    Float& operator[](const int& n)
		    {
			    return const_cast<Float&>(
                        static_cast<const dtkInterval&>(*this)[n]
                        );
		    }

            bool Contain(const Float& v) const
            {
                if( mLower > mUpper )
                    return false;

                if( v < mLower /*|| ( mLowerBound && v != mLower )*/ )
                {
                    return false;
                }
                else if( v > mUpper /*|| ( mUpperBound && v != mUpper )*/ )
                {
                    return false;
                }
                else
                    return true;
            }

            void Extend( const Float& extend )
            {
                if( extend < mLower )
                    mLower = extend;

                if( extend > mUpper )
                    mUpper = extend;
            }

        public:
            bool mLowerBound;
            bool mUpperBound;
            Float mLower;
            Float mUpper;
    };
	
    template < typename T >
    inline std::ostream& operator<<(std::ostream& stream, const dtkInterval<T>& interval)
    {
        if( interval.mLowerBound )
            stream << "[ ";
        else
            stream << "( ";

        stream << interval.mLower << ", " << interval.mUpper << " ";

        if( interval.mUpperBound )
            stream << "]";
        else
            stream << ")";
		
        return stream;
    }

    class dtkDiscreteOrientationPolytope
    {
        public:
            typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
            typedef K::FT                                               Float;
            typedef CGAL::Vector_3<K>                                   Vector3;
            const static Vector3 mPredefinedAxis[13];

        public:
            dtkDiscreteOrientationPolytope(size_t half_k)
            {
                mHalfK = half_k;
                mIntervals.resize( mHalfK );
            }

            ~dtkDiscreteOrientationPolytope()
            {
            }

            const Float& operator[](const int& n) const
		    {
                int major = n / 2;
                int minor = n - major * 2;

				assert( ((size_t)major) < mHalfK );
                    
                return mIntervals[major][minor];
		    }

		    Float& operator[](const int& n)
		    {
			    return const_cast< Float& >(
                        static_cast< const dtkDiscreteOrientationPolytope& >(*this)[n]
                        );
		    }

            size_t mHalfK;
            std::vector< dtkInterval< Float > > mIntervals;
    };
                
    inline std::ostream& operator<<(std::ostream& stream, const dtkDiscreteOrientationPolytope& kdop)
    {
        stream << "KDOP{ ";
        for( size_t i = 0; i < kdop.mHalfK; i++ )
        {
            stream << kdop.mIntervals[i] << " ";
        }
        stream << "}";
		
        return stream;
    }

    class dtkGraphicsKernel
    {
    public:
        
        typedef dtkGraphicsKernel Type;
        typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

        typedef K::FT                   Float;

        typedef CGAL::Object            Object;

        typedef dtkInterval<Float>      Interval;
        typedef dtkDiscreteOrientationPolytope  KDOP;

        //2D objects
        typedef CGAL::Point_2<K>        Point2;
        typedef CGAL::Vector_2<K>       Vector2;
        typedef CGAL::Segment_2<K>      Segment2;
        typedef CGAL::Triangle_2<K>     Triangle2;
        typedef CGAL::Circle_2<K>       Circle2;

        //3D objects
        typedef CGAL::Point_3<K>        Point3;
        typedef CGAL::Vector_3<K>       Vector3;
        typedef CGAL::Ray_3<K>          Ray3;
        typedef CGAL::Line_3<K>         Line3;
        typedef CGAL::Segment_3<K>      Segment3;
        typedef CGAL::Plane_3<K>        Plane3;
        typedef CGAL::Circle_3<K>       Circle3;
        typedef CGAL::Sphere_3<K>       Sphere3;
        typedef CGAL::Triangle_3<K>     Triangle3;
        typedef CGAL::Tetrahedron_3<K>  Tetrahedron3;
        typedef CGAL::Bbox_3            BBox3;

        typedef CGAL::Polyhedron_3<K>   Polyhedron3;


    //basic operations.
    public:
        static bool      IsZero  (const Type::Float      &v );
        static float     ToFloat (const  Type::Float     &v );
        static dtkFloat3 ToFloat3(const Type::Point3     &pt);
        static dtkFloat3 ToFloat3(const Type::Vector3    &pt);

        template <class T>
        static bool Assign(T &tgt, const Type::Object &obj)
        {
            return CGAL::assign(tgt, obj);
        }

        static Type::Float   SquaredDistance(const Type::Point3 &pt, const Type::Plane3 &plane);
        static Type::Float   SquaredDistance(const Type::Plane3 &plane, const Type::Point3 &pt);
        static Type::Float   SquaredDistance(const Type::Point3 &pt, const Type::Line3 &line);
        static Type::Float   SquaredDistance(const Type::Line3 &line, const Type::Point3 &pt);

        static Type::Float   SquaredArea(const Type::Point3 &p0, const Type::Point3 &p1, const Type::Point3 &p2);

        static Type::Vector3 CrossProduct(const Type::Vector3 &lhs, const Type::Vector3 &rhs);
        static Type::Float   DotProduct(  const Type::Vector3 &lhs, const Type::Vector3 &rhs);
        static Type::Float   DotProduct(const Type::Vector2 &lhs, const Type::Vector2 &rhs);

        static Type::Vector3 Normalize(const Type::Vector3 &v);
        static Type::Vector2 Normalize(const Type::Vector2 &v);

        static Type::Float   Sqrt(const Type::Float &val);

        static Type::Point3  Midpoint(const Type::Point3 &p0, const Type::Point3 &p1);

        static Type::Point3  Centroid(const Type::Point3 &p0, const Type::Point3 &p1, const Type::Point3 &p2);
        static Type::Point3  Centroid(const Type::Point3 &p0, const Type::Point3 &p1, const Type::Point3 &p2, const Type::Point3 &p3);
        static Type::Point3  Centroid(const Type::Tetrahedron3 &tetra);
        static Type::Point3  Centroid(const Type::Triangle3    &triangle);

        //Get Bounding box
        static Type::BBox3   BoundingBox(const Type::Triangle3 &triangle);
        static Type::BBox3   BoundingBox(const Type::Sphere3   &sphere);
        static Type::BBox3   BoundingBox(const Type::Tetrahedron3 &tetra);

        //Circumcenter
        static Type::Point3  Circumcenter(const Type::Triangle3 &triangle);
        static Type::Point3  Circumcenter(const Type::Tetrahedron3 &tetra);

        //Barycenter Weight
        static dtkDouble3    BarycentricWeight(const Type::Point3 &p0, const Type::Point3 &p1, const Type::Point3 &p2, const Type::Point3 &p3);
        static Type::Float   Length( const Type::Vector3 &vec );

    //predicates.
    public:

        //return dtkSign::NEGATIVE if in right-hand order;
        //       dtkSign::POSITIVE if in left-hand order;
        //       dtkSign::ZERO     if is degenerated.
        static dtkSign Orient2D(const Type::Point2 &a, const Type::Point2 &b, const Type::Point2 &p);

        static dtkSign Orient3D(const Type::Point3 &a, const Type::Point3 &b, const Type::Point3 &p);
        static dtkSign Orient3D(const Type::Point3 &a, const Type::Point3 &b, 
                                       const Type::Point3 &c, const Type::Point3 &p);
        static dtkSign Orient3D(const Type::Plane3 &plane, const Type::Point3 &p);

        static dtkSign InSphere(const Type::Point3 &a, const Type::Point3 &b, 
                                       const Type::Point3 &c, const Type::Point3 &d, 
                                       const Type::Point3 &p);

        static bool IsDegenerate(const Type::Segment3 &seg);
        static bool IsDegenerate(const Type::Tetrahedron3 &tetra);

    //dtkSign::POSITIVE, the point is in the bounded side
    //dtkSign::NEGATIVE, the point is in the unbounded side
    //dtkSign::ZERO,     the point is on the boundary
    public:
        static dtkSign BoundedSide(const Type::Tetrahedron3& tetra, const Type::Point3 &pt);
        static dtkSign BoundedSide(const Type::Sphere3 &sphere, const Type::Point3 &pt);

    public:
        static Type::Interval Merge(const Type::Interval& int_1, const Type::Interval& int_2);
        static Type::KDOP Merge(const Type::KDOP& kdop_1, const Type::KDOP& kdop_2);

        static void Merge(Type::Interval& int_r, const Type::Interval& int_1, const Type::Interval& int_2);
        static void Merge(Type::KDOP& kdop_r, const Type::KDOP& kdop_1, const Type::KDOP& kdop_2);
    };

    typedef dtkGraphicsKernel GK;
};

#endif //DTK_GRAPHICSKERNELCGAL_H
