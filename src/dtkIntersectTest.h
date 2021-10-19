#ifndef DTK_INTERSECTTEST_H
#define DTK_INTERSECTTEST_H

#include "dtkGraphicsKernel.h"
#include "dtkProperty.h"
#include "dtkIDTypes.h"

namespace dtk
{
	/**
	* @class <dtkIntersectTest> 
	* @brief A group of small intesection test functions
	* @author <>
	* @note Cannot instanlized.
	*/
    class dtkIntersectTest
    {
        public:
            enum IntersectResultType
            {
                INTERSECT_PRIMITIVE_1,  /**< represent the primitive1 for intersect testing */
                INTERSECT_PRIMITIVE_2,  /**< represent the primitive2 for intersect testing */
                INTERSECT_NORMAL, /**< the normal of interest. */ 
                INTERSECT_WEIGHT_1, /**< represent the intersect weight of primitive 1 */ 
                INTERSECT_WEIGHT_2, /**< represent the intersect weight of primitive 2 */ 
                INTERSECT_OBJECT /**< represent the intersect object from CGAL */ 
            };

            typedef dtkProperty<IntersectResultType> IntersectResult;
            
    public:
        // Test
        // 3D
        // --CGAL Wrapper
        static bool Test(GK::Point3 &pt, const GK::Line3 &line, const GK::Plane3 &plane);
        static bool Test(GK::Point3 &pt, const GK::Ray3 &ray, const GK::Plane3 &plane);
        static bool Test(GK::Point3 &pt, const GK::Segment3  &seg, const GK::Plane3 &plane);

        // --DTK Implement
        static bool Test(GK::Point3 &pt, const GK::Ray3 &ray, const GK::Triangle3 &tri);
        static bool Test(GK::Point3 &pt, const GK::Segment3 &seg, const GK::Triangle3 &tri);
        static bool Test(GK::Point3 &pt, const GK::Line3 &line, const GK::Triangle3 &tri);

        // Do Intersect
        // 2D
        // --CGAL Wrapper
        static bool DoIntersect(const GK::Segment2 &seg0, const GK::Segment2 &seg1);
        // --DTK Implement
        static bool DoIntersect(const GK::Triangle2 &tri, const GK::Circle2 &circle);

        // 3D
        // --CGAL Wrapper
        static bool DoIntersect(const GK::BBox3 &box1, const GK::BBox3 &box2);
        static bool DoIntersect(const GK::Line3 &line, const GK::Triangle3 &tri);
        static bool DoIntersect(const GK::Segment3 &seg, const GK::Triangle3 &tri);
        static bool DoIntersect(const GK::Segment3 &seg, const GK::Plane3 &plane);
        static bool DoIntersect(const GK::Ray3 &ray, const GK::Plane3 &plane);
        static bool DoIntersect(const GK::Interval& int_1, const GK::Interval& int_2);
        static bool DoIntersect(const GK::KDOP& kdop_1, const GK::KDOP& kdop_2);
        static bool DoIntersect(const GK::Segment3 &seg_1, const GK::Segment3 &seg_2);
        static bool DoIntersect(const GK::Triangle3 &triangle_1, const GK::Triangle3 &triangle_2);
        static bool DoIntersect(const GK::Triangle3 &triangle, const GK::Segment3 &segment); 

        // --DTK Implement
        static bool DoIntersect(const GK::Tetrahedron3 &tetra, const GK::Sphere3 &sphere);
        static bool DoIntersect(const GK::Triangle3 &triangle, const GK::Sphere3 &sphere);

        // Improved Intersect For Collision Detect Respones
        static bool DoIntersect(const GK::Segment3 &seg_1, const GK::Segment3 &seg_2, IntersectResult::Ptr& result );
        static bool DoIntersect(const GK::Triangle3 &triangle_1, const GK::Triangle3 &triangle_2, IntersectResult::Ptr& result );
        static bool DoIntersect(const GK::Triangle3 &triangle, const GK::Segment3 &segment, IntersectResult::Ptr& result ); 
        // Do Distance Intersect
        static bool DoDistanceIntersect(const GK::Segment3 &seg_1, const GK::Segment3 &seg_2, 
                double distance, IntersectResult::Ptr& result );
        static bool DoDistanceIntersect(const GK::Triangle3 &tri, const GK::Segment3 &seg, 
                double distance, IntersectResult::Ptr& result, dtkID invert = 0 );
        static bool DoDistanceIntersect(const GK::Triangle3 &tri_1, const GK::Triangle3 &tri_2, 
                double distance, IntersectResult::Ptr& result, dtkID invert = 0 );
	static bool DoDistanceIntersect(const GK::Triangle3 &tri, const GK::Sphere3 &sphere, 
			double distance, IntersectResult::Ptr& result );

        // deprecated
        static bool Test(const GK::BBox3 &box1, const GK::BBox3 &box2);
    private:
        static void UpdateMinMax(
                GK::Float &tmin, GK::Float &tmax,
                const GK::Float &val);

        static void ProjectToAxis(
                GK::Float &amin, GK::Float &amax, 
                const GK::Triangle2 &triangle,
                const GK::Vector2 &axis);

        static void ProjectToAxis(
                GK::Float &amin, GK::Float &amax,
                const GK::Circle2 &circle,
                const GK::Vector2 &axis);
        
        static void ProjectToAxis(
                GK::Float &amin, GK::Float &amax,
                const GK::Tetrahedron3 &tetra,
                const GK::Vector3      &axis);

        static void ProjectToAxis(
                GK::Float &amin, GK::Float &amax,
                const GK::Sphere3       &sphere,
                const GK::Vector3       &axis);

        static bool IsOverlap(
                GK::Float min0, GK::Float max0,
                GK::Float min1, GK::Float max1);

        dtkIntersectTest() {}
    };
}

#endif //DTK_INTERSECTTEST_H
