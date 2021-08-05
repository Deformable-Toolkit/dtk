#include "dtkIntersectTest.h"
#include "dtkAssert.h"
#include <vector>
#include <algorithm>
#include "dtkIDTypes.h"
using namespace std;

#ifdef DTK_DEBUG
    #define DTK_INTERSECTTEST_DEBUG
#endif //DTK_DEBUG

#ifdef DTK_INTERSECTTEST_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{ 
    bool 
    dtkIntersectTest::Test(
            GK::Point3 &pt, 
            const GK::Ray3 &ray,
            const GK::Triangle3 &tri)
    {
        GK::Plane3 plane = tri.supporting_plane();
        if (Test(pt, ray, plane))
        {
            const GK::Point2 &p0 = plane.to_2d(tri.vertex(0));
            const GK::Point2 &p1 = plane.to_2d(tri.vertex(1));
            const GK::Point2 &p2 = plane.to_2d(tri.vertex(2));
            const GK::Triangle2 tri2(p0, p1, p2);
            const GK::Point2 &p = plane.to_2d(pt);

            CGAL::Bounded_side side = tri2.bounded_side(p);
            return (side == CGAL::ON_BOUNDED_SIDE 
                 || side == CGAL::ON_BOUNDARY);
        }
        return false;
    }

    bool 
    dtkIntersectTest::Test(
            GK::Point3 &pt,
            const GK::Segment3 &seg,
            const GK::Triangle3 &tri)
    {
        GK::Plane3 plane(tri.vertex(0), tri.vertex(1), tri.vertex(2));
        if (Test(pt, seg, plane))
        {
            const GK::Point2 &p0 = plane.to_2d(tri.vertex(0));
            const GK::Point2 &p1 = plane.to_2d(tri.vertex(1));
            const GK::Point2 &p2 = plane.to_2d(tri.vertex(2));
            const GK::Triangle2 tri2(p0, p1, p2);
            const GK::Point2 &p = plane.to_2d(pt);

            CGAL::Bounded_side side = tri2.bounded_side(p);
            return (side == CGAL::ON_BOUNDED_SIDE
                 || side == CGAL::ON_BOUNDARY);
        }
        return false;
    }

    bool
    dtkIntersectTest::Test(
            GK::Point3 &pt,
            const GK::Line3 &line,
            const GK::Triangle3 &tri)
    {
        GK::Plane3 plane(tri.vertex(0), tri.vertex(1), tri.vertex(2));
        if (Test(pt, line, plane))
        {
            const GK::Point2 &p0 = plane.to_2d(tri.vertex(0));
            const GK::Point2 &p1 = plane.to_2d(tri.vertex(1));
            const GK::Point2 &p2 = plane.to_2d(tri.vertex(2));
            const GK::Triangle2 tri2(p0, p1, p2);
            const GK::Point2 &p = plane.to_2d(pt);

            CGAL::Bounded_side side = tri2.bounded_side(p);
            return (side == CGAL::ON_BOUNDED_SIDE
                 || side == CGAL::ON_BOUNDARY);
        }
        return false;
    }

    bool
    dtkIntersectTest::DoIntersect(
            const GK::Tetrahedron3 &tetra,
            const GK::Sphere3 &sphere)
    {
        GK::Point3 center = sphere.center();

        const GK::Point3 &p0 = tetra.vertex(0);
        const GK::Point3 &p1 = tetra.vertex(1);
        const GK::Point3 &p2 = tetra.vertex(2);
        const GK::Point3 &p3 = tetra.vertex(3);

        GK::Vector3 axis;
        GK::Float amin0, amax0, amin1, amax1;

        axis = GK::CrossProduct(p2 - p0, p1 - p0);
        axis = GK::Normalize(axis);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::CrossProduct(p2 - p1, p3 - p1);
        axis = GK::Normalize(axis);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::CrossProduct(p3 - p0, p2 - p0);
        axis = GK::Normalize(axis);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::CrossProduct(p1 - p0, p3 - p0);
        axis = GK::Normalize(axis);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p1 - p0);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p2 - p0);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p3 - p0);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p2 - p1);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p3 - p1);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        axis = GK::Normalize(p3 - p2);
        ProjectToAxis(amin0, amax0, tetra , axis);
        ProjectToAxis(amin1, amax1, sphere, axis);
        if (!IsOverlap(amin0, amax0, amin1, amax1)) return false;

        return true;
    }

    //TODO: may be more faster
    //TODO: may contains some bug
    bool
    dtkIntersectTest::DoIntersect(
            const GK::Triangle2 &triangle,
            const GK::Circle2 &circle)
    {
        GK::Vector2 axis;
        GK::Float amin[2], amax[2];

        const GK::Point2 &v0 = triangle.vertex(0);
        const GK::Point2 &v1 = triangle.vertex(1);
        const GK::Point2 &v2 = triangle.vertex(2);

        CGAL::Aff_transformation_2<GK::K> tp, tpn, rotate, trans;
        rotate = CGAL::Aff_transformation_2<GK::K>(CGAL::ROTATION, 1.0, 0.0);

        tpn = CGAL::Aff_transformation_2<GK::K>(
                CGAL::TRANSLATION, GK::Vector2(v0.x(), v0.y()));
        tp  = CGAL::Aff_transformation_2<GK::K>(
                CGAL::TRANSLATION, GK::Vector2(-v0.x(), -v0.y()));
        trans = tpn * (rotate * tp);

        axis = GK::Normalize(rotate(v1 - v0));
        ProjectToAxis(amin[0], amax[0], triangle, axis);
        ProjectToAxis(amin[1], amax[1], circle  , axis);
        if (!IsOverlap(amin[0], amax[0], amin[1], amax[1]))
            return false;

        axis = GK::Normalize(rotate(v2 - v0));
        ProjectToAxis(amin[0], amax[0], triangle, axis);
        ProjectToAxis(amin[1], amax[1], circle  , axis);
        if (!IsOverlap(amin[0], amax[0], amin[1], amax[1]))
            return false;

        tpn = CGAL::Aff_transformation_2<GK::K>(
                CGAL::TRANSLATION, GK::Vector2(v1.x(), v1.y()));
        tp  = CGAL::Aff_transformation_2<GK::K>(
                CGAL::TRANSLATION, GK::Vector2(-v1.x(), -v1.y()));
        trans = tpn * (rotate * tp);

        axis = GK::Normalize(rotate(v2 - v1));
        ProjectToAxis(amin[0], amax[0], triangle, axis);
        ProjectToAxis(amin[1], amax[1], circle  , axis);
        if (!IsOverlap(amin[0], amax[0], amin[1], amax[1]))
            return false;

        return true;
    }

    bool
    dtkIntersectTest::DoIntersect(
            const GK::Triangle3 &triangle,
            const GK::Sphere3 &sphere)
    {
        GK::Plane3 plane = triangle.supporting_plane();
        GK::Object obj = CGAL::intersection(plane, sphere);

        GK::Point3 p;
        GK::Circle3 circle;

        const GK::Point2 &v0 = plane.to_2d(triangle.vertex(0));
        const GK::Point2 &v1 = plane.to_2d(triangle.vertex(1));
        const GK::Point2 &v2 = plane.to_2d(triangle.vertex(2));
        GK::Triangle2 t2(v0, v1, v2);

        if (GK::Assign(p, obj))
        {
            GK::Point2 p2 = plane.to_2d(p);
            return (!t2.has_on_unbounded_side(p2));
        }
        else if (GK::Assign(circle, obj))
        {
            GK::Point2 c2 = plane.to_2d(circle.center());
            GK::Float r = circle.squared_radius();
            GK::Circle2 circle2(c2, r);

            GK::Triangle2 t2(v0, v1, v2);
            return DoIntersect(t2, circle2);
        }

        return false;
    }

    void
    dtkIntersectTest::ProjectToAxis(
            GK::Float &amin, GK::Float &amax,
            const GK::Triangle2 &triangle,
            const GK::Vector2 &axis)
    {
        GK::Float tmp;
        amin = dtkFloatMax;
        amax = dtkFloatMin;

        const GK::Point2 &v0 = triangle.vertex(0);
        const GK::Point2 &v1 = triangle.vertex(1);
        const GK::Point2 &v2 = triangle.vertex(2);

        tmp = GK::DotProduct(GK::Vector2(v0.x(), v0.y()), axis);
        UpdateMinMax(amin, amax, tmp);

        tmp = GK::DotProduct(GK::Vector2(v1.x(), v1.y()), axis);
        UpdateMinMax(amin, amax, tmp);

        tmp = GK::DotProduct(GK::Vector2(v2.x(), v2.y()), axis);
        UpdateMinMax(amin, amax, tmp);
    }

    void
    dtkIntersectTest::ProjectToAxis(
            GK::Float &amin, GK::Float &amax,
            const GK::Circle2 &circle,
            const GK::Vector2 &axis)
    {
        const GK::Point2 c = circle.center();
        GK::Float tmp = GK::DotProduct(GK::Vector2(c.x(), c.y()), axis);

        GK::Float r = GK::Sqrt(circle.squared_radius());
        amin = tmp - r;
        amax = tmp + r;
    }

    void 
    dtkIntersectTest::ProjectToAxis(
            GK::Float &amin, GK::Float &amax,
            const GK::Tetrahedron3 &tetra,
            const GK::Vector3      &axis)
    {
        GK::Float tmp;
        amin = dtkFloatMax;
        amax = dtkFloatMin;

        const GK::Point3 &v0 = tetra.vertex(0);
        const GK::Point3 &v1 = tetra.vertex(1);
        const GK::Point3 &v2 = tetra.vertex(2);
        const GK::Point3 &v3 = tetra.vertex(3);
        static const GK::Point3 o(0.0, 0.0, 0.0);

        tmp = GK::DotProduct(v0 - o, axis);
        UpdateMinMax(amin, amax, tmp);

        tmp = GK::DotProduct(v1 - o, axis);
        UpdateMinMax(amin, amax, tmp);

        tmp = GK::DotProduct(v2 - o, axis);
        UpdateMinMax(amin, amax, tmp);

        tmp = GK::DotProduct(v3 - o, axis);
        UpdateMinMax(amin, amax, tmp);
    }

    void 
    dtkIntersectTest::ProjectToAxis(
            GK::Float &amin, GK::Float &amax,
            const GK::Sphere3       &sphere,
            const GK::Vector3       &axis)
    {
        static const GK::Point3 o(0.0, 0.0, 0.0);

        GK::Point3 center = sphere.center();
        GK::Float r = GK::Sqrt(sphere.squared_radius());

        GK::Float tmp = GK::DotProduct(center - o, axis);
        amin = tmp - r;
        amax = tmp + r;
    }

    bool
    dtkIntersectTest::IsOverlap(
            GK::Float min0, GK::Float max0,
            GK::Float min1, GK::Float max1)
    {
        return !(max1 < min0 || min1 > max0);
    }
        
    bool 
    dtkIntersectTest::DoDistanceIntersect(
            const GK::Triangle3 &tri,
            const GK::Segment3 &seg,
            double distance, IntersectResult::Ptr& result,
            dtkID invert)
    {
		assert( distance > 0 );

		GK::Point3 triP1 = tri[0];
		GK::Point3 triP2 = tri[1];
		GK::Point3 triP3 = tri[2];

		GK::Point3 segP1 = seg[0];
		GK::Point3 segP2 = seg[1];

		GK::Vector3 tri_unit_normal;

		// justify the segment relative to triangle.
		if( invert == 1 )
			tri_unit_normal = unit_normal( triP1, triP3, triP2 );
		else
			tri_unit_normal = unit_normal( triP1, triP2, triP3 );

		//distance: present the half thickness of object.
		// depth_1, depth_2: present the distance of triangle face with segment's two point.
		double depth_1 = distance - GK::DotProduct( GK::Vector3( triP1, segP1 ), tri_unit_normal );
		double depth_2 = distance - GK::DotProduct( GK::Vector3( triP1, segP2 ), tri_unit_normal );

		bool intersected = false;
		if (depth_1 <= 0 && depth_2 <=0)
			return intersected;

		// CGAL functiion
		if(DoIntersect(tri,seg))
		{
#ifdef DTK_INTERSECTTEST_DEBUG
			cout<<"tri seg directly intersect"<<endl;
#endif
			intersected = true;
		}

		// get the below triangle of added thickness.
		if( !intersected )
		{
			GK::Vector3 planeVec = tri_unit_normal * distance;
			GK::Point3 triP10 = triP1 - planeVec;
			GK::Point3 triP11 = triP1 + planeVec;

			GK::Point3 triP20 = triP2 - planeVec;
			GK::Point3 triP21 = triP2 + planeVec;

			GK::Point3 triP30 = triP3 - planeVec;
			GK::Point3 triP31 = triP3 + planeVec;

			if(DoIntersect(GK::Triangle3(triP10,triP20,triP30),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with bottom face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP11,triP21,triP31),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with upper face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP11,triP21,triP10),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP21,triP10,triP20),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP21,triP31,triP20),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP31,triP20,triP30),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP11,triP31,triP10),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
			else if(DoIntersect(GK::Triangle3(triP31,triP10,triP30),seg)) 
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side face"<<endl;
#endif
				intersected = true;
			}
		}

		if( !intersected )
		{
			// justify two point of segment inside the triangle.
			dtkDouble3 uvw_segP1 = GK::BarycentricWeight( segP1, triP1, triP2, triP3 );
			dtkDouble3 uvw_segP2 = GK::BarycentricWeight( segP2, triP1, triP2, triP3 );

			if( uvw_segP1[0] >= 0 && uvw_segP1[0] <= 1
				&& uvw_segP1[1] >= 0 && uvw_segP1[1] <= 1
				&& uvw_segP1[2] >= 0 && uvw_segP1[2] <= 1
				&& depth_1 >= 0 && depth_1 <= distance * 2.0 )
			{
				intersected = true;
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect inside"<<endl;
#endif
			}
			else if( uvw_segP2[0] >= 0 && uvw_segP2[0] <= 1
				&& uvw_segP2[1] >= 0 && uvw_segP2[1] <= 1
				&& uvw_segP2[2] >= 0 && uvw_segP2[2] <= 1
				&& depth_2 >= 0 && depth_2 <= distance * 2.0 )
			{
				intersected = true;
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect inside"<<endl;
#endif
			}
		}

		if( intersected )
		{
			// depth_1 <= 0 : present no touching of the point of segment and triangle which has stickness.
			// depth_1 > 0:  present touching.
			result = IntersectResult::New();
			assert( depth_1 >= 0 || depth_2 >= 0 );
			dtkDouble2 uv_seg;
			if( depth_1 > 0 )
			{
				if( depth_2 > 0 )
				{
					uv_seg[0] = depth_1 / ( depth_1 + depth_2 );
					uv_seg[1] = 1.0 - uv_seg[0];
				}
				else
				{
					uv_seg[0] = 1.0;
					uv_seg[1] = 0.0;
				}
			}
			else if( depth_2 > 0 )
			{
				uv_seg[0] = 0.0;
				uv_seg[1] = 1.0;
			}
			else
			{
				uv_seg[0] = uv_seg[1] = 0.5;
			}

			// CGAL function: according to two points and correspond two weight to compute barycenter.
			// GK::BarycentricWeight: at first project segP1 to the bottom triangle, then, compute the weight.
			GK::Point3 P2 = barycenter( segP1, uv_seg[0], segP2, uv_seg[1] );
			dtkDouble3 uvw_tri = GK::BarycentricWeight( P2, triP1, triP2, triP3 );
			//GK::Point3 P1 = barycenter( triP1, uvw_tri[0], triP2, uvw_tri[1], triP3, uvw_tri[2] );
			for( int i = 0; i < 3; i++ )
			{
				int a = i;
				int b = (i + 1) % 3;
				int c = (i + 2) % 3;
				if( uvw_tri[a] < 0 )
				{
					if( uvw_tri[b] < 0 )
					{
						uvw_tri[a] = uvw_tri[b] = 0.0;
						uvw_tri[c] = 1.0;
						break;
					}
					else if( uvw_tri[c] < 0 )
					{
						uvw_tri[a] = uvw_tri[c] = 0.0;
						uvw_tri[b] = 1.0;
						break;
					}
					else
					{
						uvw_tri[b] += uvw_tri[a] * uvw_tri[b] / ( uvw_tri[b]+ uvw_tri[c] );
						uvw_tri[c] = 1.0 - uvw_tri[b];
						uvw_tri[a] = 0;
						break;
					}
				}
			}

			double sum_depth = 0.0;
			if( depth_1 > 0 )
				sum_depth += depth_1;
			if( depth_2 > 0 )
				sum_depth += depth_2;

			result->SetProperty( INTERSECT_WEIGHT_1, uvw_tri );
			result->SetProperty( INTERSECT_WEIGHT_2, uv_seg );
			result->SetProperty( INTERSECT_NORMAL, tri_unit_normal * sum_depth );
		}
		else
		{
			if(DoDistanceIntersect(GK::Segment3(triP1,triP2),seg, distance, result))
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side line"<<endl;
#endif
				dtkDouble2 uv;
				result->GetProperty( INTERSECT_WEIGHT_1, uv );
				result->SetProperty( INTERSECT_WEIGHT_1, dtkDouble3( uv[0], uv[1], 0) );
				intersected = true;
			}
			else if(DoDistanceIntersect(GK::Segment3(triP2,triP3),seg, distance, result))
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side line"<<endl;
#endif
				dtkDouble2 uv;
				result->GetProperty( INTERSECT_WEIGHT_1, uv );
				result->SetProperty( INTERSECT_WEIGHT_1, dtkDouble3( 0, uv[0], uv[1]) );
				intersected = true;
			}
			else if(DoDistanceIntersect(GK::Segment3(triP3,triP1),seg, distance, result))
			{
#ifdef DTK_INTERSECTTEST_DEBUG
				cout<<"intersect with side line"<<endl;
#endif
				dtkDouble2 uv;
				result->GetProperty( INTERSECT_WEIGHT_1, uv );
				result->SetProperty( INTERSECT_WEIGHT_1, dtkDouble3( uv[1], 0, uv[0]) );
				intersected = true;
			}

			if( intersected )
			{
				GK::Vector3 normal;
				result->GetProperty( INTERSECT_NORMAL, normal );
				double tempDepth = GK::DotProduct( normal, tri_unit_normal );
				if( tempDepth < 0 )
				{
					normal = normal - tri_unit_normal * tempDepth * 2;
				}
				result->SetProperty( INTERSECT_NORMAL, normal );
			}
		}
		return intersected;
    }
        
    bool 
    dtkIntersectTest::DoDistanceIntersect(
            const GK::Segment3 &seg_0, 
            const GK::Segment3 &seg_1, 
            double distance, IntersectResult::Ptr& result )
    {
        assert( distance > 0 );

        GK::Point3 p11 = seg_0[0];
        GK::Point3 p12 = seg_0[1];
        GK::Point3 p21 = seg_1[0];
        GK::Point3 p22 = seg_1[1];
        GK::Vector3 v_side_edge_11_21 = p21 - p11;
        GK::Vector3 v_side_edge_11_22 = p22 - p11;
        GK::Vector3 v_side_edge_12_21 = p21 - p12;
        GK::Vector3 v_side_edge_12_22 = p22 - p12;
        GK::Vector3 v_edge_1 = p12 - p11;
        GK::Vector3 v_edge_2 = p22 - p21;
        GK::Vector3 v_edge_1_normal = GK::Normalize(v_edge_1);
        GK::Vector3 v_edge_2_normal = GK::Normalize(v_edge_2);
        double v_edge_1_length = GK::DotProduct(v_edge_1,v_edge_1_normal);
        double v_edge_2_length = GK::DotProduct(v_edge_2,v_edge_2_normal);
        bool intersect = false;
            
        double length; // plane to plane distance
        GK::Vector3 apply_normal;
        dtkDouble2 uv1, uv2;

		// justify if the two vector:v_edge_1, v_edge_2 parallel .  
        if( CGAL::collinear( p11, p12, p12 + v_edge_2) )
        {
			// four point of two segments collinear
            if( CGAL::collinear( p11, p12, p22) )
                return false;

			// seg_1 parallel seg_2.
            // from seg_1 to seg_2.
			// compute the vertical line of seg_0 and seg_1.
            GK::Vector3 normal = GK::Normalize(
                GK::CrossProduct( 
                    GK::CrossProduct( v_side_edge_11_21, v_edge_1 ),
                    -v_edge_1 )
                );
			
			// compute the distance of seg_0 and seg_1.
            length = GK::DotProduct( normal, v_side_edge_11_21 );
                
            double project_21_1 = GK::DotProduct( v_side_edge_11_21, v_edge_1_normal );
            double project_22_1 = GK::DotProduct( v_side_edge_11_22, v_edge_1_normal );
        
            if( length < distance )
            {
				// justify if intersect of seg_0 and seg_1 at the horizontal direction.  
                if( ( project_21_1 > 0 && project_21_1 < v_edge_1_length ) 
                        || ( project_22_1 > 0 && project_22_1 < v_edge_1_length ) )
                    intersect = true;
            }

            if( intersect ) // parallel line intersect
            {
                vector<double> l;
                l.resize(4);
                vector<double> l_ori;
                l_ori.resize(4);

                l[0] = l_ori[0] = 0;
                l[1] = l_ori[1] = v_edge_1_length;
                l[2] = l_ori[2] = project_21_1;
                l[3] = l_ori[3] = project_22_1; 

                sort( l.begin(), l.end() );
 
				// compute the intersect_weight of seg_0 and seg_1.
                double mid = (l[2] + l[1]) / 2.0;
                uv1[0] = abs( mid - l_ori[1]) / abs( l_ori[0] - l_ori[1] ); 
                uv1[1] = 1.0 - uv1[0];
                uv2[0] = abs( mid - l_ori[3]) / abs( l_ori[3] - l_ori[2] ); 
                uv2[1] = 1.0 - uv2[0];

                result = IntersectResult::New();
#ifdef DTK_INTERSECTTEST_DEBUG
                cout << "paralell segments intersect." << endl;
#endif
                assert( length >= 0 );
				// compute the common normal of two parallel line.
                result->SetProperty( INTERSECT_NORMAL, normal * ( distance - length ) * 2.0 );
                result->SetProperty( INTERSECT_WEIGHT_1, uv1 ); 
                result->SetProperty( INTERSECT_WEIGHT_2, uv2 ); 
            }
        }
        else
        {
			// normal is Common perpendicular of seg1 and seg2.
            GK::Vector3 normal = unit_normal( p11, p12, p12 + v_edge_2 );
            length = GK::DotProduct( normal, v_side_edge_11_21 );
            if( length < 0 )
            {
                normal = -normal;
                length = -length;
            }
            
            if( length < distance)
            {
                GK::Vector3 n_edge_face_1 = GK::CrossProduct( normal, v_edge_1 );
                GK::Vector3 n_edge_face_2 = GK::CrossProduct( v_edge_2, normal );
                double fraction_1 = GK::DotProduct ( n_edge_face_2, v_side_edge_11_21 ) / GK::DotProduct( n_edge_face_2, v_edge_1);
                double fraction_2 = - GK::DotProduct( n_edge_face_1, v_side_edge_11_21 ) / GK::DotProduct( n_edge_face_1, v_edge_2);

                if( fraction_1 >= 0 && fraction_1 <= 1 )
                {
                    if( fraction_2 >= 0 && fraction_2 <= 1 )    
                    {
                        uv1 = dtkDouble2( 1.0 - fraction_1, fraction_1 );
                        uv2 = dtkDouble2( 1.0 - fraction_2, fraction_2 );
                        apply_normal = normal;
                        intersect = true;
                    }
                    else
                    {
                        GK::Point3 P1 = p11 - v_edge_1 * fraction_1;
                        if(fraction_2 < 0) 
                        {
                            uv2[0] = 1;
                            uv2[1] = 0;
                            apply_normal = p21 - P1;
                            length = GK::Length( apply_normal );
                            apply_normal = GK::Normalize( apply_normal );
                        }
                        else
                        {
                            uv2[0] = 0;
                            uv2[1] = 1;
                            apply_normal = p22 - P1;
                            length = GK::Length( apply_normal );
                            apply_normal = GK::Normalize( apply_normal );
                        }
                        if( length < distance )
                        {
                            uv1 = dtkDouble2( 1.0 - fraction_1, fraction_1 );
                            intersect = true;
                        }
                    }
                }
                else 
                {
                    if( fraction_2 >= 0 && fraction_2 <= 1 )    
                    {
                        GK::Point3 P2 = p21 - v_edge_2 * fraction_2;
                        if(fraction_1 < 0)
                        {
                            uv1[0] = 1;
                            uv1[1] = 0;
                            apply_normal = P2 - p11;
                            length = GK::Length( apply_normal );
                            apply_normal = GK::Normalize( apply_normal );
                        }
                        else
                        {
                            uv1[0] = 0;
                            uv1[1] = 1;
                            apply_normal = P2 - p12;
                            length = GK::Length( apply_normal );
                            apply_normal = GK::Normalize( apply_normal );
                        }
                        if( length < distance )
                        {
                            uv2 = dtkDouble2( 1.0 - fraction_2, fraction_2 );
                            intersect = true;
                        }
                    }
                }
            }
        
            if(intersect)
            {
#ifdef DTK_INTERSECTTEST_DEBUG
                cout << "segments intersect." << endl;
#endif
                assert( length >= 0 );
                result = IntersectResult::New();
                result->SetProperty( INTERSECT_NORMAL, apply_normal * ( distance - length ) * 2.0 );
                result->SetProperty( INTERSECT_WEIGHT_1, uv1 ); 
                result->SetProperty( INTERSECT_WEIGHT_2, uv2 ); 
            }
        }

        if( !intersect )
        {
            double penetrate[4] = { dtkDoubleMin, dtkDoubleMin, dtkDoubleMin, dtkDoubleMin };
            if( length < distance )
            {
                double temp = distance - GK::Length( v_side_edge_11_21);
                penetrate[0] = max( penetrate[0], temp );
                penetrate[2] = max( penetrate[2], temp );
                temp = distance - GK::Length( v_side_edge_11_22);
                penetrate[0] = max( penetrate[0], temp );
                penetrate[3] = max( penetrate[3], temp );
                temp = distance - GK::Length( v_side_edge_12_21);
                penetrate[1] = max( penetrate[1], temp );
                penetrate[2] = max( penetrate[2], temp );
                temp = distance - GK::Length( v_side_edge_12_22);
                penetrate[1] = max( penetrate[1], temp );
                penetrate[3] = max( penetrate[3], temp );

                for( dtkID i = 0; i < 4; i++ )
                {
                    if( penetrate[i] > 0 )
                        intersect = true;

                    if( penetrate[i] < 0 )
                        penetrate[i] = 0;
                }
            }

            if( intersect ) // parallel line end intersect
            {
                uv1[0] = penetrate[0] / ( penetrate[0] + penetrate[1] ); 
                uv1[1] = 1.0 - uv1[0];
                uv2[0] = penetrate[2] / ( penetrate[2] + penetrate[3] ); 
                uv2[1] = 1.0 - uv2[0];

                GK::Point3 P1 = barycenter( p11, uv1[0], p12, uv1[1] );
                GK::Point3 P2 = barycenter( p21, uv2[0], p22, uv2[1] );
                GK::Vector3 apply_normal = GK::Normalize( P2 - P1 );

                //length = max( penetrate[0] + penetrate[1], penetrate[2] + penetrate[3] );
                length = GK::Length( P2 - P1 ) * 2.0;

                result = IntersectResult::New();
#ifdef DTK_INTERSECTTEST_DEBUG
                cout << "segment endpoints intersect." << endl;
#endif
                assert( length >= 0 );
                result->SetProperty( INTERSECT_NORMAL, apply_normal * ( distance * 2.0 - length) );
                result->SetProperty( INTERSECT_WEIGHT_1, uv1 ); 
                result->SetProperty( INTERSECT_WEIGHT_2, uv2 ); 
            }
        }

        return intersect;

        //if(GK::DotProduct(v_edge_1_normal,v_edge_2_normal) == 1 || 
        /* YS implement
        dtkDouble2 uv1,uv2;
        if( CGAL::collinear( p11, p12, p12 + v_edge_2) )
        //if(GK::DotProduct(v_edge_1_normal,v_edge_2_normal) == 1 || 
        //    GK::DotProduct(v_edge_1_normal,v_edge_2_normal) == -1)  
        {
            GK::Vector3 v11 = p11 - p21;
            GK::Vector3 v12 = p11 - p22;
            GK::Vector3 v21 = p12 - p21;
            GK::Vector3 v22 = p12 - p22;

            vector<double> l;
            l.resize(4);
            double l_ori[4];
            int order[4];
            
            double s1 = sqrt(GK::DotProduct(v11,v11));
            double s2 = sqrt(GK::DotProduct(v12,v12));
            double edge_2_length = sqrt(GK::DotProduct(v_edge_2,v_edge_2));
            double edge_1_length = sqrt(GK::DotProduct(v_edge_1,v_edge_1));
            double p = (s1 + s2 + edge_2_length) / 2;
            double s = sqrt(p * (p - s1) * (p - s2) * (p - edge_2_length));
            double length = 2 * s / edge_2_length;
            if(length < distance)
            {
                if(abs(GK::DotProduct(v11,v_edge_1_normal) / edge_2_length) <= 1 &&
                    abs(GK::DotProduct(v12,-v_edge_1_normal) / edge_2_length) <= 1)
                {
                    intersect = true;
                }
                else
                {
                    if(abs(GK::DotProduct(v21,v_edge_1_normal) / edge_2_length) <= 1 &&
                        abs(GK::DotProduct(v22,-v_edge_1_normal) / edge_2_length) <= 1)
                    {
                        intersect = true;
                    }
                    else
                    {
                        if(abs(GK::DotProduct(v11, v_edge_2_normal) / edge_1_length) <= 1 &&
                            abs(GK::DotProduct(v21, v_edge_2_normal) / edge_1_length) <= 1)
                            intersect = true;
                        else
                        {
                            if(abs(GK::DotProduct(v12, v_edge_2_normal) / edge_1_length) <= 1 &&
                            abs(GK::DotProduct(v22, v_edge_2_normal) / edge_1_length) <= 1)
                                intersect = true;
                        }
                    }
                }
            }

            if( intersect )
            {
                l[0] = 0;
                l[1] = edge_1_length;
                l[2] = GK::DotProduct(-v11,-v_edge_1_normal);
                l[3] = GK::DotProduct(-v12,-v_edge_1_normal); 
                for( dtkID i = 0; i < 4; i++ )
                {
                    l_ori[i] = l[i];
                }
                sort( l.begin(), l.end() );
 
                double mid = (l[2] + l[1]) / 2.0;
                cout << "mid" << mid << endl;
                uv1[0] = abs( mid - l_ori[1]) / abs( l_ori[0] - l_ori[1] ); 
                uv1[1] = 1.0 - uv1[0];
                uv2[0] = abs( mid - l_ori[3]) / abs( l_ori[3] - l_ori[2] ); 
                uv2[1] = 1.0 - uv2[0];

                result = IntersectResult::New();
                GK::Vector3 normal = GK::CrossProduct( -v_edge_2_normal, v11);
                normal = GK::CrossProduct( normal, v_edge_2_normal);
                normal = GK::Normalize( normal );
                normal = normal * ( distance - abs(length) ) * 2; // double penetrate depth
                result->SetProperty( INTERSECT_NORMAL, normal );
                result->SetProperty( INTERSECT_WEIGHT_1, uv1 ); 
                result->SetProperty( INTERSECT_WEIGHT_2, uv2 ); 
            }
            return intersect;
        }

        GK::Vector3 n_area_unit = unit_normal( p11, p12, p11 + v_edge_2 );
        GK::Vector3 n_edge_face_1 = GK::CrossProduct( n_area_unit, v_edge_1 );
        GK::Vector3 n_edge_face_2 = GK::CrossProduct( v_edge_2, n_area_unit );
        double length = GK::DotProduct( n_area_unit, v_side_edge);
        double fraction_1 = GK::DotProduct ( n_edge_face_2, v_side_edge ) / GK::DotProduct( n_edge_face_2, v_edge_1);
        //GK::Vector2 uv_1( 1.0 - fraction_1, fraction_1 );
        double fraction_2 = - GK::DotProduct( n_edge_face_1, v_side_edge ) / GK::DotProduct( n_edge_face_1, v_edge_2);
        //GK::Vector2 uv_2( 1.0 - fraction_2, fraction_2 );

        if( fraction_1 >= 0 && fraction_1 <= 1 )
        {
            if( fraction_2 >= 0 && fraction_2 <= 1 )    
            {
                if( abs(length) < distance)
                {
                    uv1 = dtkDouble2( 1.0 - fraction_1, fraction_1 );
                    uv2 = dtkDouble2( 1.0 - fraction_2, fraction_2 );
                    intersect = true;
                }
                else
                    intersect = false;
            }
            else
            {
                GK::Point3 P1 = p11 - v_edge_1 * fraction_1;
                if(fraction_2 < 0) 
                {
                    uv2[0] = 1;
                    uv2[1] = 0;
                    length = sqrt( GK::DotProduct( P1 - p21, P1 - p21));
                }
                else
                {
                    uv2[0] = 0;
                    uv2[1] = 1;
                    length = sqrt( GK::DotProduct( P1 - p22, P1 - p22));
                }
                if( abs(length) < distance)
                {
                    uv1 = dtkDouble2( 1.0 - fraction_1, fraction_1 );
                    intersect = true;
                }
                else
                    intersect = false;
            }
        }
        else 
        {
            if( fraction_2 >= 0 && fraction_2 <= 1 )    
            {
                GK::Point3 P2 = p21 - v_edge_2 * fraction_2;
                if(fraction_1 < 0)
                {
                    uv1[0] = 1;
                    uv1[1] = 0;
                    length = sqrt( GK::DotProduct( P2 - p11, P2 - p11));
                }
                else
                {
                    uv1[0] = 0;
                    uv1[1] = 1;
                    length = sqrt( GK::DotProduct( P2 - p12, P2 - p12));
                }
                if(length < distance)
                {
                    uv2 = dtkDouble2( 1.0 - fraction_2, fraction_2 );
                    intersect = true;
                }
                else
                    intersect = false;
            }
            else
                intersect = false;
        }
        if(intersect)
        {
            result = IntersectResult::New();
            GK::Vector3 normal = GK::CrossProduct( v_edge_2_normal, v_edge_1_normal );
            normal = GK::Normalize( normal );
            normal = normal * ( distance - abs(length) ) * 2; // double penetrate depth
            result->SetProperty( INTERSECT_NORMAL, normal );
            result->SetProperty( INTERSECT_WEIGHT_1, uv1 ); 
            result->SetProperty( INTERSECT_WEIGHT_2, uv2 ); 
        }
        return intersect;
        */
    }
        
    bool 
    dtkIntersectTest::DoDistanceIntersect(
            const GK::Triangle3 &tri_1, 
            const GK::Triangle3 &tri_2, 
            double distance, IntersectResult::Ptr& result,
			dtkID invert)
    {
		bool intersect = false;
		
		if( invert == 2 ) // tri_2 contains tri_1
		{
			GK::Vector3 normal_tri = unit_normal( tri_2[0], tri_2[1], tri_2[2] );
			dtkDouble3 uvw1(0,0,0);
			for( dtkID i = 0; i < 3; i++ )
			{
				GK::Float apart = GK::DotProduct( normal_tri, tri_1[i] - tri_2[0] );

				if( abs(apart) > distance )
					continue;

				dtkDouble3 uvw = GK::BarycentricWeight( tri_1[i], tri_2[0], tri_2[1], tri_2[2] );
				if( uvw[0] < 0 || uvw[0] > 1 ||
					uvw[1] < 0 || uvw[2] > 1 ||
					uvw[2] < 0 || uvw[2] > 1 )
					continue;

				uvw1[i] = 1;
				intersect = true;
			}

			if( intersect )
			{
				result = IntersectResult::New();
				result->SetProperty( INTERSECT_WEIGHT_1, uvw1 );
			}

			return intersect;
		}

        GK::Point3 center1 = GK::Centroid( tri_1 );
        GK::Point3 center2 = GK::Centroid( tri_2 );
        GK::Vector3 normal = center2 - center1;
        GK::Point3 triP11 = tri_1[0];
        GK::Point3 triP12 = tri_1[1];
        GK::Point3 triP13 = tri_1[2];

        GK::Point3 triP21 = tri_2[0];
        GK::Point3 triP22 = tri_2[1];
        GK::Point3 triP23 = tri_2[2];

        double maxL1, maxL2, l;
        maxL1 = GK::Length( tri_1[0] - center1 );
        maxL2 = GK::Length( tri_2[0] - center2 );
        for( dtkID i = 1; i < 3; i++ )
        {
           l = GK::Length( tri_1[i] - center1 ); 
           if( l < maxL1 )
               maxL1 = l;
        }

        for( dtkID i = 1; i < 3; i++ )
        {
           l = GK::Length( tri_2[i] - center2 ); 
           if( l < maxL2 )
               maxL2 = l;
        }

        GK::Vector3 face1Normal, face2Normal;
		face1Normal = unit_normal( tri_1[0], tri_1[1], tri_1[2] );
		face2Normal = unit_normal( tri_2[0], tri_2[1], tri_2[2] );

		// if the condition doesn't mean, two triangle can't intersect.
        if( GK::Length( normal ) < maxL1 + maxL2 )
        {
			for( dtkID i = 0; i < 3; i++ )
			{
				l = GK::DotProduct( face1Normal, tri_2[i] - tri_1[0] );
				if( abs(l) < distance )
				{
					intersect = true;
					break;
				}
			}

			if(!intersect)
			{
				for( dtkID i = 0; i < 3; i++ )
				{
					l = GK::DotProduct( face2Normal, tri_1[i] - tri_2[0] );
					if( abs(l) < distance )
					{
						intersect = true;
						break;
					}
				}
			}
        }

		GK::Vector3 n;
		dtkDouble3 uvw1, uvw2;

		double pro11;  
		double pro12;  
		double pro13;  
		double pro21;  
		double pro22;  
		double pro23;  

		double sum1, sum2;

        if( intersect )
        {
            double halfDistance = distance / 2.0;
            if( GK::DotProduct( face1Normal, face2Normal ) >= 0 )
                n = GK::Normalize( face1Normal + face2Normal );
            else
                n = GK::Normalize( face1Normal - face2Normal );
            GK::Point3 midPointBetweenCenters = barycenter( center1, 0.5, center2 );    

            GK::Point3 surfacePoint1 = midPointBetweenCenters + n * ( -halfDistance );
            GK::Point3 surfacePoint2 = midPointBetweenCenters + n * ( halfDistance );

            pro21 = GK::DotProduct( -n, triP21 - surfacePoint2 );  
            pro22 = GK::DotProduct( -n, triP22 - surfacePoint2 );  
            pro23 = GK::DotProduct( -n, triP23 - surfacePoint2 );  
            
            pro11 = GK::DotProduct( n, triP11 - surfacePoint1 );  
            pro12 = GK::DotProduct( n, triP12 - surfacePoint1 );  
            pro13 = GK::DotProduct( n, triP13 - surfacePoint1 );  

			if( pro11 < 0 ) pro11 = 0;
			if( pro12 < 0 ) pro12 = 0;
			if( pro13 < 0 ) pro13 = 0;

			if( pro21 < 0 ) pro21 = 0;
			if( pro22 < 0 ) pro22 = 0;
			if( pro23 < 0 ) pro23 = 0;

			sum1 = pro11 + pro12 + pro13;
			sum2 = pro21 + pro22 + pro23;

			if( sum1 == 0 || sum2 == 0 )
				intersect = false;
		}
		if(intersect)
		{
			if( GK::DotProduct( face1Normal, face2Normal ) > 0 )
				intersect = false;
		}
		if(intersect)
		{
            uvw1[0] = pro11 / sum1;
            uvw1[1] = pro12 / sum1;
            uvw1[2] = pro13 / sum1;

            uvw2[0] = pro21 / sum2;
            uvw2[1] = pro22 / sum2;
            uvw2[2] = pro23 / sum2;

            result = IntersectResult::New();
            normal = n * ( sum1 + sum2 );
            result->SetProperty( INTERSECT_NORMAL, normal );
            result->SetProperty( INTERSECT_WEIGHT_1, uvw1 );
            result->SetProperty( INTERSECT_WEIGHT_2, uvw2 );
        }
        return intersect;
    }

	bool 
	dtkIntersectTest::DoDistanceIntersect(
		const GK::Triangle3 &tri, 
		const GK::Sphere3 &sphere, 
		double distance, 
		IntersectResult::Ptr& result )
	{
		bool intersect = false;
		GK::Vector3 normal_tri = unit_normal( tri[0], tri[2], tri[1] );

		GK::Float apart = GK::DotProduct( normal_tri, sphere.center() - tri[0] );

		// collision detection two sides
	/*	if( abs(apart) > distance )
			return false;*/

		// collision detection one side
		if (apart > distance )
			return false;

		// avoid collision detection disturbed by others triangle mesh
		if (apart < - 0.5 * distance )
			return false;

		GK::Vector3 normal = normal_tri * ( distance - apart );

		dtkDouble3 uvw = GK::BarycentricWeight( sphere.center(), tri[0], tri[1], tri[2] );
		if( uvw[0] < 0 || uvw[0] > 1 ||
			uvw[1] < 0 || uvw[2] > 1 ||
			uvw[2] < 0 || uvw[2] > 1 )
			return false;

		double weight2 = 1.0;
		result = IntersectResult::New();
		result->SetProperty( INTERSECT_NORMAL, normal );
		result->SetProperty( INTERSECT_WEIGHT_1, uvw );
		result->SetProperty( INTERSECT_WEIGHT_2, weight2 );

		return true;
	}

    bool 
    dtkIntersectTest::DoIntersect(
            const GK::Triangle3 &triangle_1, 
            const GK::Triangle3 &triangle_2, 
            IntersectResult::Ptr& result )
    {
        if( CGAL::do_intersect(triangle_1, triangle_2) )
        {
            result = IntersectResult::New();
            return true;
        }
        else
            return false;
    }
        
    bool 
    dtkIntersectTest::DoIntersect(
            const GK::Triangle3 &triangle, 
            const GK::Segment3 &segment, 
            IntersectResult::Ptr& result )
    {
        if( CGAL::do_intersect(triangle, segment) )
        {
            result = IntersectResult::New();
            GK::Object object = CGAL::intersection(triangle, segment);
            result->SetProperty( INTERSECT_OBJECT, object );
            return true;
        }
        else
            return false;
    }

    bool 
    dtkIntersectTest::DoIntersect(
            const GK::Segment3 &seg_1, 
            const GK::Segment3 &seg_2, 
            IntersectResult::Ptr& result )
    {
        // wait for CGAL 3.7
        /*
        if( CGAL::do_intersect(seg_1, seg_2) )
        {
            result = IntersectResult::New();
            return true;
        }
        else
            return false;
        */
        if( CGAL::do_intersect(seg_1, seg_2) )
        {
            result = IntersectResult::New();
            return true;
        }
        else
            return false;
        //dtkAssert( false, NOT_IMPLEMENTED );
        return false;
    }

	bool 
		dtkIntersectTest::Test(
		const GK::BBox3 &box1, 
		const GK::BBox3 &box2)
	{
		return CGAL::do_overlap(box1, box2);
	}

	bool
		dtkIntersectTest::Test(
		GK::Point3 &pt,
		const GK::Line3 &line,
		const GK::Plane3 &plane)
	{
		CGAL::Object result = CGAL::intersection(plane, line);
		return (CGAL::assign(pt, result));
	}

	bool 
		dtkIntersectTest::Test(
		GK::Point3 &pt,
		const GK::Segment3 &seg,
		const GK::Plane3 &plane)
	{
		return CGAL::assign(pt, CGAL::intersection(plane, seg));
	}

	bool 
		dtkIntersectTest::Test(
		GK::Point3 &pt, 
		const GK::Ray3 &ray,
		const GK::Plane3 &plane)
	{
		return CGAL::assign(pt, CGAL::intersection(plane, ray));
	}

	bool 
		dtkIntersectTest::DoIntersect(
		const GK::BBox3 &box1, 
		const GK::BBox3 &box2)
	{
		return CGAL::do_overlap(box1, box2);
	}

	bool
		dtkIntersectTest::DoIntersect(
		const GK::Segment2 &seg0,
		const GK::Segment2 &seg1)
	{
		return CGAL::do_intersect(seg0, seg1);
	}

	bool
		dtkIntersectTest::DoIntersect(
		const GK::Line3 &line,
		const GK::Triangle3 &tri)
	{
		return CGAL::do_intersect(tri, line);
	}

	bool 
		dtkIntersectTest::DoIntersect(
		const GK::Segment3 &seg_1, 
		const GK::Segment3 &seg_2)
	{
		// wait for CGAL 3.7
		return CGAL::do_intersect( seg_1, seg_2 );
		//dtkAssert( false, NOT_IMPLEMENTED );
		//return false;
	}

	bool
		dtkIntersectTest::DoIntersect(
		const GK::Segment3 &seg,
		const GK::Triangle3 &tri)
	{
		return CGAL::do_intersect(tri, seg);
	}

	bool
		dtkIntersectTest::DoIntersect(
		const GK::Segment3 &seg,
		const GK::Plane3   &plane)
	{
		return CGAL::do_intersect(plane, seg);
	}

	bool
		dtkIntersectTest::DoIntersect(
		const GK::Ray3 &ray,
		const GK::Plane3 &plane)
	{
		return CGAL::do_intersect(plane, ray);
	}

	bool 
		dtkIntersectTest::DoIntersect(
		const GK::Triangle3 &triangle_1, 
		const GK::Triangle3 &triangle_2)
	{
		return CGAL::do_intersect(triangle_1, triangle_2);
	}

	bool 
		dtkIntersectTest::DoIntersect(
		const GK::Triangle3 &triangle, 
		const GK::Segment3 &segment)
	{
		return CGAL::do_intersect(triangle, segment);
	}

	bool 
		dtkIntersectTest::DoIntersect( 
		const GK::Interval& int_1, 
		const GK::Interval& int_2)
	{
		if( int_2.mLower > int_2.mUpper )
			return false;

		if( int_1.Contain( int_2.mLower ) || int_1.Contain( int_2.mUpper ) )
			return true;
		else if( int_2.Contain( int_1.mLower ) )
			return true;
		else
			return false;
	}

	bool
		dtkIntersectTest::DoIntersect( 
		const GK::KDOP& kdop_1, 
		const GK::KDOP& kdop_2)
	{
		assert( kdop_1.mHalfK == kdop_2.mHalfK );

		for( dtkID i = 0; i < kdop_1.mHalfK; i++ )
		{
			if( !DoIntersect( kdop_1.mIntervals[i], kdop_2.mIntervals[i] ) )
				return false;
		}
		return true;
	}

	void
		dtkIntersectTest::UpdateMinMax(
		GK::Float &tmin, GK::Float &tmax, 
		const GK::Float &val)
	{
		if (val > tmax) tmax = val;
		if (val < tmin) tmin = val;
	}
}

