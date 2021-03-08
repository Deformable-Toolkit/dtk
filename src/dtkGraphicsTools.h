#ifndef DTK_GRAPHICSTOOLS_H
#define DTK_GRAPHICSTOOLS_H

#include "dtkPoints.h"

/***************************************************
 * ATTENTION
 * ---------------------------
 *  This file is designed to deprecated.
 *  Use dtkIntersectTest instead.
 ***************************************************/

namespace dtk
{
	//Calculate the bounding box of points.
	extern void CalcBoundingBox(dtkFloat3 &boxMin, dtkFloat3 &boxMax, dtkPoints::Ptr points);

	//calculte the circumsphere of a tetrahedron, return the center and radius's square of the sphere.
	extern void CalcCircumSphere2(dtkFloat3 &center, float &radius2, 
		const dtkFloat3 &x1, const dtkFloat3 &x2, 
		const dtkFloat3 &x3, const dtkFloat3 &x4,
        float tolerance);

	//calculte the circumsphere of a tetrahedron, return the center and radius's square of the sphere.
	inline void CalcCircumSphere2(dtkFloat3 &center, float &radius2, dtkFloat3 pts[4], float tolerance)
	{
		CalcCircumSphere2(center, radius2, pts[0], pts[1], pts[2], pts[3], tolerance);
	}

	//Calculate the area of a triangle.
	extern float Area(const dtkFloat3 &ptA, const dtkFloat3 &ptB, const dtkFloat3 &ptC);

	//Test whether a sphere is intersect with a AABB Bounding-Box
	extern bool Intersect_SphereAABB(const dtkFloat3 &sphereCenter, const float &radius2, const dtkFloat3 &boxMin, const dtkFloat3 &boxMax);

	//Test whether a point is inside/on/outside a tetrahedron.
	//Inputs:
	//	pt: the points coordinate
	//	pa, pb, pc, pd: the tetrahedron's vertices' coordinates
	//Return:
	//	-1: outisde, 0: on the boundary, 1: inside.
	extern int Inclusion_TetraPoint(const dtkFloat3 &pt, const dtkFloat3 &pa, dtkFloat3 &pb, dtkFloat3 &pc, dtkFloat3 &pd);
}

#endif //DTK_GRAPHICSTOOLS_H
