#ifndef COLLISIONRESPONSE_H
#define  COLLISIONRESPONSE_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkPhysMassSpring.h"
#include "dtkIntersectTest.h"
#include "guideWire.h"
#include <vector>
#include <map>

namespace dtk
{
	class collisionResponse : public boost::noncopyable
	{

	public:
		typedef std::shared_ptr< collisionResponse > Ptr;

		static Ptr New(dtkStaticTriangleMesh::Ptr ptr) 
		{
			return Ptr( new collisionResponse(ptr) );
		}

	public:
		~collisionResponse();

		void Update( double timeslice, 
			std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, const std::vector< dtkID3>& avoid_1, double stiffness);

		void SetMassSpring( dtkID i, dtkPhysMassSpring::Ptr massSpring )
		{
			mMassSprings[i] = massSpring;
		}

		void RemoveMassSpring( dtkID i )
		{
			mMassSprings.erase( i );
		}

		dtkPhysMassSpring::Ptr GetMassSpring( dtkID majorID )
		{
			assert( mMassSprings.find( majorID ) != mMassSprings.end() );
			return mMassSprings[majorID];
		}

		void SetGuideWire(dtkID i, guideWire::Ptr massPoints)
		{
			mGuideWireMassPoints[i] = massPoints;
		}

		void RemoveGuideWire( dtkID i)
		{
			mGuideWireMassPoints.erase(i);
		}

		guideWire::Ptr GetGuideWire( dtkID majorID)
		{
			assert(mGuideWireMassPoints.find(majorID) != mGuideWireMassPoints.end());
			return mGuideWireMassPoints[majorID];
		}

		void collisionResponse::ResetContactForces()
		{
			mGuideWireMassPoints[1]->ResetContactForces();
		}

		void collisionResponse::ResetCollisionFlag()
		{
			mGuideWireMassPoints[1]->ResetCollisionFlag();
		}


	private:
		collisionResponse(dtkStaticTriangleMesh::Ptr ptr);
	private:
		std::map<dtkID, dtkPhysMassSpring::Ptr > mMassSprings;
		std::map<dtkID, guideWire::Ptr > mGuideWireMassPoints;

		dtkStaticTriangleMesh::Ptr mVascularSurfaceMesh;
		std::vector<GK::Vector3> mVascularSurfaceNormals; 
	};
}

#endif 
