#ifndef CONTROLCORE_H
#define CONTROLCORE_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkPhysTetraMassSpring.h"
#include "dtkCollisionDetectStage.h"
#include "dtkCollisionDetectHierarchyKDOPS.h"
#include "dtkIntersectTest.h"

#include "collisionResponse.h"

namespace dtk
{
	class controlCore : public boost::noncopyable
	{
	public:

		typedef struct  
		{
			dtkCollisionDetectStage::HierarchyPair hierarchy_pair;
			double strength;
		} CollisionResponseSet;

		double GetCollisionDetectTime() const
		{
			return mCollisionDetectTime;
		}
		double GetUpdateGuideWireTime() const
		{
			return mUpdateGuideWireTime;
		}
		double GetRotateOnceTime() const
		{
			return mRotateOnceTime;
		}

		// 计算力反馈器在操纵导丝的过程中所受到的力
		dtkT3<double> GetHapticTranslationForce();
		dtkT3<double> GetHapticCollisionForce();

	public:
		typedef std::shared_ptr< controlCore > Ptr;

		static Ptr New( double clothDepth = 2.0 ) 
		{
			return Ptr( new controlCore(clothDepth) );
		}

	public:
		~controlCore();

		void Update( double timeslice, const vector<dtkID3> & avoid );
		void CreateMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel, double specialExtend );


		void CreateTriangleMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel );


		void CreateTetraMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel);
		void CreateTriangleSurfaceMesh(const char* filename, dtkID id);

		void CreateGuideWire(dtkID id, dtkPoints::Ptr points,  dtkID lastTipID, double segInterval, double tipSegInterval, const std::vector<double> & tipOriginAngle);
		void UpdateGuideWireHierarchy(dtkID id, dtkPoints::Ptr points);


		void CreateCollisionResponse( dtkID object1_id, dtkID object2_id, double strength);
		void UpdateCollisionResponse(dtkID object1_id, dtkID object2_id, double strength);


		dtkPhysMassSpring::Ptr GetMassSpring( dtkID id );

		dtkPhysMassSpring::Ptr GetTetraMassSpring( dtkID id );

		dtkCollisionDetectHierarchyKDOPS::Ptr GetCollisionDetectHierarchy( dtkID id);

		dtkStaticTriangleMesh::Ptr GetTriangleMesh( dtkID id );

		void ApplyExternalForce(dtkID id, dtkT3<double> force);
		void ApplyExternalTwist(dtkID id, double twist);

		

	private:
		controlCore( double clothDepth = 2.0 );
		

	public:
		const static size_t mPairOffset = 1000;
		// Collision Detect
		dtkCollisionDetectStage::Ptr mStage;
		collisionResponse::Ptr mCollisionDetectResponse;
		std::map< dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr > mCollisionDetectHierarchies;

		// Model
		std::map< dtkID, dtkPhysMassSpring::Ptr > mMassSprings;
		std::map< dtkID, dtkPhysTetraMassSpring::Ptr > mTetraMassSprings;
		std::map<dtkID, guideWire::Ptr > mGuideWireMassPoints;

		// Response
		std::map< dtkID, CollisionResponseSet > mCollisionDetectResponseSets;

		// Meshes
		std::map< dtkID, dtkStaticTriangleMesh::Ptr > mTriangleMeshes;
		std::map< dtkID, dtkStaticTetraMesh::Ptr > mTetraMeshes;

		double mClothDepth;
		int mCount;

	public:

		double mTimeslice;

		// collision detect research
		//---------------------
		bool responseTag;
		//--------------------

	private:
		vector<dtkID3> mAvoid;  // 血管的开口处的三角形面片的顶点ID 

		double mCollisionDetectTime;	// the time cost of collision detection with one time step
		double mUpdateGuideWireTime;	// the time cost of update the total guide wire with one time step
		double mRotateOnceTime;			// the time cost of rotate the total guide wire once
		dtkT3<double> mExternalForce;	

	};
}

#endif