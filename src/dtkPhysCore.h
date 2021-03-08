#ifndef DTK_PHYSCORE_H
#define DTK_PHYSCORE_H

#include <memory>
#include <boost/utility.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>

#include "dtkPhysTetraMassSpring.h"
#include "dtkPhysMassSpringThread.h"

#include "dtkCollisionDetectStage.h"
#include "dtkCollisionDetectHierarchyKDOPS.h"
#include "dtkPhysMassSpringCollisionResponse.h"
#include "dtkPhysMassSpringThreadCollisionResponse.h"

#include "dtkIntersectTest.h"
#include "dtkPhysParticleSystem.h"
#include "dtkStaticMeshEliminator.h"

#include "dtkPhysKnotPlanner.h"

#include <vector>
#include <map>
#include <set>

namespace dtk
{
	class dtkPhysCore : public boost::noncopyable
	{
	public:
		enum CollisionHierarchyType
		{
			SURFACE = 0,
			THREAD,
			INTERIOR,
			THREADHEAD
		};
		enum CollisionResponseType
		{
			NORMAL = 0,
			KNOTPLANNING,
			THREAD_SURFACE,
			INTERIOR_THREADHEAD
		};

		typedef struct  
		{
			dtkCollisionDetectStage::HierarchyPair hierarchy_pair;
			double strength;
			bool self;
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext );
			void* pContext;
			CollisionResponseType responseType;

		} CollisionResponseSet;

		typedef struct  
		{
			dtkPoints::Ptr pts;
			dtkPhysParticleSystem::Ptr particlesystem;
			dtkCollisionDetectStage::HierarchyPair hierarchy_pair;
			double viscosityCoef;
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext );
			void* pContext;
		} ObstacleSet;

		typedef struct  
		{
			dtkID dominate_ID;
			dtkPoints::Ptr dominate_pts;
			dtkID3 dominate_tri;
			dtkID dominate_triID;
			dtkDouble3 uvw;
			
			dtkID slave_ID;
			dtkPoints::Ptr slave_pts;
			dtkID slave_p;

			double slave_ratio;
		} AdherePointSet;

	public:
		typedef std::shared_ptr< dtkPhysCore > Ptr;

		static Ptr New( double clothDepth = 2.0 ) 
		{
			return Ptr( new dtkPhysCore( clothDepth ) );
		}

	public:
		~dtkPhysCore();

        void Update( double timeslice );

		void SetNumberOfThreads( size_t n );

		void CreateMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel, double specialExtend );

		void DestroyMassSpring( dtkID id );

		void CreateTriangleMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel );

		void DestroyTriangleMassSpring( dtkID id );

		void CreateTetraMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel, /*bool surface = true,*/
			dtkStaticMeshEliminator::MeshEliminatorResultsCallback callback = 0, void* pContext = 0 );// tetraMassSpringType: 1 for surface, 2 for body, 3 for all

		void DestroyTetraMassSpring( dtkID id );

		void CreateSutureThread( dtkID id, double length, dtkDouble3 firstPointPos, 
			dtkPhysMassSpringThread::Orientation orientation, double mass,
			double edgeStiff, double bendStiff,double torsionStiff, 
			double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp,
			double interval = 2.0, double radius = 0.8, double selfCollisionStrength = 20000.0 );

		void CreateCollisionResponse( dtkID object1_id, dtkID object2_id, double strength,
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext ) = 0, void* pContext = 0 );

		void CreateSutureResponse( dtkID object_id, dtkID thread_id,
			double strength, void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext ) = 0, void* pContext = 0 );

		void DestroyCollisionResponse( dtkID object1_id, dtkID object2_id );

		void DestroyCollisionResponse( dtkID object1_id, CollisionHierarchyType obj1_type, dtkID object2_id, CollisionHierarchyType obj2_type );

		size_t ConnectMassSpring( dtkID object1_id, dtkID object2_id, double range );

		void DisconnectMassSpring( dtkID object1_id, dtkID object2_id );

		size_t AdhereMassSpring( dtkID from_id, dtkID to_id, double range );

		void AdjustAdhereStatus();

		void DetachAllMassSpring();

		dtkPhysMassSpring::Ptr GetMassSpring( dtkID id );

		dtkPhysMassSpring::Ptr GetTetraMassSpring( dtkID id );

		dtkPhysMassSpringThread::Ptr GetMassSpringThread( dtkID id );

		dtkPoints::Ptr GetThreadPoints( dtkID id );

		dtkCollisionDetectHierarchyKDOPS::Ptr GetCollisionDetectHierarchy( dtkID id, CollisionHierarchyType type );

		dtkStaticTriangleMesh::Ptr GetTriangleMesh( dtkID id );

		dtkPhysMassSpringThreadCollisionResponse::Ptr GetThreadCollisionResponse();

		dtkPhysKnotPlanner::Ptr GetKnotPlanner( dtkID plannerID );

		dtkPoints::Ptr CreateCustomDetectTriangleArea( dtkID id, double half_width );

		void DestroyCustomDetectTriangleArea( dtkID id );

		void ExecuteCustomDetectTriangleArea( dtkID id, dtkID targetID, 
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext ), void* pContext = 0 );

		dtkPhysParticleSystem::Ptr CreateParticleSystem( dtkID id, double particleRadius, double particleMass, double particleLifetime );

		void DestroyParticleSystem( dtkID id );

		void CreateObstacleForParticleSystem( dtkID particlesystem_id, dtkID object_id, double viscosityCoef, 
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext ) = 0, void* pContext = 0  );

		void DestroyObstacleForParticleSystem( dtkID particlesystem_id, dtkID object_id );

		void EliminateTriangles( dtkID id, size_t originalFaceNum, std::vector< dtkID3 >& collisionFace, bool oldMethod = false );

		void RegisterDevice( dtkID deviceLabel );

		void LabelObjectAsDevice( dtkID objectID, dtkID deviceLabel );

		const dtkT3<double>& GetDeviceForceFeedback( dtkID deviceLabel );

    private:
		dtkPhysCore( double clothDepth = 2.0 );

		dtkID AllocateDetails( const std::multimap< dtkID, dtkID, std::greater<dtkID> >& sortedMap, size_t average );

		void Reallocate();

		void RebundleConnectedMassSpring();

		void CreateCollisionResponse( dtkID object1_id, CollisionHierarchyType obj1_type, dtkID object2_id, CollisionHierarchyType obj2_type, double strength,
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext ) = 0, void* pContext = 0 );

		void _Update_s( double timeslice );
		void _Update_mt( double timeslice );

	public:
		const static size_t mPairOffset = 1000;
		// Collision Detect
		dtkCollisionDetectStage::Ptr mStage;
		dtkPhysMassSpringCollisionResponse::Ptr mCollisionDetectResponse;
		dtkPhysMassSpringThreadCollisionResponse::Ptr mThreadCollisionDetectResponse;
		std::map< dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr > mCollisionDetectHierarchies;
		std::map< dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr > mThreadCollisionDetectHierarchies;
		std::map< dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr > mInteriorCollisionDetectHierarchies;			
		std::map< dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr > mThreadHeadCollisionDetectHierarchies;

		// Model
		std::map< dtkID, dtkPhysMassSpring::Ptr > mMassSprings;
		std::map< dtkID, dtkPhysTetraMassSpring::Ptr > mTetraMassSprings;
		std::map< dtkID, dtkPhysMassSpringThread::Ptr > mSutureThreads;

		// Response
		std::map< dtkID, CollisionResponseSet > mCollisionDetectResponseSets;
		std::map< dtkID, CollisionResponseSet > mInternalCollisionDetectResponseSets;
		std::map< dtkID, ObstacleSet > mObstacleSets;

		//Knot Planners
		std::map< dtkID, dtkPhysKnotPlanner::Ptr> mKnotPlanners;

		// Connect
		std::vector< dtkID2 > mConnectMap;
		std::set< dtkID > mConnectedMassSpring;

		std::map< dtkID, std::set< dtkID > > mConnectMasterMap;

		// Adhere
		std::vector< AdherePointSet > mAdherePointSets;
		std::map< dtkID, std::map< dtkID, size_t > > mAdhereCounts;

		// Meshes
		std::map< dtkID, dtkStaticTriangleMesh::Ptr > mTriangleMeshes;
		std::map< dtkID, dtkStaticTetraMesh::Ptr > mTetraMeshes;
		std::map< dtkID, dtkPoints::Ptr > mThreadPoints;

		// Particle System
		std::map< dtkID, dtkPhysParticleSystem::Ptr > mParticleSystems;

		// Eliminator
		dtkStaticMeshEliminator::Ptr mStaticMeshEliminator;

		// Device
		std::map< dtkID, std::vector< dtkID > >  mDeviceLabels;
		std::map< dtkID, dtkT3<double> > mDeviceForceFeedbacks;

		double mClothDepth;

	public:
		size_t mNumberOfThreads;

		bool mLive;

		double mTimeslice;

		boost::thread_group* mThreadGroup;
		boost::barrier* mEnterBarrier;
		boost::barrier* mExitBarrier;

		// Allocator
		std::vector< std::vector< std::vector< dtkID > > > mAllocator;
		dtkID mAllocatePosMassSpring;
		dtkID mAllocatePosPrimitive;
		dtkID mAllocatePosCollisionDetect;
		dtkID mAllocatePosInternalCollisionDetect;
	};
}

#endif
