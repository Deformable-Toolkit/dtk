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
			SURFACE = 0,  // 面 
			THREAD, //  线
			INTERIOR, // 内部
			THREADHEAD // 线头
		};
		enum CollisionResponseType
		{
			NORMAL = 0, //
 			KNOTPLANNING, 
			THREAD_SURFACE, /**< 线与面 */
 			INTERIOR_THREADHEAD /**< 内部与线头 */
		};

		/**
		 * @brief 碰撞响应集
		 */ 
		typedef struct  
		{
			dtkCollisionDetectStage::HierarchyPair hierarchy_pair;
			double strength;
			bool self;
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext );
			void* pContext;
			CollisionResponseType responseType;

		} CollisionResponseSet;  

		/**
		 * @brief 障碍集
		 */ 
		typedef struct  
		{
			dtkPoints::Ptr pts;
			dtkPhysParticleSystem::Ptr particlesystem;
			dtkCollisionDetectStage::HierarchyPair hierarchy_pair;
			double viscosityCoef;
			void (*custom_handle)( const std::vector<dtkIntersectTest::IntersectResult::Ptr>& intersectResults, void* pContext );
			void* pContext;
		} ObstacleSet; //障碍集

		/**
		 * @brief 固定点集
		 */ 
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
		} AdherePointSet; //固定点集

	public:
		typedef std::shared_ptr< dtkPhysCore > Ptr;

		static Ptr New( double clothDepth = 2.0 ) 
		{
			return Ptr( new dtkPhysCore( clothDepth ) );
		}

	public:
		~dtkPhysCore();

		/**
		* @brief		碰撞检测树更新，力更新，包围盒更新，点更新
		* @param[in]	timeslice : 更新时间间隔
		* @note	 碰撞检测树更新，力更新，包围盒更新，点更新
		*/					
        void Update( double timeslice );

		/**
		* @brief 新建更新多线程
		* @param[in]	n : 更新线程数
		* @note	 新建更新多线程
		*/
		void SetNumberOfThreads( size_t n );

		/**
		* @brief 从文件获取点集新建弹簧图元
		* @param[in]	filename : 输入文件名
		* @param[in]	id       : 图元id
		* @param[in]	point_mass       : 点的质量
		* @param[in]	stiffness       : 弹簧弹性系数
		* @param[in]	damp       : 弹簧阻尼
		* @param[in]	pointDamp       : 点阻尼
		* @param[in]	pointResistence : 点阻力系数
		* @param[in]	gravityAccel       : 重力加速度
		* @param[in]	specialExtend       : 碰撞检测间隔
		* @note	 从文件获取点集新建弹簧图元
		*/
		void CreateMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel, double specialExtend );
		void DestroyMassSpring( dtkID id );

		/**
		* @brief 从文件获取点集新建三角网格图元
		* @param[in]	filename : 输入文件名
		* @param[in]	id       : 图元id
		* @param[in]	point_mass       : 点的质量
		* @param[in]	stiffness       : 弹簧弹性系数
		* @param[in]	damp       : 弹簧阻尼
		* @param[in]	pointDamp       : 点阻尼
		* @param[in]	pointResistence : 点阻力系数
		* @param[in]	gravityAccel       : 重力加速度
		* @note	 从文件获取点集新建弹簧图元
		*/
		void CreateTriangleMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel );
		void DestroyTriangleMassSpring( dtkID id );

		/**
		* @brief 从文件新建四面体网格图元
		* @param[in]	filename : 输入文件名
		* @param[in]	id       : 图元id
		* @param[in]	point_mass       : 点的质量
		* @param[in]	stiffness       : 弹簧弹性系数
		* @param[in]	damp       : 弹簧阻尼
		* @param[in]	pointDamp       : 点阻尼
		* @param[in]	pointResistence : 点阻力系数
		* @param[in]	gravityAccel       : 重力加速度
		* @note	 从文件获取点集新建弹簧图元
		*/
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


		/**
		* @brief 链接弹簧
		* @param[in]	object1_id  : 对象id1
		* @param[in]	object2_id  ：对象id2
		* @param[in]	range       : 距离范围
		* @return		
		*	在range范围内连接的弹簧数 \n
		* @note	 链接弹簧
		*/
		size_t ConnectMassSpring( dtkID object1_id, dtkID object2_id, double range );
		void DisconnectMassSpring( dtkID object1_id, dtkID object2_id );

		size_t AdhereMassSpring( dtkID from_id, dtkID to_id, double range );

		void AdjustAdhereStatus();

		/**
		* @brief 断开所有弹簧	
		* @note	 断开所有弹簧	
		*/
		void DetachAllMassSpring();

		dtkPhysMassSpring::Ptr GetMassSpring( dtkID id );

		dtkPhysMassSpring::Ptr GetTetraMassSpring( dtkID id );

		dtkPhysMassSpringThread::Ptr GetMassSpringThread( dtkID id );

		dtkPoints::Ptr GetThreadPoints( dtkID id );

		dtkCollisionDetectHierarchyKDOPS::Ptr GetCollisionDetectHierarchy( dtkID id, CollisionHierarchyType type );

		dtkStaticTriangleMesh::Ptr GetTriangleMesh( dtkID id );

		dtkPhysMassSpringThreadCollisionResponse::Ptr GetThreadCollisionResponse();

		dtkPhysKnotPlanner::Ptr GetKnotPlanner( dtkID plannerID );

		//自定义检测
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
		std::map< dtkID, CollisionResponseSet > mCollisionDetectResponseSets; /**< 碰撞响应集 */
		std::map< dtkID, CollisionResponseSet > mInternalCollisionDetectResponseSets; /**< 内部碰撞响应集 */
		std::map< dtkID, ObstacleSet > mObstacleSets; /**< 障碍集 */

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

		double mClothDepth; /**< 碰撞检测间隔 */

	public:
		size_t mNumberOfThreads; /**< 构建多线程数 */

		bool mLive;  /**< 核是否存活， 析构时置0,杀死循环检测更新线程 */

		double mTimeslice; /**< 更新的时间间隔 */

		boost::thread_group* mThreadGroup;
		boost::barrier* mEnterBarrier;
		boost::barrier* mExitBarrier;

		// Allocator
		std::vector< std::vector< std::vector< dtkID > > > mAllocator; /**< different id group to different threads */
		dtkID mAllocatePosMassSpring; /**< 已分配质量弹簧ID的范围 */
		dtkID mAllocatePosPrimitive; /**< 已分配图元ID的范围 */
		dtkID mAllocatePosCollisionDetect; /** 已分配碰撞检测的ID的范围 */
		dtkID mAllocatePosInternalCollisionDetect; /**< 已分配内部碰撞检测的ID范围 */
	};
}

#endif
