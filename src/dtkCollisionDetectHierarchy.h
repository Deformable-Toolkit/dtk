#ifndef DTK_COLLISIONDETECTHIERARCHY_H
#define DTK_COLLISIONDETECTHIERARCHY_H

#include <memory>
#include <boost/utility.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>

#include "dtkConfig.h"
#include "dtkIDTypes.h"

#include "dtkCollisionDetectBasic.h"
#include "dtkStaticTriangleMesh.h"
#include "dtkStaticTetraMesh.h"

namespace dtk
{
	class dtkCollisionDetectHierarchy: public boost::noncopyable
	{
	public:

        enum InsertOption
        {
            SURFACE,
            INTERIOR
        };
		typedef std::shared_ptr<dtkCollisionDetectHierarchy> Ptr;

		typedef dtkCollisionDetectPrimitive Primitive;

	public:
		virtual ~dtkCollisionDetectHierarchy();

        virtual void Build() = 0;

        virtual void Rebuild() = 0;

        virtual void Update() = 0;
        
        void UpdateAllPrimitives();
        
        Primitive* InsertTriangle( dtkPoints::Ptr pts, dtkID3 ids );

        Primitive* InsertSegment( dtkPoints::Ptr pts, dtkID2 ids );

		Primitive* InsertSphere( dtkPoints::Ptr pts, dtkID id );

        void InsertTriangleMesh( dtkStaticTriangleMesh::Ptr triMesh, dtkID majorID, double extend = 0 );

        void InsertTetraMesh( dtkStaticTetraMesh::Ptr tetraMesh, dtkID majorID, InsertOption opt, double extend);

        void SetMaxLevel( size_t maxLevel );

		void AutoSetMaxLevel();

        void SetNumberOfThreads( size_t n );

        inline Primitive* GetPrimitive( dtkID id )
        {
            assert( id < mPrimitives.size() );

            return mPrimitives[id];
        }
        
        inline size_t GetNumberOfPrimitives() const
        {
            return mPrimitives.size();
        }

        size_t GetNumberOfIntersectedPrimitives() const;

        inline const GK::BBox3& GetBox() const
        {
            return mBox;
        }

        inline const GK::Point3& GetOrigin() const
        {
            return mOrigin;
        }

        inline dtkCollisionDetectNode* GetRoot()
        {
            return mRoot;
        }

        size_t GetNumberOfNodes() const
        {
            return mNodes.size();
        }

        inline void AddNode( dtkCollisionDetectNode* node )
        {
            mNodes.push_back( node );
        }

        inline dtkCollisionDetectNode* GetNode( dtkID i )
        {
            return mNodes[i];
        }

    protected:
		dtkCollisionDetectHierarchy();

		void AddPrimitive( Primitive* primitive );

        dtkCollisionDetectNode* mRoot; /**< 碰撞检测树根结点 */
        
        std::vector< dtkCollisionDetectNode* > mNodes;  /**< 当前层结点集 */

        std::vector< Primitive* > mPrimitives; /**< 图元集 */

        GK::BBox3 mBox;  /**< AABB包围盒 */

        GK::Point3 mOrigin;  /**< 原点 */

        bool mLive;  /**< 碰撞检测更新线程是否运行 */

		size_t mMaxLevel; /**< 最大层数 */

    private:
        void _UpdateAllPrimitives_s();  /**< 单线程更新图元 */

        void _UpdateAllPrimitives_mt(); /**< 多线程更新图元 */

    private:
        size_t mNumberOfThreads;

        boost::thread_group* mThreadGroup;
        boost::barrier* mEnterBarrier;
        boost::barrier* mExitBarrier;
	};
}

#endif //DTK_COLLISIONDETECTHIERARCHY_H
