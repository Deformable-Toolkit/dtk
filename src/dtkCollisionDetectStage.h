#ifndef DTK_COLLISIONDETECTSTAGE_H
#define DTK_COLLISIONDETECTSTAGE_H

#include <memory>
#include <boost/utility.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>

#include <vector>
#include <CGAL/box_intersection_d.h>

#include "dtkConfig.h"
#include "dtkIDTypes.h"

#include "dtkCollisionDetectHierarchy.h"

namespace dtk
{   
    /**
    * @class <dtkCollisionDetectStage> 
    * @brief 分阶段进行相交测试
    * @author <>
    * @note
    * 分阶段进行相交测试
    */
	class dtkCollisionDetectStage: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkCollisionDetectStage> Ptr;

        typedef dtkCollisionDetectBasic::IntersectResult IntersectResult;

        typedef std::pair< dtkCollisionDetectHierarchy::Ptr, dtkCollisionDetectHierarchy::Ptr > HierarchyPair;

        typedef CGAL::Box_intersection_d::Box_d< double, 3 >    Box;

        static dtkCollisionDetectStage::Ptr New()
		{
			return dtkCollisionDetectStage::Ptr(new dtkCollisionDetectStage());
		}

	public:
		~dtkCollisionDetectStage();

        /**
		* @brief		两个检测树层内的图元分别进行相交测试
		* @param[in]	pair : 碰撞检测树层次dtkCollisionDetectHierarchy对
		* @param[in]	intersectResults : 碰撞检测树相交结果集
		* @param[in]	self : 是否进行自相交测试
		* @param[in]	ignore_extend : 是否有碰撞间隔测试
		* @note		
		*/
        void DoIntersect( HierarchyPair pair, std::vector<IntersectResult::Ptr>& intersectResults,
                bool self = false, bool ignore_extend = false );
        
        /**
		* @brief		相交递归调用两个检测树节点进行相交测试
		* @param[in]	node_1 : 碰撞检测树节点1
        * @param[in]	node_2 : 碰撞检测树节点2
		* @param[in]	intersectResults : 碰撞检测树相交结果集
		* @param[in]	self : 是否进行自相交测试
		* @param[in]	ignore_extend : 是否有碰撞间隔测试
		* @note		
		*/
        void TraverseHierarchy( dtkCollisionDetectNode* node_1, dtkCollisionDetectNode* node_2, 
                std::vector<IntersectResult::Ptr>& intersectResults,
                bool self = false, bool ignore_extend = false );

        /**
		* @brief		同一个检测树层进行自相交测试
		* @param[in]	hierarchy : 碰撞检测树层
		* @note	从每一层的根节点开始递归检测
		*/
        void SelfIntersect( dtkCollisionDetectHierarchy::Ptr hierarchy );
        
        /**
		* @brief		所有图元进行相交测试
		* @note	先计算可能相交的层，再对每一个层次对开始从根节点递归检测
		*/
        void AllIntersect();

        /**
		* @brief 根据包围盒计算可能相交的层次对
		* @note	
		*/
        const std::vector< HierarchyPair >& GetPossibleIntersectPairs();
        
        /**
		* @brief 更新每一层的图元包围盒
		* @note 
        * 更新每一层的图元包围盒， 根据mNumberOfThreads决定使用多线程还是单线程。	
		*/
        void Update();

		inline size_t GetNumberOfHierarchies() const
        {
            return mHierarchies.size();
        }

        inline void AddHierarchy( dtkCollisionDetectHierarchy::Ptr hierarchy )
        {
            mHierarchies.push_back( hierarchy );
		}

		inline void RemoveHierarchy( dtkCollisionDetectHierarchy::Ptr hierarchy )
		{
			for( dtkID i = 0; i < mHierarchies.size(); i++ )
			{
				if( mHierarchies[i] == hierarchy )
					mHierarchies.erase( mHierarchies.begin() + i );
			}
		}

        dtkCollisionDetectHierarchy::Ptr GetHierarchy( dtkID i )
        {
            return mHierarchies[i];
        }

        void SetNumberOfThreads( size_t n );

        inline size_t GetNumberOfIntersectResults()
        {
            return mIntersectResults.size();
        }

        inline IntersectResult::Ptr GetIntersectResult( dtkID i )
        {
            assert( i < mIntersectResults.size() );
            return mIntersectResults[i];
        }

        inline void ClearIntersectResult()
        {
            mIntersectResults.clear();
        }
        
    private:
        dtkCollisionDetectStage();
        
        //所有层
        std::vector< dtkCollisionDetectHierarchy::Ptr > mHierarchies; /**< 所有层 */

        //可能相交的层对
        std::vector< HierarchyPair > mPossibleIntersectPairs; /**< only for temp-storage 可能相交的层对 */

        //包围盒相交回调函数
        static void BoxIntersectCallBack( const Box& a, const Box& b ); /**< 包围盒相交回调函数 */
        
        //可能相交的层的ID对
        static std::vector< std::pair<size_t, size_t> > mPossibleIntersectPairIDs; /**< only for temp-storage. 可能相交的层的ID对. */

    private:

        /**
		* @brief 单线程更新每一层的图元包围盒
		* @note 
        * 更新每一层的图元包围盒， 根据mNumberOfThreads决定使用多线程还是单线程。	
		*/
        void _Update_s(); 

        /**
		* @brief 多线程更新每一层的图元包围盒
		* @note 
        * 更新每一层的图元包围盒， 根据mNumberOfThreads决定使用多线程还是单线程。	
		*/
        void _Update_mt();

    private:
        size_t mNumberOfThreads; /**< 多线程树 */
        bool mLive;

        std::vector<IntersectResult::Ptr> mIntersectResults;

        boost::thread_group* mThreadGroup;
        boost::barrier* mEnterBarrier;
        boost::barrier* mExitBarrier;
	};

    inline std::ostream& operator<<(std::ostream& stream, const dtkCollisionDetectStage::Box& box)
    {
        stream << "dtkCollisionDetectStage::Box{ ";
        for( int i = 0; i < 3; i++ )
            stream << box.min_coord( i ) << " ";
        for( int i = 0; i < 3; i++ )
            stream << box.max_coord( i ) << " ";
        stream << "}";
		
        return stream;
    }
}

#endif //DTK_COLLISIONDETECTSTAGE_H
