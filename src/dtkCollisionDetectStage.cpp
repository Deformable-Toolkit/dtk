#include "dtkCollisionDetectStage.h"

#include <map>
using namespace boost;
using namespace std;

#ifdef DTK_DEBUG
    #define DTKCOLLISIONDETECTSTAGE_DEBUG
#endif //DTK_DEBUG

#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{
    std::vector< std::pair<size_t, size_t> > dtkCollisionDetectStage::mPossibleIntersectPairIDs;

    void thread_update( dtkID id, size_t numberOfThreads, 
            barrier* enter_barrier, barrier* exit_barrier, bool* running, 
            std::vector< dtkCollisionDetectHierarchy::Ptr >* hierarchies )
    {
        do {
            enter_barrier->wait();

            if( !(*running) )
                break;

            for( dtkID i = id; i < hierarchies->size(); i = i + numberOfThreads )
            {
#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
                cout << "update: " << id << " - " << i << endl;
#endif
                (*hierarchies)[i]->Update();
            }
            
            exit_barrier->wait();
        } while( true );
    }

    dtkCollisionDetectStage::dtkCollisionDetectStage()
    {
        mNumberOfThreads = 0;
        mThreadGroup = 0;
        mEnterBarrier = 0;
        mExitBarrier = 0;
    }
    
    dtkCollisionDetectStage::~dtkCollisionDetectStage()
    {
        if( mNumberOfThreads > 0 )
        {
            mLive = false;
            mEnterBarrier->wait();
            mThreadGroup->join_all();
            delete mThreadGroup;
            delete mEnterBarrier;
            delete mExitBarrier;
        }

    }
 
    const std::vector< dtkCollisionDetectStage::HierarchyPair >& dtkCollisionDetectStage::GetPossibleIntersectPairs()
    {
#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
        cout << "[dtkCollisionDetectStage::GetPossibleIntersectPairs]" << endl;
#endif

        std::vector< Box > boxes;
        std::map< Box::ID, dtkID > map;
        for( dtkID i = 0; i < GetNumberOfHierarchies(); i++ )
        {
            boxes.push_back( Box( mHierarchies[i]->GetBox() ) );

            map[ boxes[i].id() ] = i;
#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
            cout << boxes[i] << endl;
#endif
        }

        mPossibleIntersectPairIDs.clear();
        CGAL::box_self_intersection_d( boxes.begin(), boxes.end(), BoxIntersectCallBack );

        mPossibleIntersectPairs.clear();
        for( dtkID i = 0; i < mPossibleIntersectPairIDs.size(); i++ )
            mPossibleIntersectPairs.push_back( HierarchyPair( mHierarchies[ map[mPossibleIntersectPairIDs[i].first] ],
                        mHierarchies[ map[mPossibleIntersectPairIDs[i].second] ] ) );

#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
        cout << "[/dtkCollisionDetectStage::GetPossibleIntersectPairs]" << endl;
        cout << endl;
#endif
        return mPossibleIntersectPairs;
    }
        
    void dtkCollisionDetectStage::Update()
    {
#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
        cout << "[dtkCollisionDetectStage::Update]" << endl;
#endif
        mIntersectResults.clear();

        if( mNumberOfThreads > 0 )
            _Update_mt();
        else
            _Update_s();
#ifdef DTKCOLLISIONDETECTSTAGE_DEBUG
        cout << "[/dtkCollisionDetectStage::Update]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectStage::_Update_s()
    {
        for( dtkID i = 0; i < GetNumberOfHierarchies(); i++ )
            mHierarchies[i]->Update();
    }
        
    void dtkCollisionDetectStage::_Update_mt()
    {   
        for( dtkID i = 0; i < GetNumberOfHierarchies(); i++ )
            mHierarchies[i]->Update();

        // multi_thread ?
        /*

        mEnterBarrier->wait();
        mExitBarrier->wait();
        */

    }
        
    void dtkCollisionDetectStage::BoxIntersectCallBack( const Box& a, const Box& b )
    {
        mPossibleIntersectPairIDs.push_back( std::pair<size_t, size_t>(a.id(), b.id()) );
    }
        
    void dtkCollisionDetectStage::DoIntersect( HierarchyPair pair, 
            vector<IntersectResult::Ptr>& intersectResults,
            bool self, bool ignore_extend )
    {
        TraverseHierarchy( pair.first->GetRoot(), pair.second->GetRoot(), intersectResults, self, ignore_extend );
    }
        
    void dtkCollisionDetectStage::TraverseHierarchy( 
            dtkCollisionDetectNode* node_1, dtkCollisionDetectNode* node_2, 
            vector<IntersectResult::Ptr>& intersectResults,
            bool self, bool ignore_extend )
    {
        if( CDBasic::DoIntersect( node_1, node_2 ) )//粗相交检测
        {
            if( node_1->IsLeaf() )
            {
                if( node_2->IsLeaf() )
                {//叶子与叶子相交测试
                    IntersectResult::Ptr result;
                    for( dtkID i = 0; i < node_1->GetNumOfPrimitives(); i++ )
                    {
                        for( dtkID j = 0; j < node_2->GetNumOfPrimitives(); j++ )
                        {
                            if( CDBasic::DoIntersect( node_1->GetPrimitive( i ), node_2->GetPrimitive( j ), result, self, ignore_extend ) )
                            {
                                intersectResults.push_back( result );
                            }
                        }
                    }
                }
                else
                {//叶子与非叶结点相交测试
                    for( dtkID i = 0; i < node_2->GetNumOfChildren(); i++ )
                        TraverseHierarchy( node_1, node_2->GetChild(i), intersectResults, self, ignore_extend );
                }
            }
            else
            {
                if( node_2->IsLeaf() || node_1->GetLevel() > node_2->GetLevel() )
                {//非叶子结点与叶结点（或两个非叶结点相交测试）相交测试
                    for( dtkID i = 0; i < node_1->GetNumOfChildren(); i++ )
                        TraverseHierarchy( node_1->GetChild(i), node_2, intersectResults, self, ignore_extend );
                }
                else
                {//两个非叶结点相交测试
                    for( dtkID i = 0; i < node_2->GetNumOfChildren(); i++ )
                        TraverseHierarchy( node_1, node_2->GetChild(i), intersectResults, self, ignore_extend );
                }
            }
        }
    }
        
    void dtkCollisionDetectStage::SelfIntersect( dtkCollisionDetectHierarchy::Ptr hierarchy )
    {
        dtkCollisionDetectNode* root = hierarchy->GetRoot();
        TraverseHierarchy( root, root, mIntersectResults, true );
    }

    void dtkCollisionDetectStage::AllIntersect()
    {
        const std::vector< HierarchyPair >& pairs = GetPossibleIntersectPairs();
        for( dtkID i = 0; i < pairs.size(); i++ )
        {
            DoIntersect( pairs[i], mIntersectResults, false, false );
        }
    }
        
    void dtkCollisionDetectStage::SetNumberOfThreads( size_t n )
    {
        if( n <= 0 )
            return;

        mNumberOfThreads = n;
        mLive = true;
        mThreadGroup = new thread_group();
        mEnterBarrier = new barrier( mNumberOfThreads + 1 );
        mExitBarrier = new barrier( mNumberOfThreads + 1 );
        for( dtkID i = 0; i < mNumberOfThreads; i++ )
        {
            mThreadGroup->add_thread( new boost::thread( thread_update, i, mNumberOfThreads, 
                        mEnterBarrier, mExitBarrier, &mLive, &mHierarchies ) );
        }
    }
}

