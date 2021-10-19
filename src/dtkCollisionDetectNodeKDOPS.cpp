#include "dtkCollisionDetectNodeKDOPS.h"
#include "dtkCollisionDetectHierarchyKDOPS.h"

#ifdef DTK_DEBUG
    #define DTKCOLLISIONDETECTNODEKDOPS_DEBUG
#endif //DTK_DEBUG

#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{
    void thread_update_node( dtkCollisionDetectNode* node )
    {
        node->Update();
    }

    dtkCollisionDetectNodeKDOPS::dtkCollisionDetectNodeKDOPS( dtkCollisionDetectHierarchy* father, size_t half_k )
        : dtkCollisionDetectNode( father ), mKDOP( half_k )
    {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[dtkCollisionDetectNodeKDOPS::dtkCollisionDetectNodeKDOPS]" << endl;
#endif
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[/dtkCollisionDetectNodeKDOPS::dtkCollisionDetectNodeKDOPS]" << endl;
        cout << endl;
#endif
    }
    
    dtkCollisionDetectNodeKDOPS::~dtkCollisionDetectNodeKDOPS()
    {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[dtkCollisionDetectNodeKDOPS::~dtkCollisionDetectNodeKDOPS]" << endl;
        cout << "[/dtkCollisionDetectNodeKDOPS::~dtkCollisionDetectNodeKDOPS]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectNodeKDOPS::Split()
    {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[dtkCollisionDetectNodeKDOPS::Split]" << endl;
#endif
        if( GetNumOfPrimitives() < 2 )
        {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
            cout << "No enough primitives." << endl;
            cout << endl;
#endif
            return;
        }

        if( mMaxLevel > 0 && mLevel >= mMaxLevel )
        {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
            cout << "Exceed max level." << endl;
            cout << endl;
#endif
            return;
        }

#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "Creating two children..." << endl; 
#endif

        // Creating two children
        dtkCollisionDetectNodeKDOPS* leftChild = new dtkCollisionDetectNodeKDOPS( mHierarchy, mKDOP.mHalfK );
        leftChild->mLevel = mLevel + 1;
        leftChild->mMaxLevel = mMaxLevel;
        dtkCollisionDetectNodeKDOPS* rightChild = new dtkCollisionDetectNodeKDOPS( mHierarchy, mKDOP.mHalfK );
        rightChild->mLevel = mLevel + 1;
        rightChild->mMaxLevel = mMaxLevel;

        mChildren.push_back( leftChild );
        mChildren.push_back( rightChild );

        SplitRule();

        //检测划分是否成功。
        for( dtkID i = 0; i < GetNumOfChildren(); i++ )
        {
            if( mChildren[i]->GetNumOfPrimitives() < 1)
            {
                for( dtkID j = 0; j < GetNumOfChildren(); j++ )
                {
                    delete mChildren[j];
                }
                mChildren.clear();

#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
                cout << "Split failed." << endl;
                cout << "[/dtkCollisionDetectNodeKDOPS::Split]" << endl;
                cout << endl;
#endif
                return;
            }
        }

#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "... Succeed." << endl;
#endif
        mHierarchy->AddNode( leftChild );
        mHierarchy->AddNode( rightChild );
        // recursive split
        leftChild->Split();
        rightChild->Split();
        
        mLeaf = false;
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[/dtkCollisionDetectNodeKDOPS::Split]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectNodeKDOPS::SplitRule()
    {
        // split primitive
        // choose split axis
        double mean[] = { 0.0, 0.0, 0.0 };
        double variance[] = { 0.0, 0.0, 0.0 };

        size_t numOfPrimitives = GetNumOfPrimitives();
        for( dtkID i = 0; i < numOfPrimitives; i++ )
        {
            const GK::Point3& centroid = mHierarchy->GetPrimitive( mPrimitiveIDs[i] )->GetCentroid();

            for( dtkID j = 0; j < 3; j++ )
                mean[j] += centroid[j]; 
        }
        for( dtkID j = 0; j < 3; j++ )
            mean[j] /= numOfPrimitives; 

        for( dtkID i = 0; i < numOfPrimitives; i++ )
        {
            const GK::Point3& centroid = mHierarchy->GetPrimitive( mPrimitiveIDs[i] )->GetCentroid();

            for( dtkID j = 0; j < 3; j++ )
                variance[j] += pow( centroid[j] - mean[j], 2 ); 
        }

        dtkID chooseAxis;
        if( variance[0] > variance[1] )
        {
            if( variance[0] > variance[2] )
                chooseAxis = 0;
            else
                chooseAxis = 2;
        }
        else
        {
            if( variance[1] > variance[2] )
                chooseAxis = 1;
            else
                chooseAxis = 2;
        }

        // choose split point
        double splitPoint = mean[chooseAxis];

        // split
        assert( mChildren.size() == 2 );
        for( dtkID i = 0; i < numOfPrimitives; i++ )
        {
            const GK::Point3& centroid = mHierarchy->GetPrimitive( mPrimitiveIDs[i] )->GetCentroid();

            if( centroid[chooseAxis] < splitPoint )
                mChildren[0]->AddPrimitive( mPrimitiveIDs[i] );
            else
                mChildren[1]->AddPrimitive( mPrimitiveIDs[i] );
        }
    }

    void dtkCollisionDetectNodeKDOPS::Update()
    {
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << "[dtkCollisionDetectNodeKDOPS::Update]" << endl;
#endif
        if( mLeaf )
        { //叶节点
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
            cout << "Encounter Leaf." << endl;
#endif
            /*
            dtkCollisionDetectPrimitive* primitive = mHierarchy->GetPrimitive( mPrimitiveIDs[0] );
            const GK::Point3& first_point = primitive->GetPoint( 0 );
            GK::Vector3 first_vec( mHierarchy->GetOrigin(), first_point );
            for( dtkID k = 0; k < mKDOP.mHalfK; k++ )
            {
                GK::Float extend = GK::DotProduct( first_vec, GK::KDOP::mPredefinedAxis[k] );

                mKDOP.mIntervals[k][0] = mKDOP.mIntervals[k][1] = extend;
            }
            */
            for( dtkID k = 0; k < mKDOP.mHalfK; k++ )
            {//包围盒每一维设置上下限
                mKDOP.mIntervals[k][0] = dtkDoubleMax;
                mKDOP.mIntervals[k][1] = dtkDoubleMin;
            }

            size_t numOfPrimitives = GetNumOfPrimitives();
            const GK::Point3& origin = mHierarchy->GetOrigin();
            for( dtkID i = 0; i < numOfPrimitives; i++ )
            {
                dtkCollisionDetectPrimitive* primitive = mHierarchy->GetPrimitive( mPrimitiveIDs[i] );
                size_t numOfPoints = primitive->GetNumberOfPoints();
                for( dtkID j = 0; j < numOfPoints; j++ )
                {
                    const GK::Point3& point = primitive->GetPoint( j );
                    GK::Vector3 vec( origin, point );
                    if( mKDOP.mHalfK == 9)
                    {
                        for( dtkID k = 0; k < 3; k++ )
                        {
                            GK::Float extend = GK::DotProduct( vec, GK::KDOP::mPredefinedAxis[k] );

							mKDOP.mIntervals[k].Extend( extend );
							//mKDOP.mIntervals[k].Extend( extend + primitive->GetExtend() );
							//mKDOP.mIntervals[k].Extend( extend - primitive->GetExtend() );
                        }
                        for( dtkID k = 3; k < 9; k++ )
                        {
                            GK::Float extend = GK::DotProduct( vec, GK::KDOP::mPredefinedAxis[k + 4] );

							mKDOP.mIntervals[k].Extend( extend );
							//mKDOP.mIntervals[k].Extend( extend + primitive->GetExtend() );
							//mKDOP.mIntervals[k].Extend( extend - primitive->GetExtend() );
                        }
                    }
                    else
                    {
                        for( dtkID k = 0; k < mKDOP.mHalfK; k++ )
                        {
                            GK::Float extend = GK::DotProduct( vec, GK::KDOP::mPredefinedAxis[k] );

							mKDOP.mIntervals[k].Extend( extend + primitive->GetExtend() );
							mKDOP.mIntervals[k].Extend( extend - primitive->GetExtend() );
                        }
                    }
                }
            }
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
            cout << "Leaf-KDOP complete." << endl;
#endif

        }
        else
        {//非叶节点
            GK::Merge( mKDOP, ( ( dtkCollisionDetectNodeKDOPS* )mChildren[0] )->mKDOP, 
                    ( ( dtkCollisionDetectNodeKDOPS* )mChildren[1] )->mKDOP );
        }
#ifdef DTKCOLLISIONDETECTNODEKDOPS_DEBUG
        cout << mKDOP << endl;
        cout << "[/dtkCollisionDetectNodeKDOPS::Update]" << endl;
        cout << endl;
#endif
    }
}

