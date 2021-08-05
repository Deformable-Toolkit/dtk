#include "dtkCollisionDetectHierarchyKDOPS.h"

#include <boost/thread/thread.hpp>

#ifdef DTK_DEBUG
    #define DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
#endif //DTK_DEBUG

#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
#include <iostream>
using namespace std;
#endif

using namespace boost;

namespace dtk
{
    dtkCollisionDetectHierarchyKDOPS::dtkCollisionDetectHierarchyKDOPS( size_t half_k )
        : mHalfK( half_k )
    {
#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "[dtkCollisionDetectHierarchyKDOPS::dtkCollisionDetectHierarchyKDOPS]" << endl;
        cout << "[/dtkCollisionDetectHierarchyKDOPS::dtkCollisionDetectHierarchyKDOPS]" << endl;
        cout << endl;
#endif
    }
    
    dtkCollisionDetectHierarchyKDOPS::~dtkCollisionDetectHierarchyKDOPS()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "[dtkCollisionDetectHierarchyKDOPS::~dtkCollisionDetectHierarchyKDOPS]" << endl;
        cout << "[/dtkCollisionDetectHierarchyKDOPS::~dtkCollisionDetectHierarchyKDOPS]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectHierarchyKDOPS::Build()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "[dtkCollisionDetectHierarchyKDOPS::Build]" << endl;
#endif
        mRoot = new dtkCollisionDetectNodeKDOPS( this, mHalfK );
		mRoot->SetMaxLevel( mMaxLevel );
        AddNode( mRoot );

        for( dtkID i = 0; i < mPrimitives.size(); i++ )
            mRoot->AddPrimitive(i);

        mRoot->Split();
#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "[/dtkCollisionDetectHierarchyKDOPS::Build]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectHierarchyKDOPS::Update()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "[dtkCollisionDetectHierarchyKDOPS::Update]" << endl;
#endif
        UpdateAllPrimitives(); //更新所有图元

        size_t numOfNodes = mNodes.size();
        for( int i = numOfNodes - 1; i > -1; i-- )
            static_cast< dtkCollisionDetectNodeKDOPS* >( mNodes[i] )->Update();

        const GK::KDOP& kdop = ( ( dtkCollisionDetectNodeKDOPS* )mRoot )->GetKDOP();
        
        mBox = GK::BBox3( kdop[0], kdop[2], kdop[4],
                kdop[1], kdop[3], kdop[5] );

#ifdef DTKCOLLISIONDETECTHIERARCHYKDOPS_DEBUG
        cout << "Box[ " << mBox << " ]" << endl;
        cout << "[/dtkCollisionDetectHierarchyKDOPS::Update]" << endl;
        cout << endl;
#endif
    }

    void dtkCollisionDetectHierarchyKDOPS::Rebuild()
    {
        if( mRoot != 0 )
            delete mRoot;

        mNodes.clear();

        Build();
    }
}

