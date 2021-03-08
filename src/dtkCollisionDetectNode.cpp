#include "dtkCollisionDetectNode.h"
#include "dtkCollisionDetectHierarchy.h"

#ifdef DTK_DEBUG
    #define DTKCOLLISIONDETECTNODE_DEBUG
#endif //DTK_DEBUG

#ifdef DTKCOLLISIONDETECTNODE_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{
    dtkCollisionDetectNode::dtkCollisionDetectNode( dtkCollisionDetectHierarchy* father )
    {
#ifdef DTKCOLLISIONDETECTNODE_DEBUG
        cout << "[dtkCollisionDetectNode::dtkCollisionDetectNode]" << endl;
        cout << "[/dtkCollisionDetectNode::dtkCollisionDetectNode]" << endl;
        cout << endl;
#endif
        mHierarchy = father;
        mLeaf = true;
        mLevel = 0;

        mMaxLevel = -1;
    }
    
    dtkCollisionDetectNode::~dtkCollisionDetectNode()
    {
#ifdef DTKCOLLISIONDETECTNODE_DEBUG
        cout << "[dtkCollisionDetectNode::~dtkCollisionDetectNode]" << endl;
        cout << "[/dtkCollisionDetectNode::~dtkCollisionDetectNode]" << endl;
        cout << endl;
#endif
        for( dtkID i = 0; i < GetNumOfChildren(); i++ )
        {
            delete mChildren[i];
        }
    }
    
    dtkCollisionDetectPrimitive* dtkCollisionDetectNode::GetPrimitive( dtkID id )
    {
        return mHierarchy->GetPrimitive( mPrimitiveIDs[id] );
    }
}

