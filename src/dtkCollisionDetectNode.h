#ifndef DTK_COLLISIONDETECTNODE_H
#define DTK_COLLISIONDETECTNODE_H

#include <memory>
#include <boost/utility.hpp>

#include <vector>

#include "dtkConfig.h"
#include "dtkIDTypes.h"
#include "dtkCollisionDetectPrimitive.h"

namespace dtk
{
    class dtkCollisionDetectHierarchy;

	class dtkCollisionDetectNode
	{
	public:
        dtkCollisionDetectNode( dtkCollisionDetectHierarchy* father );

		virtual ~dtkCollisionDetectNode();

        virtual void Split() = 0;

        virtual void Update() = 0;

        inline bool IsLeaf() const
        {
            return mLeaf;
        }

        inline void SetLeaf( bool leaf )
        {
            mLeaf = leaf;
        }

        inline void AddPrimitive( dtkID id )
        {
            mPrimitiveIDs.push_back( id );
        }

		inline void AddPrimitive( dtkCollisionDetectPrimitive* primitive )
		{
			mPrimitiveIDs.push_back( primitive->mLocalID );
		}

		inline void DeletePrimitive( dtkCollisionDetectPrimitive* primitive )
		{
			primitive->mActive = false;
			std::vector< dtkID >::iterator it;
			it = std::find( mPrimitiveIDs.begin(), mPrimitiveIDs.end(), primitive->mLocalID );
			assert( it != mPrimitiveIDs.end() );
			mPrimitiveIDs.erase( it );
		}

        inline size_t GetNumOfPrimitives() const
        {
            return mPrimitiveIDs.size();
        }

        dtkCollisionDetectPrimitive* GetPrimitive( dtkID id );

        inline size_t GetNumOfChildren() const
        {
            return mChildren.size();
        }

        inline size_t GetLevel() const
        {
            return mLevel;
        }

        inline void SetMaxLevel( size_t level )
        {
            mMaxLevel = level;
        }

        inline dtkCollisionDetectNode* GetChild( size_t id )
        {
            return mChildren[id];
        }

    protected:
        dtkCollisionDetectHierarchy* mHierarchy;
        std::vector< dtkID > mPrimitiveIDs;
        std::vector< dtkCollisionDetectNode* > mChildren;
        bool mLeaf;
        size_t mLevel;

        size_t mMaxLevel;
	};
}

//#include "impl/dtkCollisionDetectNodeImpl.hpp"

#endif //DTK_COLLISIONDETECTNODE_H
