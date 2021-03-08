#ifndef DTK_COLLISIONDETECTNODEKDOPS_H
#define DTK_COLLISIONDETECTNODEKDOPS_H

#include "dtkConfig.h"
#include "dtkCollisionDetectNode.h"

#include "dtkGraphicsKernel.h"

namespace dtk
{
    class dtkCollisionDetectHierarchyKDOPS;

	class dtkCollisionDetectNodeKDOPS : public dtkCollisionDetectNode
	{
	public:
        dtkCollisionDetectNodeKDOPS( dtkCollisionDetectHierarchy* father, size_t half_k );
        
		~dtkCollisionDetectNodeKDOPS();

        void Split();
        
        void Update();

        void SplitRule();

        inline const GK::KDOP& GetKDOP() const
        {
            return mKDOP;
        }

    private:
        GK::KDOP mKDOP;
	};
}

#endif //DTK_COLLISIONDETECTNODEKDOPS_H
