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

        //递归划分
        void Split();
        
        //更新包围盒
        void Update();


        //根据图元重心平均值划分为左右分支。
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
