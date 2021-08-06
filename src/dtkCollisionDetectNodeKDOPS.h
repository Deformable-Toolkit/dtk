#ifndef DTK_COLLISIONDETECTNODEKDOPS_H
#define DTK_COLLISIONDETECTNODEKDOPS_H

#include "dtkConfig.h"
#include "dtkCollisionDetectNode.h"

#include "dtkGraphicsKernel.h"

namespace dtk
{
    class dtkCollisionDetectHierarchyKDOPS;

    /**
    * @class <dtkCollisionDetectNode> 
    * @brief k-Dops算法冲突检测树结点
    * @author <>
    * @note
    * k-Dops算法冲突检测树结点类，继承于dtkCollisionDetectNode基类。
    */
	class dtkCollisionDetectNodeKDOPS : public dtkCollisionDetectNode
	{
	public:
        dtkCollisionDetectNodeKDOPS( dtkCollisionDetectHierarchy* father, size_t half_k );
        
		~dtkCollisionDetectNodeKDOPS();

        /**
         * @brief 递归划分k-Dops冲突检测树。
         */
        //递归划分
        void Split();
        
        /**
         * @brief 更新k-Dops冲突检测树轴向包围盒。
         */
        //更新包围盒
        void Update();

        /**
         * @brief 根据图元重心平均值划分为节点为左右分支。
         */
        //根据图元重心平均值划分为左右分支。
        void SplitRule();

        inline const GK::KDOP& GetKDOP() const
        {
            return mKDOP;
        }

    private:
        GK::KDOP mKDOP; /**< 轴向多面体包围盒 */
	};
}

#endif //DTK_COLLISIONDETECTNODEKDOPS_H
