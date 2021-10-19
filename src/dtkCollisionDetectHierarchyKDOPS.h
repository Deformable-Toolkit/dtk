#ifndef DTK_COLLISIONDETECTHIERARCHYKDOPS_H
#define DTK_COLLISIONDETECTHIERARCHYKDOPS_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkCollisionDetectHierarchy.h"
#include "dtkCollisionDetectNodeKDOPS.h"

namespace dtk
{   
    /**
    * @class <dtkCollisionDetectHierarchyKDOPS> 
    * @brief k-Dops冲突检测算法
    * @author <>
    * @note
    * k-DOPs算法冲突检测树的构建，碰撞检测的执行， 图元的更新等。
    */
	class dtkCollisionDetectHierarchyKDOPS: public dtkCollisionDetectHierarchy
	{
	public:
		typedef std::shared_ptr<dtkCollisionDetectHierarchyKDOPS> Ptr;
        
        static dtkCollisionDetectHierarchyKDOPS::Ptr New( size_t half_k )
        {
			return dtkCollisionDetectHierarchyKDOPS::Ptr(new dtkCollisionDetectHierarchyKDOPS( half_k ));
        }

	public:
		virtual ~dtkCollisionDetectHierarchyKDOPS();

        /**
		* @brief		构建k-Dpos冲突检测树
		*/
        //构建冲突检测树
        void Build();

        /**
		* @brief		在图元更新时，重新构建冲突检测树
		*/
        //重新构建冲突检测树
        void Rebuild();

        /**
		* @brief		k-Dops算法更新，更新轴向包围盒,更新图元状态
		*/
        //更新图元包围盒
        void Update();

    private:
        dtkCollisionDetectHierarchyKDOPS( size_t half_k );

    private:
        size_t mHalfK;  /**< k-Dops算法轴向包围盒维度 */
	};
}

#endif //DTK_COLLISIONDETECTHIERARCHYKDOPS_H
