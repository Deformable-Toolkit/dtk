#ifndef DTK_COLLISIONDETECTHIERARCHYKDOPS_H
#define DTK_COLLISIONDETECTHIERARCHYKDOPS_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkCollisionDetectHierarchy.h"
#include "dtkCollisionDetectNodeKDOPS.h"

namespace dtk
{
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

        //构建冲突检测树
        void Build();

        //重新构建冲突检测树
        void Rebuild();

        //更新图元包围盒
        void Update();

    private:
        dtkCollisionDetectHierarchyKDOPS( size_t half_k );

    private:
        size_t mHalfK;  //包围盒维度
	};
}

#endif //DTK_COLLISIONDETECTHIERARCHYKDOPS_H
