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

        void Build();

        void Rebuild();

        void Update();

    private:
        dtkCollisionDetectHierarchyKDOPS( size_t half_k );

    private:
        size_t mHalfK;
	};
}

#endif //DTK_COLLISIONDETECTHIERARCHYKDOPS_H
