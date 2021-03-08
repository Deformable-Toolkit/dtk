#ifndef DTK_DTKCOLLISIONDETECTBASIC_H
#define DTK_DTKCOLLISIONDETECTBASIC_H

#include "dtkGraphicsKernel.h"
#include "dtkCollisionDetectNode.h"
#include "dtkCollisionDetectPrimitive.h"
#include "dtkIntersectTest.h"

namespace dtk
{
    class dtkCollisionDetectBasic
    {
        public:
            typedef dtkIntersectTest::IntersectResult IntersectResult;

        public:
            static bool DoIntersect( 
                    dtkCollisionDetectPrimitive* pri_1, 
                    dtkCollisionDetectPrimitive* pri_2, 
                    IntersectResult::Ptr& result, 
                    bool self = false,
                    bool ignore_extend = false);

            static bool DoIntersect( const dtkCollisionDetectNode* node_1, const dtkCollisionDetectNode* node_2 );
    };

    typedef dtkCollisionDetectBasic CDBasic;
};

#endif //DTK_DTKCOLLISIONDETECTBASIC_H
