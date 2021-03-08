#ifndef DTK_ASSERT_H
#define DTK_ASSERT_H

#include "dtkConfig.h"
#include "dtkError.h"
#include "dtkErrorManager.h"

namespace dtk
{
#ifdef DTK_DEBUG
	inline bool dtkAssert(bool cond, dtkError err = UNKNOW_ERROR)
    {
        if (!cond) dtkErrMgr.PushError(err);
        if (cond) return true;

        assert(cond);
        return cond;
    }
#else
    //#define dtkAssert( cond, err ) ( (void) 0 )
	inline bool dtkAssert(bool cond, dtkError err = UNKNOW_ERROR)
	{
		if (!cond) 
			dtkErrMgr.PushError(err);
		return cond;
	}
#endif //DTK_DEBUG
}

#endif //DTK_ASSERT_H
