#ifndef DTK_ERROR_H
#define DTK_ERROR_H

#include <cassert>

namespace dtk
{
	enum dtkError
	{
		//NO_ERROR = 0,
		NOT_ERROR = 0,
		OUT_OF_RANGE,
		ILLEGAL_ARGUMENT,
		ILLEGAL_STATE,
		OPERATION_FAILED,
		MUST_BE_MANIFOLD_MESH,
		DEGENERACY_TRIANGLE,
		DEGENERACY_TETRA,
		CANNOT_FIND_OBJECT,
		NULL_POINTER,
        EMPTY_DATASET,
		DIVIDED_BY_ZERO,
		CUDA_ERROR,
		OUT_OF_MEMORY,
		SOLUTION_ERROR,
		CUDPP_SCAN_FAILED,
		PROPERTY_WRONG_VALUETYPE,
        LOGICAL_ERROR,
        NOT_IMPLEMENTED,
        SIZE_NOT_MATCH,

		//This enum must in the last entry. (for counting)
		//Insert new error type before it.
		UNKNOW_ERROR
	};
}

#endif //DTK_ERROR_H
