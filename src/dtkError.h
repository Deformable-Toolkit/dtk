#ifndef DTK_ERROR_H
#define DTK_ERROR_H

#include <cassert>

namespace dtk
{
	/** dtkError */
	enum dtkError
	{
		//NO_ERROR = 0,
		NOT_ERROR = 0, /**< not error. */
		OUT_OF_RANGE, /**< index out of range. */
		ILLEGAL_ARGUMENT, /**< argument is illegal. */
		ILLEGAL_STATE, /**< state is illegal. */
		OPERATION_FAILED, /**< operation failed. */
		MUST_BE_MANIFOLD_MESH, /**< mush must be manifold mesh. */
		DEGENERACY_TRIANGLE, /**< degeneracy triangle mesh. */
		DEGENERACY_TETRA, /**< degeneracy tetra mesh. */
		CANNOT_FIND_OBJECT, /**< object coan not find. */
		NULL_POINTER, /**< null pointer. */
        EMPTY_DATASET, /**< empty dataset. */
		DIVIDED_BY_ZERO, /**< divided by zero. */
		CUDA_ERROR, /**< CUDA error. */
		OUT_OF_MEMORY, /**< out of memory. */
		SOLUTION_ERROR, /**< solution error */
		CUDPP_SCAN_FAILED, /**< CUDPP scan failed. */
		PROPERTY_WRONG_VALUETYPE, /**< property wrong value type */
        LOGICAL_ERROR, /**< logical error */
        NOT_IMPLEMENTED, /**< not implemented. */
        SIZE_NOT_MATCH, /**< size not match. */

		/**
		* @brief unknown error. 
		* @note This enum must in the last entry. (for counting)
		* Insert new error type before it.
		*/
		UNKNOW_ERROR
	};
}

#endif //DTK_ERROR_H
