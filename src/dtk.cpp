#include "dtkGraphicsKernel.h"

#include "dtk.h"

#ifdef DTK_CUDA
    #include "cuda.h"
#endif //DTK_CUDA

namespace dtk
{
	bool dtkInited = false;

	void Init()
	{
		if (dtkInited) return;

        srand(static_cast<unsigned int>(time(NULL)));

    #ifdef DTK_CUDA
		//Initialize the CUDA enviroment.
		int deviceCount = 0;
		float *testMem;
		cudaGetDeviceCount(&deviceCount);
        //dtkAssert(deviceCount > 0, CUDA_ERROR);

		cudaMalloc((void**)&testMem, sizeof(int));
		cudaFree(testMem);
    #endif //DTK_CUDA

        //Initialize the components of DTK.
        //dtkStaticTetraMeshQuery::Init();

		dtkInited = true;
	}

}
