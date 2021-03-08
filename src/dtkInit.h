#ifndef DTK_INIT_H
#define DTK_INIT_H

namespace dtk
{
	//Whether the dtk has been initialized.
	//Defined in dtk.cpp
	extern bool dtkInited;

	//You must call this function once before you use DTK package.
	//Defined in dtk.cpp
    extern "C" void Init();
}

#endif
