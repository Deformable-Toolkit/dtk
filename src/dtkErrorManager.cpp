#include "dtkErrorManager.h"

#include "dtkAssert.h"

namespace dtk
{	
	
	//---------------------------------------------
	dtkErrorManager& dtkErrMgr = dtkErrorManager::GetInstance();
	
	dtkErrorManager dtkErrorManager::msErrorMgr;

	//---------------------------------------------
	dtkErrorManager::dtkErrorManager()
	{
		std::cout << "default constructor:"  << &msErrorMgr << std::endl; 
		InitErrorStrings();
	}
	
	//---------------------------------------------
	dtkErrorManager& dtkErrorManager::GetInstance()
	{
		//std::cout << "IN class:"  << &msErrorMgr << std::endl; 
		return msErrorMgr;
	}

	//---------------------------------------------
	size_t dtkErrorManager::GetNumberOfErrors()
	{
		return mErrors.size();
	}

	//---------------------------------------------
	dtkError dtkErrorManager::GetLatestError()
	{
		dtkError retVal;

		if (mErrors.empty())
		{
			retVal = NOT_ERROR;
		}
		else
		{
			retVal = mErrors.top();
		}

		return retVal;
	}
	
	//---------------------------------------------
	const std::string& 
		dtkErrorManager::GetErrorString(dtkError error)
	{
		return mErrStrs[error];
	}

	//---------------------------------------------
	void dtkErrorManager::PushError(dtkError error)
	{
		mErrors.push(error);
	}

	//---------------------------------------------
	void dtkErrorManager::PopError(dtkError error)
	{
		mErrors.pop();
	}

	//---------------------------------------------
	void dtkErrorManager::Reset()
	{
		size_t size = mErrors.size();
		for (size_t i = 0; i != size; ++i)
			mErrors.pop();

		dtkAssert(mErrors.empty(), UNKNOW_ERROR);
	}

	//---------------------------------------------	
	void dtkErrorManager::InitErrorStrings()
	{
		InitErrorString(NOT_ERROR,			"No Error");
		InitErrorString(OUT_OF_RANGE, 		"Out of range!");
	}

	//---------------------------------------------
	void dtkErrorManager::InitErrorString(dtkError id, const char* str)
	{
		mErrStrs.insert(std::make_pair(id, std::string(str)));
	}
}
