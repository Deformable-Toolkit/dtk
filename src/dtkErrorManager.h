#ifndef DTK_ERRORMANAGER_H
#define DTK_ERRORMANAGER_H

//STL headers
#include <vector>
#include <stack>
#include <string>
#include <map>

#include <iostream>

#include <boost/utility.hpp>

//DTK headers
#include "dtkConfig.h"
#include "dtkError.h"

namespace dtk
{
	//! A standard error reporter
    /*!
     * Implemented in Singleton
     */
	class dtkErrorManager: public boost::noncopyable
	{
	public:
		static dtkErrorManager& GetInstance();

		size_t GetNumberOfErrors();
		
		dtkError GetLatestError();
		const std::string& GetErrorString(dtkError error);

		void PushError	(dtkError error);
		void PopError	(dtkError error);

		void Reset();

	private:
		dtkErrorManager();
		static dtkErrorManager msErrorMgr;

		void InitErrorStrings();
		void InitErrorString(dtkError id, const char* str);
		
	private:
		std::map<dtkError, std::string> mErrStrs;
		std::stack<dtkError> mErrors;
	};

	extern dtkErrorManager& dtkErrMgr;
}

#endif //DTK_ERRORMANAGER_H
