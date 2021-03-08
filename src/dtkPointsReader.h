#ifndef DTK_POINTSREADER_H
#define DTK_POINTSREADER_H

#include "dtkPoints.h"

#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkPointsReader: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkPointsReader> Ptr;

		static dtkPointsReader::Ptr New()
		{
			return dtkPointsReader::Ptr(new dtkPointsReader());
		}

	public:

		void SetFileName(const char* filePath);
		void SetOutput(dtkPoints::Ptr pts);

		bool Read();

	private:

		dtkPoints::Ptr	mPts;
		std::string		mFilePath;
	};
}

#endif //DTK_POINTSREADER_H

