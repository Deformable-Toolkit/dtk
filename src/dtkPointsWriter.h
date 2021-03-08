#ifndef DTK_POINTSWRITER_H
#define DTK_POINTSWRITER_H

#include <vector>
#include <cstring>
#include <memory>
#include <boost/utility.hpp>

#include "dtkPoints.h"

namespace dtk
{
	class dtkPointsWriter:public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkPointsWriter> Ptr;

		static dtkPointsWriter::Ptr New()
		{
			return dtkPointsWriter::Ptr(new dtkPointsWriter());
		}

	public:

		void SetPoints(dtkPoints::Ptr pts);
		void SetPoints(dtkPoints::Ptr pts, const std::vector<bool> &valid);
		void SetFileName(const char* fileName);

		bool Write();

	private:
		dtkPointsWriter();

		dtkPoints::Ptr	mPts;
		std::string		mFilePath;
		std::vector<bool> mValid;
	};
}

#endif //DTK_POINTSWRITER_H

