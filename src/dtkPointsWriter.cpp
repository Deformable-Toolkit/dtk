#include "dtkPointsWriter.h"

#include "dtkAssert.h"

#include <fstream>

namespace dtk
{
	dtkPointsWriter::dtkPointsWriter()
	{
		//nothing.
	}

	void dtkPointsWriter::SetPoints(dtkPoints::Ptr pts)
	{
		dtkAssert(pts.get() != NULL, NULL_POINTER);
		mPts = pts;
		mValid.clear();
		mValid.resize(mPts->GetMaxID() + 1, true);
	}

	void dtkPointsWriter::SetPoints(dtkPoints::Ptr pts, const std::vector<bool> &valid)
	{
		dtkAssert(pts.get() != NULL, NULL_POINTER);
		mPts = pts;
		mValid = valid;
	}

	void dtkPointsWriter::SetFileName(const char* fileName)
	{
		dtkAssert(fileName != NULL, NULL_POINTER);
		mFilePath = std::string(fileName);
	}

	bool dtkPointsWriter::Write()
	{
		bool rtn = true;

		rtn &= mPts.get() != NULL;
		rtn &= mFilePath.size() > 0;
		if (!rtn) 
        {
            dtkAssert(rtn, ILLEGAL_STATE);
            return false;
        }

		std::ofstream file(mFilePath.c_str());
		dtkID id;
        GK::Point3 coord;
		size_t numOfPts = 0;
		dtkID maxID = 0;

		//first iteration, count the number of valid points.
		mPts->Begin();
		while (mPts->Next(id, coord))
		{
			if (mValid[id]) ++numOfPts;
			if (id > maxID) maxID = id;
		}

		file<<numOfPts<<std::endl;
		file<<maxID<<std::endl;

		//Points
		mPts->Begin();
		while (mPts->Next(id, coord))
		{
			file.precision(16);
			if (mValid[id]) file<<id<<"\t"<<coord.x()<<" "<<coord.y()<<" "<<coord.z()<<std::endl;
		}

		file.close();

		return true;
	}
}

