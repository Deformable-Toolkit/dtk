#include "dtkPointsReader.h"

#include "dtkAssert.h"

#include <fstream>

namespace dtk
{
	void dtkPointsReader::SetOutput(dtkPoints::Ptr pts)
	{
		dtkAssert(pts.get() != NULL, NULL_POINTER);
		mPts = pts;
	}

	void dtkPointsReader::SetFileName(const char* fileName)
	{
		dtkAssert(fileName != NULL, NULL_POINTER);
		mFilePath = std::string(fileName);
	}

	bool dtkPointsReader::Read()
	{
		bool rtn = true;

		rtn &= mPts.get() != NULL;
		rtn &= mFilePath.size() > 0;
		if (!rtn) 
        {
            dtkAssert(rtn, ILLEGAL_STATE);
            return false;
        }

		std::ifstream file(mFilePath.c_str());
		size_t numOfPts;
		dtkID maxID;

		file >> numOfPts >> maxID;

		for (size_t i = 0; i < numOfPts; ++i)
		{
			dtkID id;
			dtkFloat3 coord;

			file >> id >> coord.x >> coord.y >> coord.z;
			mPts->SetPoint(id, GK::Point3(coord.x, coord.y, coord.z));
		}

		file.close();
		return rtn;
	}
}

