#include "cutHead.h"
#include <algorithm>

using namespace std;
namespace dtk
{
	cutHead::cutHead(dtkStaticTriangleMesh::Ptr triangleMesh, const std::vector<dtkID> & avoidPoint)
	{
		mTriangleMesh = triangleMesh;
		mAvoidPoint = avoidPoint;
	}

	const std::vector<dtkID3> & cutHead::AvoidTriangle()
	{
		const std::vector<dtkID3> & ecTable = mTriangleMesh->GetECTable();
		dtkID size = ecTable.size();

		for (dtkID i = 0; i < size; i++)
		{
			vector<dtkID>::iterator it1;  // the first point of triangle
			vector<dtkID>::iterator it2;  // the second point of triangle
			vector<dtkID>::iterator it3;  // the third point of triangle 

			it1 = find(mAvoidPoint.begin(), mAvoidPoint.end(), ecTable[i].a);
			if (it1 == mAvoidPoint.end() );
			else
			{
				mAvoidTriangle.push_back(ecTable[i]);
				continue;
			}

			it2 = find(mAvoidPoint.begin(), mAvoidPoint.end(), ecTable[i].b);
			if (it2 == mAvoidPoint.end() );
			else
			{
				mAvoidTriangle.push_back(ecTable[i]);
				continue;
			}

			it3 = find(mAvoidPoint.begin(), mAvoidPoint.end(), ecTable[i].c);
			if (it3 == mAvoidPoint.end() );
			else
			{
				mAvoidTriangle.push_back(ecTable[i]);
				continue;
			}
		}

		return mAvoidTriangle;
	}
}