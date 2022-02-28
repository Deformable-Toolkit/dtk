#include "dtkPhysKnotPlanner.h"

#include <algorithm>
#include <set>
using namespace std;
using namespace boost;

namespace dtk
{
	dtkPhysKnotPlanner::dtkPhysKnotPlanner(dtkPhysMassSpringThread::Ptr newSutureThread)
	{
		mSutureThread = newSutureThread;
		mDoKnotPlanning = true;
		mNumberOfFormatedKnots = 0;
	}

	dtkPhysKnotPlanner::~dtkPhysKnotPlanner()
	{
	}

	void dtkPhysKnotPlanner::KnotRecognition(vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults)
	{
		if (mDoKnotPlanning)
		{
			set<dtkID> collisionSegments;
			for (dtkID i = 0; i < intersectResults.size(); i++)
			{
				dtkIntersectTest::IntersectResult::Ptr result = intersectResults[i];
				dtkCollisionDetectPrimitive *pri_1;
				dtkCollisionDetectPrimitive *pri_2;

				result->GetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1);
				result->GetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2);

				int pairDistance = abs((int)pri_1->mMinorID - (int)pri_2->mMinorID);
				if (pairDistance > 3 && pairDistance < 15)
				{
					collisionSegments.insert(pri_1->mMinorID);
					collisionSegments.insert(pri_2->mMinorID);
				}
			}

			bool knotFormed = true;
			int serializedSegmentsNum = 0;
			int interSegmentsNum = 0;
			int crossNum = 0;
			std::vector<dtkID> sortedCollisionSegments;
			for (set<dtkID>::iterator itr = collisionSegments.begin(); itr != collisionSegments.end(); itr++)
				sortedCollisionSegments.push_back(*itr);

			vector<dtkID2> continuousSegments;
			dtkID continuousSegmentStart = dtkIntMax;
			dtkID continuousSegmentEnd = dtkIntMax;
			// merge discretion interval to continuous interval.
			for (dtkID i = 0; i < sortedCollisionSegments.size(); i++)
			{
				if (continuousSegmentStart == dtkIntMax)
				{
					continuousSegmentStart = continuousSegmentEnd = sortedCollisionSegments[i];
				}
				else if (sortedCollisionSegments[i] - continuousSegmentEnd == 1)
				{
					continuousSegmentEnd = sortedCollisionSegments[i];
				}
				else
				{
					continuousSegments.push_back(dtkID2(
						continuousSegmentStart, continuousSegmentEnd));
					continuousSegmentStart = continuousSegmentEnd = sortedCollisionSegments[i];
				}
			}

			if (continuousSegmentStart != dtkIntMax)
				continuousSegments.push_back(dtkID2(
					continuousSegmentStart, continuousSegmentEnd));

			for (dtkID i = 0; i + 1 < continuousSegments.size(); i++)
			{
				if (continuousSegments[i][1] - continuousSegments[i][0] > 1 && continuousSegments[i + 1][1] - continuousSegments[i + 1][0] > 1 && continuousSegments[i + 1][0] - continuousSegments[i][1] < 7)
				{
					mKnots.push_back(dtkID2(
						continuousSegments[i][0], continuousSegments[i + 1][1]));
					mSegmentInKnots.push_back(dtkIntMax);
					mKnotCenterPoints.push_back(dtkDouble3());
					mKnotOrientations.push_back(dtkDouble3());
					mSegmentInKnotOrientations.push_back(true);
					mPointOnSegmentPercents.push_back(0);

					// Knot Only One
					mDoKnotPlanning = false;
					break;
				}
			}

			// Method: Two Crosses

			//vector< dtkID2 > crosses;
			//for( dtkID i = 0; i + 1 < continuousSegments.size(); i++ )
			//{
			//	int difference = continuousSegments[i+1][0] - continuousSegments[i][1];
			//	if( difference > 3 && difference < 8)
			//	{
			//		crosses.push_back( dtkID2(
			//			continuousSegments[i][0], continuousSegments[i+1][1] ) );
			//	}
			//}

			//for( dtkID i = 0; i + 1 < crosses.size(); i++ )
			//{
			//	if( crosses[i][1] >= crosses[i+1][0] )
			//	{
			//		// At least three points in one knot
			//		if( crosses[i+1][1] < crosses[i][0] + 2 )
			//			assert(false);

			//		cout << "continuousSegments: ";
			//		for( dtkID c = 0; c < continuousSegments.size(); c++ )
			//		{
			//			cout << continuousSegments[c] << "; ";
			//		}
			//		cout << endl;
			//		cout << "crosses: ";
			//		for( dtkID c = 0; c < crosses.size(); c++ )
			//		{
			//			cout << crosses[c] << "; ";
			//		}
			//		cout << endl;

			//		mKnots.push_back( dtkID2(
			//			crosses[i][0], crosses[i+1][1] ) );
			//		mSegmentInKnots.push_back( dtkIntMax );
			//		mKnotCenterPoints.push_back( dtkDouble3() );
			//		mKnotOrientations.push_back( dtkDouble3() );
			//		mSegmentInKnotOrientations.push_back( true );
			//		cout<<"range: "<<crosses[i][0]<<" to "<<crosses[i+1][1]<<endl;
			//
			//		// Knot Only One
			//		mDoKnotPlanning = false;
			//		break;
			//	}
			//}
		}

		UpdateKnotCenterPoints();
		UpdateKnotOrientations();

		double trapRange = mSutureThread->GetInterval() * 3.0;
		for (dtkID i = 0; i < mKnots.size(); i++)
		{
			if (mSegmentInKnots[i] != dtkIntMax)
				continue;

			double minDistance = 1000;
			dtkID minSegID = dtkIntMax;
			dtkDouble3 minTrappedSegmentCenter(0, 0, 0);
			for (dtkID j = 0; j < mSutureThread->GetNumberOfSegments(); j++)
			{
				if (j + 7 > mKnots[i][0] && j < mKnots[i][1] + 7)
					continue;

				dtkDouble3 trappedSegmentCenter = (mSutureThread->GetMassPoint(j * 2)->GetPosition() + mSutureThread->GetMassPoint(j * 2 + 2)->GetPosition()) * 0.5;
				if (length(trappedSegmentCenter - mKnotCenterPoints[i]) < trapRange)
				{
					if (length(trappedSegmentCenter - mKnotCenterPoints[i]) < minDistance)
					{
						minDistance = length(trappedSegmentCenter - mKnotCenterPoints[i]);
						minSegID = j;
						minTrappedSegmentCenter = trappedSegmentCenter;
					}
				}
			}
			if (minSegID != dtkIntMax)
			{
				mSegmentInKnots[i] = minSegID;
				mPointOnSegmentPercents[i] = 0.5;
				mSegmentInKnotOrientations[i] =
					dot(mSutureThread->GetMassPoint(i * 2)->GetPosition() -
							mSutureThread->GetMassPoint(i * 2 + 2)->GetPosition(),
						mKnotOrientations[i]) > 0;
				mAvoidIntervals.push_back(dtkInterval<int>((int)mSegmentInKnots[i] - 4, (int)mSegmentInKnots[i] + 4));
			}
		}

		return;
	}

	void dtkPhysKnotPlanner::DoKnotFormation()
	{
		if (mKnots.size() <= mNumberOfFormatedKnots)
			return;

		for (dtkID i = mNumberOfFormatedKnots; i < mKnots.size(); i++)
		{
			dtkID pointId1 = mKnots[i][0] * 2;
			dtkID pointId2 = mKnots[i][1] * 2 + 2;
			for (dtkID i = pointId1; i < pointId2; i += 2)
			{
				dtkPhysMassPoint *point1 = mSutureThread->GetMassPoint(i);
				for (dtkID j = i + 2; j <= pointId2; j += 2)
				{
					dtkPhysMassPoint *point2 = mSutureThread->GetMassPoint(j);
					point1->AddTwin(point2);
					point2->AddTwin(point1);
				}
			}
			for (dtkID i = pointId1; i <= pointId2; i += 2)
			{
				dtkPhysMassPoint *point = mSutureThread->GetMassPoint(i);
				point->ResetDynamicState();
			}
		}

		mNumberOfFormatedKnots = mKnots.size();
	}

	void dtkPhysKnotPlanner::UpdateKnotOrientations()
	{
		for (dtkID i = 0; i < mKnots.size(); i++)
		{
			dtkID mid = (mKnots[i][0] + mKnots[i][1]) / 2;
			dtkID segmentID_1 = mid - 1;
			dtkID segmentID_2 = mid + 1;
			//if( segmentID_1 == segmentID_2 )
			//	segmentID_2 = mKnotSegments[mid+2];

			//if( segmentID_2 - segmentID_1 > 2)
			//{
			//	segmentID_1 += (segmentID_2 - segmentID_1) / 2;
			//	segmentID_2 = segmentID_1 + 1;
			//}
			dtkPhysMassPoint *point1_1 = mSutureThread->GetMassPoint(segmentID_1 * 2);
			dtkPhysMassPoint *point1_2 = mSutureThread->GetMassPoint(segmentID_1 * 2 + 2);

			dtkPhysMassPoint *point2_1 = mSutureThread->GetMassPoint(segmentID_2 * 2);
			dtkPhysMassPoint *point2_2 = mSutureThread->GetMassPoint(segmentID_2 * 2 + 2);

			dtkDouble3 vec1 = point1_2->GetPosition() - point1_1->GetPosition();
			dtkDouble3 vec2 = point2_2->GetPosition() - point2_1->GetPosition();

			//cout<<"vec1: "<<vec1[0]<<" "<<vec1[1]<<" "<<vec1[2]<<endl;
			//cout<<"vec2: "<<vec2[0]<<" "<<vec2[1]<<" "<<vec2[2]<<endl;
			mKnotOrientations[i] = normalize(cross(vec1, vec2));
		}
	}

	void dtkPhysKnotPlanner::UpdateKnotCenterPoints()
	{
		for (dtkID i = 0; i < mKnots.size(); i++)
		{
			mKnotCenterPoints[i] = dtkDouble3(0, 0, 0);
			double sum = 3;
			mKnotCenterPoints[i] = mKnotCenterPoints[i] + mSutureThread->GetMassPoint(mKnots[i][0] * 2)->GetPosition();
			mKnotCenterPoints[i] = mKnotCenterPoints[i] + mSutureThread->GetMassPoint(mKnots[i][1] * 2 + 2)->GetPosition();
			mKnotCenterPoints[i] = mKnotCenterPoints[i] + mSutureThread->GetMassPoint(mKnots[i][0] + mKnots[i][1] + 1)->GetPosition();

			mKnotCenterPoints[i] = mKnotCenterPoints[i] / sum;
		}
	}

	void dtkPhysKnotPlanner::UpdateKnot(double timeslice)
	{
		UpdateKnotOrientations();
		UpdateKnotCenterPoints();
		UpdateSegmentInKnot();

		for (dtkID i = 0; i < mKnots.size(); i++)
		{
			if (mSegmentInKnots[i] == dtkIntMax)
				continue;

			dtkPhysMassPoint *pointOnSegment_1 = mSutureThread->GetMassPoint(mSegmentInKnots[i] * 2);
			dtkPhysMassPoint *pointOnSegment_2 = mSutureThread->GetMassPoint(mSegmentInKnots[i] * 2 + 2);

			dtkDouble3 pointOnSegment_1_oldPos = pointOnSegment_1->GetPosition();
			dtkDouble3 pointOnSegment_2_oldPos = pointOnSegment_2->GetPosition();

			dtkDouble3 segmentVec = pointOnSegment_1_oldPos - pointOnSegment_2_oldPos;
			double segmentLength = length(segmentVec);

			double percent = mPointOnSegmentPercents[i];

			double interval_1 = segmentLength * percent;
			double interval_2 = segmentLength * (1.0 - percent);
			//dtkDouble3 tempPoint;
			//tempPoint = (pointOnSegment_1->GetPosition() + pointOnSegment_2->GetPosition()) * 0.5;

			//dtkDouble3 vec = tempPoint - mKnotCenterPoints[i];
			//vec = vec / (double)2;

			//for( dtkID i = 0; i < mKnotSegments.size(); i++ )
			//{
			//	dtkPhysMassPoint* p = mSutureThread->GetMassPoint( mKnotSegments[i] * 2 );
			//	p->SetPosition(p->GetPosition() + vec);
			//	p->SetVel(p->GetVel() + vec / timeslice);
			//}

			dtkDouble3 pointOnSegment_1_newPos;
			dtkDouble3 pointOnSegment_2_newPos;
			//if( mSegmentInKnotOrientations[i] )
			//{
			//	pointOnSegment_1_newPos = mKnotCenterPoints[i] + mKnotOrientations[i] * interval_1;//halfInterval;
			//	pointOnSegment_2_newPos = mKnotCenterPoints[i] - mKnotOrientations[i] * interval_2;//halfInterval;
			//}
			//else
			//{
			//	pointOnSegment_2_newPos = mKnotCenterPoints[i] + mKnotOrientations[i] * interval_2;//halfInterval;
			//	pointOnSegment_1_newPos = mKnotCenterPoints[i] - mKnotOrientations[i] * interval_1;//halfInterval;
			//}
			dtkDouble3 vec = pointOnSegment_1_oldPos - segmentVec * percent - mKnotCenterPoints[i];
			pointOnSegment_1_newPos = pointOnSegment_1_oldPos - vec;
			pointOnSegment_2_newPos = pointOnSegment_2_oldPos - vec;

			pointOnSegment_1->SetPosition(pointOnSegment_1_newPos);
			pointOnSegment_1->SetVel(pointOnSegment_1->GetVel() + (pointOnSegment_1_newPos - pointOnSegment_1_oldPos) / timeslice);
			pointOnSegment_2->SetPosition(pointOnSegment_2_newPos);
			pointOnSegment_2->SetVel(pointOnSegment_2->GetVel() + (pointOnSegment_2_newPos - pointOnSegment_2_oldPos) / timeslice);
		}
	}

	void dtkPhysKnotPlanner::UpdateSegmentInKnot()
	{
		mAvoidIntervals.clear();
		for (dtkID i = 0; i < mSegmentInKnots.size(); i++)
		{
			if (mSegmentInKnots[i] == dtkIntMax)
				continue;
			dtkDouble3 pointOnSegment_0 = mSutureThread->GetMassPoint(mSegmentInKnots[i] * 2)->GetPosition();
			dtkDouble3 pointOnSegment_1 = mSutureThread->GetMassPoint(mSegmentInKnots[i] * 2 + 2)->GetPosition();

			dtkDouble3 knotCenterPoint = mKnotCenterPoints[i];
			dtkDouble3 knotOrientation = mKnotOrientations[i];

			bool segmentInKnotOrientation = mSegmentInKnotOrientations[i];
			double oriPercent = mPointOnSegmentPercents[i];

			dtkDouble3 tempSegmentOri = pointOnSegment_0 - pointOnSegment_1;

			dtkDouble3 impulseVec(pointOnSegment_0 - tempSegmentOri * oriPercent - knotCenterPoint);
			dtkDouble3 p01_normal = normalize(tempSegmentOri);
			double cosangle = dot(p01_normal, impulseVec);

			dtkDouble3 proj = p01_normal * cosangle;
			double percent = /*length( proj )*/ cosangle / /*mSutureThread->GetInterval();*/ length(tempSegmentOri);
			/*if( percent > 0.1 )
				percent = 0.1;
			else if( percent < -0.1 )
				percent = -0.1;*/
			double temp = mSegmentInKnots[i];
			if (cosangle > 0)
			{
				int advance = (int)(oriPercent + percent);
				mSegmentInKnots[i] = mSegmentInKnots[i] + advance;
				if (mSegmentInKnots[i] >= mSutureThread->GetNumberOfSegments())
					mSegmentInKnots[i] = mSutureThread->GetNumberOfSegments() - 1;
				if (advance > 0)
					mPointOnSegmentPercents[i] = oriPercent + percent - (double)advance;
				else
					mPointOnSegmentPercents[i] = oriPercent + percent;
			}
			else
			{
				int advance = (int)(oriPercent + percent);
				mSegmentInKnots[i] = mSegmentInKnots[i] + advance;
				if (mSegmentInKnots[i] < 0)
					mSegmentInKnots[i] = 0;
				if (advance < 0)
					mPointOnSegmentPercents[i] = oriPercent + percent - (double)advance;
				else
					mPointOnSegmentPercents[i] = oriPercent + percent;
			}

			mAvoidIntervals.push_back(dtkInterval<int>((int)mSegmentInKnots[i] - 4, (int)mSegmentInKnots[i] + 4));
		}
	}
};
