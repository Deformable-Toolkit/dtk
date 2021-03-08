#ifndef DTK_PHYSKNOTPLANNER_H
#define DTK_PHYSKNOTPLANNER_H

#include <memory>
#include <boost/utility.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>

#include "dtkPhysTetraMassSpring.h"
#include "dtkPhysMassSpringThread.h"

#include "dtkCollisionDetectPrimitive.h"
#include "dtkIntersectTest.h"

#include <vector>
#include <map>

namespace dtk
{
	class dtkPhysKnotPlanner : public boost::noncopyable
	{
	public:
		typedef std::shared_ptr< dtkPhysKnotPlanner > Ptr;

	public:
		~dtkPhysKnotPlanner();
		void KnotRecognition( std::vector<dtkIntersectTest::IntersectResult::Ptr>&);

		void DoKnotFormation();

		void UpdateKnotOrientations();

		void UpdateKnot(double timeslice);

		std::vector< dtkID2 > GetKnots()
		{
			return mKnots;
		}

		std::vector< dtkDouble3 > GetKnotOrientations()
		{
			return mKnotOrientations;
		}

		std::vector< dtkDouble3 > GetKnotCenterPoints()
		{
			return mKnotCenterPoints;
		}

		std::vector< dtkID > GetSegmentInKnots()
		{
			return mSegmentInKnots;
		}

		std::vector< double > GetPointOnSegmentPercent()
		{
			return mPointOnSegmentPercents;
		}
		void UpdateKnotCenterPoints();

		void UpdateSegmentInKnot();

		static Ptr New( dtkPhysMassSpringThread::Ptr newSutureThread )
		{
			return Ptr( new dtkPhysKnotPlanner( newSutureThread ) );
		}

		const std::vector< dtkInterval<int> >& GetAvoidIntervals() { return mAvoidIntervals; }



	public:
		dtkPhysKnotPlanner( dtkPhysMassSpringThread::Ptr newSutureThread );

		bool mDoKnotPlanning;
		
		std::vector< dtkID2 > mKnots;
		dtkID mNumberOfFormatedKnots;
		std::vector< dtkID > mSegmentInKnots;
		std::vector< dtkDouble3 > mKnotCenterPoints;
		std::vector< double > mPointOnSegmentPercents;
		std::vector< dtkDouble3 > mKnotOrientations;
		std::vector< bool > mSegmentInKnotOrientations;
		std::vector< dtkInterval<int> > mAvoidIntervals;
		
	public:
		dtkPhysMassSpringThread::Ptr mSutureThread;
	};
}
#endif