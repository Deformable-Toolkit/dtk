#ifndef DTK_PHYSMASSSPRINGTHREADCOLLISIONRESPONSE_H
#define DTK_PHYSMASSSPRINGTHREADCOLLISIONRESPONSE_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkPhysMassSpringCollisionResponse.h"
#include "dtkPhysMassSpringThread.h"

#include <vector>
#include <map>

namespace dtk
{
	class dtkPhysMassSpringThreadCollisionResponse : public boost::noncopyable
	{
	public:
        struct virtualNodeOnSegment
        {
            dtkDouble2 uv;
            int segmentID;
        };
        
        enum PiercedResultType
        {
            PIERCED_PRIMITIVE_1,
            PIERCED_PRIMITIVE_2,
            PIERCED_WEIGHT_1,
            PIERCED_WEIGHT_2,
            PIERCED_SEGMENTID,
            PIERCED_SURFACE,
            PIERCED_VALID
        };

        typedef dtkProperty<PiercedResultType> PiercedResult;
 
		typedef std::shared_ptr< dtkPhysMassSpringThreadCollisionResponse > Ptr;

		static Ptr New( dtkPhysMassSpringCollisionResponse::Ptr priorResponse ) 
		{
			return Ptr( new dtkPhysMassSpringThreadCollisionResponse( priorResponse ) );
		}

        //use for debug
		std::pair< dtkDouble3, dtkDouble3 > GetVirtualPair( dtkID threadID, dtkID id );
        //std::pair< dtkDouble3, dtkDouble3 > GetInternalVirtualPair( dtkID threadID, dtkID id );

        size_t GetPiercedNum( dtkID i )
        {
            return mPiercedResults[i].size();
        }

        //size_t GetInternalPiercedNum( dtkID i )
        //{
        //    return mInternalPiercedResults[i].size();
        //}

	public:
		~dtkPhysMassSpringThreadCollisionResponse();

        void Update( double timeslice, std::vector<dtkIntersectTest::IntersectResult::Ptr>& internalPiercingResults );

		void PostProcess(double range);

        void SetThread( dtkID i, dtkPhysMassSpringThread::Ptr thread )
        {
            mThreads[i] = thread;

            mThreadHeadInsides[i] = false;
			mNumerOfSurfacePiercedResults[i] = 0;
            mPiercedResults[i] = std::vector< PiercedResult::Ptr >();
            mAvoidIntervals[i] = std::vector< dtkInterval<int> >();
            mInternalIntervals[i] = std::vector< dtkInterval<int> >();

            //mInternalPiercedResults[i] = std::vector< PiercedResult::Ptr >();
            //mInternalPiercingTriangleIDs[i] = dtkErrorID;
        }

        dtkPhysMassSpringThread::Ptr GetThread( dtkID majorID )
        {
            assert( mThreads.find( majorID ) != mThreads.end() );
            return mThreads[majorID];
        }

        const std::vector< dtkInterval<int> >& GetAvoidIntervals( dtkID i )
        {
            assert( mAvoidIntervals.find( i ) != mAvoidIntervals.end() );
            return mAvoidIntervals[i];
        }

        const std::vector< dtkInterval<int> >& GetInternalIntervals( dtkID i )
        {
            assert( mInternalIntervals.find( i ) != mInternalIntervals.end() );
            return mInternalIntervals[i];
        }

		bool stable;

    private:
		dtkPhysMassSpringThreadCollisionResponse( dtkPhysMassSpringCollisionResponse::Ptr priorResponse );

    private:
        dtkPhysMassSpringCollisionResponse::Ptr                     mPriorResponse;

        std::map< dtkID, dtkPhysMassSpringThread::Ptr >     mThreads;

        std::map< dtkID, bool >                                     mThreadHeadInsides;

		std::map< dtkID, int >										mNumerOfSurfacePiercedResults;

        std::map< dtkID, std::vector< PiercedResult::Ptr > >        mPiercedResults;

        std::map< dtkID, std::vector< dtkInterval<int> > >          mAvoidIntervals;

        std::map< dtkID, std::vector< dtkInterval<int> > >          mInternalIntervals;

        //std::map< dtkID, std::vector< PiercedResult::Ptr > >        mInternalPiercedResults;

        //std::map< dtkID, std::vector< dtkID > >                     mInternalPiercingTriangleIDs;
	};
}

#endif
