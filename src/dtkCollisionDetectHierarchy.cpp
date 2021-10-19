#include "dtkCollisionDetectHierarchy.h"
#ifdef DTK_TBB
	#include <tbb/tbb.h>
	using namespace tbb;
#endif
#include <set>

#ifdef DTK_DEBUG
    #define DTKCOLLISIONDETECTHIERARCHY_DEBUG
#endif //DTK_DEBUG

#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
#include <iostream>
using namespace std;
#endif

using namespace boost;

namespace dtk
{
#ifdef DTK_TBB
    class ApplyUpdate {
        public:
        void operator()( const blocked_range<size_t>& r ) const {
            dtkCollisionDetectPrimitive** primitives = my_primitives;
            for( size_t i=r.begin(); i!=r.end(); ++i )
                Update(primitives[i]);
        }

        ApplyUpdate( dtkCollisionDetectPrimitive** primitives ) 
            : my_primitives(primitives)
        {
        }

        void Update(dtkCollisionDetectPrimitive* primitive) const
        {
            primitive->Update();
        }

        private:
            dtkCollisionDetectPrimitive** my_primitives;
    };
#endif

    void update( dtkID id, size_t numberOfThreads, 
            barrier* enter_barrier, barrier* exit_barrier, bool* running, 
            std::vector< dtkCollisionDetectPrimitive* >* primitives )
    {
        do {
            enter_barrier->wait();

            if( !(*running) )
                break;

            for( dtkID i = id; i < (dtkID)primitives->size(); i = i + numberOfThreads )
            {
                (*primitives)[i]->Update();
            }
            
            exit_barrier->wait();
        } while( true );
    }

    dtkCollisionDetectHierarchy::dtkCollisionDetectHierarchy()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[dtkCollisionDetectHierarchy::dtkCollisionDetectHierarchy]" << endl;
#endif
        mRoot = 0;
        mOrigin = GK::Point3( 0, 0, 0 );
		mMaxLevel = -1;

        mNumberOfThreads = 0;
        mThreadGroup = 0;
        mEnterBarrier = 0;
        mExitBarrier = 0;
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[/dtkCollisionDetectHierarchy::dtkCollisionDetectHierarchy]" << endl;
        cout << endl;
#endif
    }
    
    dtkCollisionDetectHierarchy::~dtkCollisionDetectHierarchy()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[dtkCollisionDetectHierarchy::~dtkCollisionDetectHierarchy]" << endl;
#endif
        if( mNumberOfThreads > 0 )
        {
            mLive = false;
            mEnterBarrier->wait();
            mThreadGroup->join_all();
            delete mThreadGroup;
            delete mEnterBarrier;
            delete mExitBarrier;
        }

        if( mRoot != 0 )
            delete mRoot;

        for( dtkID i = 0; i < GetNumberOfPrimitives(); i++ )
        {
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
            cout << "delete mPrimitives[" << i << "]" << endl;
#endif
            delete mPrimitives[i];
        }
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[/dtkCollisionDetectHierarchy::~dtkCollisionDetectHierarchy]" << endl;
        cout << endl;
#endif
    }
        
    void dtkCollisionDetectHierarchy::SetNumberOfThreads( size_t n )
    {
        if( n <= 0 )
            return;
        mNumberOfThreads = n;
        mLive = true;
        mThreadGroup = new thread_group();
        mEnterBarrier = new barrier( mNumberOfThreads + 1 );
        mExitBarrier = new barrier( mNumberOfThreads + 1 );
        for( dtkID i = 0; i < mNumberOfThreads; i++ )
        {
            mThreadGroup->add_thread( new thread( update, i, mNumberOfThreads, 
                        mEnterBarrier, mExitBarrier, &mLive, &mPrimitives ) );
        }
    }

	void dtkCollisionDetectHierarchy::AddPrimitive( Primitive* primitive )
	{
		primitive->mLocalID = (int)mPrimitives.size();
		mPrimitives.push_back( primitive );
	}

    dtkCollisionDetectPrimitive* dtkCollisionDetectHierarchy::InsertTriangle( dtkPoints::Ptr pts, dtkID3 ids )
    {
        Primitive* primitive = new Primitive( dtkCollisionDetectPrimitive::TRIANGLE, pts, ids[0], ids[1], ids[2] );
        AddPrimitive( primitive );
        return primitive;
    }

    dtkCollisionDetectPrimitive* dtkCollisionDetectHierarchy::InsertSegment( dtkPoints::Ptr pts, dtkID2 ids )
    {
        Primitive* primitive = new Primitive( dtkCollisionDetectPrimitive::SEGMENT, pts, ids[0], ids[1] );
		AddPrimitive( primitive );
        return primitive;
    }

	dtkCollisionDetectPrimitive* dtkCollisionDetectHierarchy::InsertSphere( dtkPoints::Ptr pts, dtkID id )
	{
		Primitive* primitive = new Primitive( dtkCollisionDetectPrimitive::SPHERE, pts, id );
		AddPrimitive( primitive );
		return primitive;
	}
        
    void dtkCollisionDetectHierarchy::InsertTriangleMesh( dtkStaticTriangleMesh::Ptr triMesh, dtkID majorID, double extend )
    {
		triMesh->Rebuild();
        dtkPoints::Ptr pts = triMesh->GetPoints();
        const std::vector<dtkID3>& ecTable = triMesh->GetECTable();       
        Primitive* primitive = 0;
        for( dtkID i = 0; i < ecTable.size(); i++ )  //对每个三角形
        {
            primitive = InsertTriangle( pts, ecTable[i] );
            primitive->mMajorID = majorID;
            primitive->mMinorID = i;
			primitive->SetExtend(extend);
			primitive->mDetailIDs[0] = ecTable[i][0];
			primitive->mDetailIDs[1] = ecTable[i][1];
			primitive->mDetailIDs[2] = ecTable[i][2];
        }
    }

    void dtkCollisionDetectHierarchy::InsertTetraMesh( dtkStaticTetraMesh::Ptr tetraMesh, dtkID majorID, InsertOption opt, double extend )
    {
        tetraMesh->Rebuild(); // 
        dtkPoints::Ptr pts = tetraMesh->GetPoints();
        const std::vector<dtkID4>& ecTable = tetraMesh->GetECTable();
        std::vector<dtkID> b2fTable = tetraMesh->GetB2FTable();
        Primitive* primitive = 0;
        std::set<dtkID3> tempFaceSet;
        std::set<dtkID3>::iterator setIt;
        std::vector<dtkID>::iterator vecIt;
        for( dtkID i = 0; i < ecTable.size(); i++ )
        {
            dtkID4 tetra = ecTable[i];
            dtkID3 tempSortedFace;
            dtkID3 tempOriFace;
            for( dtkID j = 0; j < 4; j++ )
            {
                tempOriFace = dtkID3(tetra[tetraMesh->EA2V[j][0]],tetra[tetraMesh->EA2V[j][1]],tetra[tetraMesh->EA2V[j][2]]);
                tempSortedFace = tetraMesh->SortTriVertex(tempOriFace[0],tempOriFace[1],tempOriFace[2]);
                setIt = tempFaceSet.find(tempSortedFace);
                bool findHf = false;
                for( vecIt = b2fTable.begin(); vecIt != b2fTable.end(); vecIt++)
                {
                    if( *vecIt == tetraMesh->EncodeAHF(i, j + 1, 0) )
                    {
                        findHf = true;
                        break;
                    }
                }
                if( setIt == tempFaceSet.end() ) //面没加入过
                {
                    if( ( opt == INTERIOR && !findHf ) || ( opt == SURFACE && findHf ) )
                    {
                        tempFaceSet.insert(tempSortedFace);
                        primitive = InsertTriangle( pts, tempOriFace);
                        primitive->SetExtend(extend);
                        primitive->mMajorID = majorID;
                        primitive->mMinorID = i;
                        primitive->mDetailIDs[0] = tempOriFace[0];
                        primitive->mDetailIDs[1] = tempOriFace[1];
                        primitive->mDetailIDs[2] = tempOriFace[2];
                    }
                }
            }
        }
    }

    void dtkCollisionDetectHierarchy::UpdateAllPrimitives()
    {
        if( mNumberOfThreads > 0 )
            _UpdateAllPrimitives_mt();
        else
            _UpdateAllPrimitives_s();
    }

    void dtkCollisionDetectHierarchy::_UpdateAllPrimitives_s()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[dtkCollisionDetectHierarchy::UpdateAllPrimitives]" << endl;
#endif
        for( dtkID i = 0; i < GetNumberOfPrimitives(); i++ )
            mPrimitives[i]->Update();
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[/dtkCollisionDetectHierarchy::UpdateAllPrimitives]" << endl;
        cout << endl;
#endif
    }
 
    void dtkCollisionDetectHierarchy::_UpdateAllPrimitives_mt()
    {
#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[dtkCollisionDetectHierarchy::UpdateAllPrimitives]" << endl;
#endif

#ifdef DTK_TBB
        // tbb
        static affinity_partitioner ap;
        parallel_for(blocked_range<size_t>(0,mPrimitives.size()), ApplyUpdate(&mPrimitives[0]), ap);
#else
		 //boost thread
		 mEnterBarrier->wait();
		 mExitBarrier->wait();
#endif

#ifdef DTKCOLLISIONDETECTHIERARCHY_DEBUG
        cout << "[/dtkCollisionDetectHierarchy::UpdateAllPrimitives]" << endl;
        cout << endl;
#endif
    }
        
    size_t dtkCollisionDetectHierarchy::GetNumberOfIntersectedPrimitives() const
    {
        size_t num = 0;
        for( dtkID i = 0; i < mPrimitives.size(); i++ )
        {
            if( mPrimitives[i]->IsIntersected() )
                num++;
        }
        return num;
    }
        
    void dtkCollisionDetectHierarchy::SetMaxLevel( size_t maxLevel )
    {
        mMaxLevel = maxLevel;
    }

	void dtkCollisionDetectHierarchy::AutoSetMaxLevel()
	{
		size_t maxLevel = 0;
		double num = GetNumberOfPrimitives() / 1.0;
		while( num >= 2 )
		{
			maxLevel++;
			//num = sqrt( num );
			num /= 2;
		}
		mMaxLevel = maxLevel;
		//mMaxLevel = 0;
	}
}

