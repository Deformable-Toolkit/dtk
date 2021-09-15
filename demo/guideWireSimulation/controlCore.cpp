#include "dtkStaticTriangleMeshReader.h"
#include "dtkStaticTetraMeshReader.h"


#define K 3

#include <set>
#include <queue>
#include <fstream>
#include <time.h>
#include <windows.h>
#include "controlCore.h"


using namespace std;
using namespace boost;

int test = 0;

namespace dtk
{
	controlCore::~controlCore()
	{
		// nothing
		mCount = 0;
	}

	void controlCore::Update( double timeslice, const vector<dtkID3> & avoid )
	{
		mAvoid = avoid;

		////--------------------------------------------
		LARGE_INTEGER litmp;
		LONGLONG qt1,qt2;
		double dft,dff,dfm;
		QueryPerformanceFrequency(&litmp);//获得时钟频率
		dff=(double)litmp.QuadPart;
		////--------------------------------------------

		mTimeslice = timeslice;

		//--------------------------------------------
		QueryPerformanceCounter(&litmp);//获得初始值
		qt1=litmp.QuadPart;
		//--------------------------------------------
		//mStage->GetHierarchy(1)->Update();
				
		// reset contact force 重新膨胀检测
		mCollisionDetectResponse->ResetContactForces();
		mCollisionDetectResponse->ResetCollisionFlag();

		// 第一次迭代
		for (map< dtkID, guideWire::Ptr>::iterator itr = mGuideWireMassPoints.begin();
			itr != mGuideWireMassPoints.end(); itr++)
		{
			if (timeslice == 0)
				continue;
			// 动态增删导丝质点
			itr->second->DynamicGuideWirePoint();
			itr->second->PreUpdate(timeslice);
			itr->second->Update(timeslice, Collision, 0);
		}

		UpdateGuideWireHierarchy(1, mGuideWireMassPoints[1]->GetPoints());

		QueryPerformanceCounter(&litmp);//获得终止值
		qt2=litmp.QuadPart;
		dfm=(double)(qt2-qt1);
		dft=dfm/dff * 1000;//获得对应的时间值
		mUpdateGuideWireTime = dft;

		// update hierarchy
		//yzk add 可能是用于检测碰撞
		mStage->Update();
		//yzk add end

		//--------------------------------------------
		QueryPerformanceCounter(&litmp);	//获得初始值
		qt1=litmp.QuadPart;
		//--------------------------------------------
		//yzk add start
		mStage->GetHierarchy(1)->Update();
		mCollisionDetectHierarchies[1]->Update();
		UpdateGuideWireHierarchy(1, mGuideWireMassPoints[1]->GetPoints());
		//yzk add end
			
		QueryPerformanceCounter(&litmp);	//获得终止值
		qt2=litmp.QuadPart;
		dfm=(double)(qt2-qt1);
		dft=dfm/dff * 1000;					//获得对应的时间值

		vector< dtkInterval<int> > emptyIntervals;

		// 测试碰撞检测的计算时间 start
		QueryPerformanceCounter(&litmp);//获得初始值
		qt1=litmp.QuadPart;

		// 创建碰撞检测对
		mCollisionDetectResponseSets.clear();
		CreateCollisionResponse(0, 1, 2000);

		for( map< dtkID, CollisionResponseSet >::iterator itr = mCollisionDetectResponseSets.begin();
			itr != mCollisionDetectResponseSets.end(); itr++ )
		{
			//mCount = 0;
			vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
			mStage->DoIntersect( itr->second.hierarchy_pair,  intersectResults,false, false );
			mCollisionDetectResponse->Update( timeslice, intersectResults, avoid, (itr->second).strength);
		}
		// 测试碰撞检测的计算时间 end
		QueryPerformanceCounter(&litmp);//获得终止值
		qt2=litmp.QuadPart;
		dfm=(double)(qt2-qt1);
		dft=dfm/dff * 1000;//获得对应的时间值
		mCollisionDetectTime = dft;

		// 第二次迭代 ? 为什么这么做
		for (map< dtkID, guideWire::Ptr>::iterator itr = mGuideWireMassPoints.begin();
			itr != mGuideWireMassPoints.end(); itr++)
		{
			if (timeslice == 0)
				continue;
			itr->second->DynamicGuideWirePoint();
			itr->second->PreUpdate(timeslice);
			itr->second->Update(timeslice, Collision, 1);
		}
	}

	void controlCore::CreateMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel, double specialExtend )
	{
		dtkPoints::Ptr targetPts = dtkPointsVector::New();

		dtkPhysMassSpring::Ptr massSpring = dtkPhysMassSpring::New( point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel );
		massSpring->SetPoints( targetPts );
		mMassSprings[id] = massSpring;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );

		std::ifstream file(filename);
		size_t numOfPts;
		dtkID maxID;

		file >> numOfPts >> maxID;

		for (size_t i = 0; i < numOfPts; ++i)
		{
			dtkID id;
			dtkFloat3 coord;

			file >> id >> coord.x >> coord.y >> coord.z;
			targetPts->SetPoint(id, GK::Point3(coord.x, coord.y, coord.z));
		}

		for( dtkID i = 0; i < targetPts->GetNumberOfPoints(); i++ )
		{
			massSpring->AddMassPoint( i, point_mass, dtkT3<double>(0,0,0), pointDamp, pointResistence, gravityAccel );
		}

		size_t numOfEdges;
		file >> numOfEdges;
		for (size_t i = 0; i < numOfEdges; ++i)
		{
			dtkID id0, id1;

			file >> id0 >> id1;

			massSpring->AddSpring( id0, id1, stiffness, damp );
			dtkCollisionDetectPrimitive* primitive = mCollisionDetectHierarchies[id]->InsertSegment( targetPts, dtkID2( id0, id1 ) );
			primitive->SetExtend( specialExtend );
			primitive->mMajorID = id;
			primitive->mMinorID = i;
			primitive->mDetailIDs[0] = id0;
			primitive->mDetailIDs[1] = id1;
		}

		file.close();

		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy( mCollisionDetectHierarchies[id] );
		mCollisionDetectResponse->SetMassSpring( id, mMassSprings[id] );
	}


	void controlCore::CreateTriangleMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel )
	{
		//没有被用到 yzk added
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		dtkStaticTriangleMesh::Ptr targetMesh = dtkStaticTriangleMesh::New();
		targetMesh->SetPoints(targetPts);

		dtkStaticTriangleMeshReader::Ptr reader_trianglemesh = dtkStaticTriangleMeshReader::New();
		reader_trianglemesh->SetOutput(targetMesh);
		reader_trianglemesh->SetFileName( filename );
		reader_trianglemesh->Read();

		dtkPhysMassSpring::Ptr triangleMassSpring = dtkPhysMassSpring::New( point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel );
		triangleMassSpring->SetTriangleMesh( targetMesh );
		mMassSprings[id] = triangleMassSpring;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );
		mCollisionDetectHierarchies[id]->InsertTriangleMesh( targetMesh, id );
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy( mCollisionDetectHierarchies[id] );
		mCollisionDetectResponse->SetMassSpring( id, mMassSprings[id] );

		mTriangleMeshes[id] = targetMesh;
	}


	void controlCore::CreateTetraMassSpring( const char* filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkT3<double> gravityAccel)
	{
		// Target Mesh
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		dtkStaticTetraMesh::Ptr targetMesh = dtkStaticTetraMesh::New();
		targetMesh->SetPoints(targetPts);
		//读取三角面片
		dtkStaticTetraMeshReader::Ptr reader_tetramesh = dtkStaticTetraMeshReader::New();
		reader_tetramesh->SetOutput(targetMesh);
		reader_tetramesh->SetFileName( filename );
		reader_tetramesh->Read();

		mTetraMeshes[id] = targetMesh;
		//构造弹簧质点模型
		dtkPhysTetraMassSpring::Ptr tetraMassSpring = dtkPhysTetraMassSpring::New( false, point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel );
		tetraMassSpring->SetTetraMesh(targetMesh);
		mMassSprings[id] = tetraMassSpring;
		mTetraMassSprings[id] = tetraMassSpring;
		//构造碰撞模型
		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );
		mCollisionDetectHierarchies[id]->InsertTetraMesh( targetMesh, id, dtkCollisionDetectHierarchy::SURFACE, mClothDepth );
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();
		mCollisionDetectHierarchies[id]->Update();

		mStage->AddHierarchy( mCollisionDetectHierarchies[id] );
		// get the surface mesh of tetra 
		dtkStaticTriangleMesh::Ptr surface = dtkStaticTriangleMesh::New();
		targetMesh->GetSurface(surface);
		mTriangleMeshes[id] = surface;
		mCollisionDetectResponse = collisionResponse::New(surface);
		mCollisionDetectResponse->SetMassSpring( id, mMassSprings[id] );
	}

	void controlCore::CreateTriangleSurfaceMesh(const char* filename, dtkID id)
	{
		// Target Mesh
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		dtkStaticTriangleMesh::Ptr targetMesh = dtkStaticTriangleMesh::New();
		targetMesh->SetPoints(targetPts);
		//读取三角面片
		dtkStaticTriangleMeshReader::Ptr reader_tetramesh = dtkStaticTriangleMeshReader::New();
		reader_tetramesh->SetOutput(targetMesh);
		reader_tetramesh->SetFileName( filename );
		reader_tetramesh->Read();

		mTriangleMeshes[id] = targetMesh;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );
		mCollisionDetectHierarchies[id]->InsertTriangleMesh( targetMesh, id, mClothDepth);
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();
		mCollisionDetectHierarchies[id]->Update();

		mStage->AddHierarchy( mCollisionDetectHierarchies[id] );
		mCollisionDetectResponse = collisionResponse::New(targetMesh);
	}

	void controlCore::CreateCollisionResponse( dtkID object1_id, dtkID object2_id, double strength)
	{
		dtkID response_id = object1_id * mPairOffset + object2_id;

		CollisionResponseSet newset;
		newset.hierarchy_pair = dtkCollisionDetectStage::HierarchyPair(GetCollisionDetectHierarchy( object1_id), GetCollisionDetectHierarchy( object2_id) );
		newset.strength = strength;
		mCollisionDetectResponseSets[response_id] = newset;
	}

	void controlCore::UpdateCollisionResponse(dtkID object1_id, dtkID object2_id, double strength)
	{
		dtkID response_id = object1_id * mPairOffset + object2_id;
		mCollisionDetectResponseSets[response_id].hierarchy_pair = dtkCollisionDetectStage::HierarchyPair(GetCollisionDetectHierarchy( object1_id), GetCollisionDetectHierarchy( object2_id) );
		mCollisionDetectResponseSets[response_id].strength = strength;
	}

   void controlCore::CreateGuideWire(dtkID id, dtkPoints::Ptr points,  dtkID lastTipID, double segInterval, double tipSegInterval, const std::vector<double> & tipOriginAngle)
	{
		/*	if (points->GetNumberOfPoints() != curvatures.size())
		assert(false);*/
		guideWire::Ptr guidewire = guideWire::New();
		guidewire->SetPoints(points);
		guidewire->SetLastTipID(lastTipID);
		guidewire->SetTipOriginAngle(tipOriginAngle);
		guidewire->SetSegInterval(segInterval);
		guidewire->SetTipSegInterval(tipSegInterval);
		guidewire->SetMass(2.0);
		guidewire->SetPointResistence(36.0 * guidewire->GetMass());			// 设置导丝体部部分的摩擦系数
		guidewire->SetTipPointResistence(24.0 * guidewire->GetMass());		// 设置导丝尖端部分的摩擦系数
		guidewire->SetBendModulus(240.0 * 4.0 * 0.05);						// 240.0 * 4.0  0.07
		guidewire->SetTipBendModulus(240.0 * 4.0 * 0.02);					// 360 * 4.0 * 0.1
		guidewire->Set3DBendModulus(360.0 * 4.0 * 0.1);						// 30000
		mGuideWireMassPoints[id] = guidewire;
		mGuideWireMassPoints[id]->ConstructGuideWireMassPoints();

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );

		for (dtkID i = 0; i < points->GetNumberOfPoints(); i++)
		{
			dtkCollisionDetectPrimitive * primitive;
			if (i < points->GetNumberOfPoints() )
			{
				primitive = mCollisionDetectHierarchies[id]->InsertSphere(points, i);
				primitive->mMajorID = id;
				primitive->mMinorID = i;
				primitive->mDetailIDs[0] = i;
				primitive->mInvert = 1;
			}
		}

		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy( mCollisionDetectHierarchies[id] );
		mCollisionDetectResponse->SetGuideWire(id, guidewire);
	}

	void controlCore::UpdateGuideWireHierarchy(dtkID id, dtkPoints::Ptr points)
	{
		//	clean guide wire hierarchy
		//	delete mCollisionDetectHierarchies[id];
		//	rebuild 
		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New( K );
		for (dtkID i = 0; i < points->GetNumberOfPoints(); i++)
		{
			dtkCollisionDetectPrimitive * primitive;
			if (i < points->GetNumberOfPoints() )
			{
				primitive = mCollisionDetectHierarchies[id]->InsertSphere(points, i);
				primitive->mMajorID = id;
				primitive->mMinorID = i;
				primitive->mDetailIDs[0] = i;
				primitive->mInvert = 0;
			}
		}
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();
		mCollisionDetectHierarchies[id]->Update();
	}

	dtkPhysMassSpring::Ptr controlCore::GetMassSpring( dtkID id )
	{
		if( mMassSprings.find(id) == mMassSprings.end() )
			assert( false );

		return mMassSprings[id]; 
	}

	dtkPhysMassSpring::Ptr controlCore::GetTetraMassSpring( dtkID id )
	{
		if( mTetraMassSprings.find(id) == mTetraMassSprings.end() )
			assert( false );

		return mTetraMassSprings[id]; 
	}

	dtkCollisionDetectHierarchyKDOPS::Ptr controlCore::GetCollisionDetectHierarchy( dtkID id)
	{
		if( mCollisionDetectHierarchies.find(id) == mCollisionDetectHierarchies.end() )
			assert( false );
		return mCollisionDetectHierarchies[id];
	}

	dtkStaticTriangleMesh::Ptr controlCore::GetTriangleMesh( dtkID id )
	{
		if( mTriangleMeshes.find(id) == mTriangleMeshes.end() )
			assert( false );
		return mTriangleMeshes[id]; 
	}

	controlCore::controlCore( double clothDepth)
	{
		mStage = dtkCollisionDetectStage::New();
		mClothDepth = clothDepth;

		// test
		responseTag = true;

		// collision detect time
		mCollisionDetectTime = 0;
	}

	void controlCore::ApplyExternalForce(dtkID id, dtkT3<double> force)
	{
		mExternalForce = force;
		//如果到末端
		if ( mGuideWireMassPoints.find(id) == mGuideWireMassPoints.end() )
			assert(false);

		if ((mGuideWireMassPoints[id]->GetPoints()->GetMaxID() <= 2 * (mGuideWireMassPoints[id]->GetLastTipID() + 1)) && force.y < 0)
			return;
		mGuideWireMassPoints[id]->ApplyExternalForce(force);
	}

	void controlCore::ApplyExternalTwist(dtkID id, double twist)
	{
		// 导丝的体部进行力的计算
		if (mGuideWireMassPoints.find(id) == mGuideWireMassPoints.end())
			assert(false);
		mGuideWireMassPoints[id]->AddTwistForce(twist);

		// 导丝的尖端进行旋转
		dtkID lastTipID = mGuideWireMassPoints[id]->GetLastTipID();
		for (dtkID i = 0; i < lastTipID; i++)
		{
			mGuideWireMassPoints[id]->RotateMassPoint(mGuideWireMassPoints[id]->GetPoints(), i, lastTipID + 1, lastTipID, twist);
		}
	}

	dtkT3<double> controlCore::GetHapticTranslationForce()
	{
		// assume the max ID of guidewire is 400
		double coefficient = 1.0 /( 400 * 400 * 2);
		return coefficient * (mGuideWireMassPoints[1]->GetPoints()->GetMaxID() + 1) * mExternalForce;
	}

	dtkT3<double> controlCore::GetHapticCollisionForce()
	{
		double coefficient = mTimeslice * mTimeslice * 2.0;
		return dtkT3<double>(0, length(mGuideWireMassPoints[1]->mContactForces[0]) * coefficient, 0);
	}
}