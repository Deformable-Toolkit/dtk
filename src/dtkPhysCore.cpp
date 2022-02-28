#include "dtkPhysCore.h"
#include "dtkStaticTriangleMeshReader.h"
#include "dtkStaticTetraMeshReader.h"

#define K 3

#include <set>
#include <queue>
#include <fstream>
using namespace std;
using namespace boost;

namespace dtk
{
	void dtkphyscore_mt_update(dtkID id, dtkPhysCore *core)
	{
		vector<dtkInterval<int>> emptyIntervals;

		do
		{	
			/*
			//const std::vector< dtkID >* massSpringRange = &( core->mAllocator[id][0] );
			//const std::vector< dtkID >* collisionPairRange = &( core->mAllocator[id][1] );

			//const std::vector< dtkID >& massSpringRange = core->mAllocator[id][0];
			//const std::vector< dtkID >& collisionPairRange = core->mAllocator[id][1];

			// Phase 1
			*/
			core->mEnterBarrier->wait();

			if (!core->mLive)
				break;

			std::vector<dtkID> massSpringRange;
			for (dtkID i = 0; i < core->mAllocator[id][core->mAllocatePosMassSpring].size(); i++)
				massSpringRange.push_back(core->mAllocator[id][core->mAllocatePosMassSpring][i]);

			std::vector<dtkID> primitiveRange;
			for (dtkID i = 0; i < core->mAllocator[id][core->mAllocatePosPrimitive].size(); i++)
				primitiveRange.push_back(core->mAllocator[id][core->mAllocatePosPrimitive][i]);

			std::vector<dtkID> collisionPairRange;
			for (dtkID i = 0; i < core->mAllocator[id][core->mAllocatePosCollisionDetect].size(); i++)
				collisionPairRange.push_back(core->mAllocator[id][core->mAllocatePosCollisionDetect][i]);

			std::vector<dtkID> internalCollisionPairRange;
			for (dtkID i = 0; i < core->mAllocator[id][core->mAllocatePosInternalCollisionDetect].size(); i++)
				internalCollisionPairRange.push_back(core->mAllocator[id][core->mAllocatePosInternalCollisionDetect][i]);

			for (dtkID i = 0; i < massSpringRange.size(); i++)
			{
				if (core->mTimeslice == 0 || core->mMassSprings[massSpringRange[i]]->IsUnderControl())
					continue;
				core->mMassSprings[massSpringRange[i]]->PreUpdate(core->mTimeslice, Collision, 0);
				core->mMassSprings[massSpringRange[i]]->UpdateStrings(core->mTimeslice, Collision, 0, true);
			}

			// Phase 2
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < massSpringRange.size(); i++)
			{
				if (core->mTimeslice == 0 || core->mMassSprings[massSpringRange[i]]->IsUnderControl())
					continue;
				core->mMassSprings[massSpringRange[i]]->TransportForce(core->mTimeslice);
				core->mMassSprings[massSpringRange[i]]->UpdateMassPoints(core->mTimeslice, Collision, 0);
				//core->mMassSprings[ massSpringRange[i] ]->PostUpdate(Collision, 0);
			}

			// Phase 2.1
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < primitiveRange.size(); i++)
			{
				dtkID type = primitiveRange[i] / core->mPairOffset;
				dtkID id = primitiveRange[i] % core->mPairOffset;
				switch (type)
				{
				case 0:
					core->mCollisionDetectHierarchies[id]->Update();
					break;
				case 1:
					core->mThreadCollisionDetectHierarchies[id]->Update();
					break;
				case 2:
					core->mInteriorCollisionDetectHierarchies[id]->Update();
					break;
				case 3:
					core->mThreadHeadCollisionDetectHierarchies[id]->Update();
					break;
				default:
					break;
				}

			}

			// Phase 2.2
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < collisionPairRange.size(); i++)
			{
				vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
				core->mStage->DoIntersect(core->mCollisionDetectResponseSets[collisionPairRange[i]].hierarchy_pair,
										  intersectResults, core->mCollisionDetectResponseSets[collisionPairRange[i]].self, false);
				if (core->mCollisionDetectResponseSets[collisionPairRange[i]].responseType == dtkPhysCore::THREAD_SURFACE)
				{
					core->mCollisionDetectResponse->Update(core->mTimeslice, intersectResults,
														   emptyIntervals,
														   core->mThreadCollisionDetectResponse->GetAvoidIntervals(collisionPairRange[i] % core->mPairOffset),
														   core->mCollisionDetectResponseSets[collisionPairRange[i]].strength);
				}
				else if (core->mCollisionDetectResponseSets[collisionPairRange[i]].responseType == dtkPhysCore::KNOTPLANNING)
				{
					core->mCollisionDetectResponse->Update(core->mTimeslice, intersectResults,
														   core->mKnotPlanners[collisionPairRange[i] % core->mPairOffset]->GetAvoidIntervals(),
														   core->mKnotPlanners[collisionPairRange[i] % core->mPairOffset]->GetAvoidIntervals(), core->mCollisionDetectResponseSets[collisionPairRange[i]].strength);
				}
				else if (core->mCollisionDetectResponseSets[collisionPairRange[i]].responseType == dtkPhysCore::INTERIOR_THREADHEAD)
				{
					assert(false);
				}
				else
				{
					core->mCollisionDetectResponse->Update(core->mTimeslice, intersectResults,
														   emptyIntervals, emptyIntervals, core->mCollisionDetectResponseSets[collisionPairRange[i]].strength);
				}

				if (core->mCollisionDetectResponseSets[collisionPairRange[i]].responseType == dtkPhysCore::KNOTPLANNING)
				{
					core->mKnotPlanners[collisionPairRange[i] / core->mPairOffset]->KnotRecognition(intersectResults);
				}

				if (intersectResults.size() != 0 && core->mCollisionDetectResponseSets[collisionPairRange[i]].custom_handle != 0)
					core->mCollisionDetectResponseSets[collisionPairRange[i]].custom_handle(intersectResults, core->mCollisionDetectResponseSets[collisionPairRange[i]].pContext);
			}

			// Phase 2.3
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < internalCollisionPairRange.size(); i++)
			{
				vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
				core->mStage->DoIntersect(core->mInternalCollisionDetectResponseSets[internalCollisionPairRange[i]].hierarchy_pair,
										  intersectResults, core->mInternalCollisionDetectResponseSets[internalCollisionPairRange[i]].self, false);
				core->mThreadCollisionDetectResponse->Update(core->mTimeslice, intersectResults);

				const std::vector<dtkInterval<int>> &internalIntervals = core->mThreadCollisionDetectResponse->GetInternalIntervals(internalCollisionPairRange[i] % core->mPairOffset);
				dtkCollisionDetectHierarchyKDOPS::Ptr threadHierarchy = core->mThreadCollisionDetectHierarchies[internalCollisionPairRange[i] % core->mPairOffset];
				for (dtkID n = 0; n < threadHierarchy->GetNumberOfPrimitives(); n++)
				{
					threadHierarchy->GetPrimitive(n)->mInvert = 0;
				}
				for (dtkID n = 0; n < internalIntervals.size(); n++)
				{
					for (int l = internalIntervals[n][0]; l <= internalIntervals[n][1]; l++)
					{
						threadHierarchy->GetPrimitive(l)->mInvert = 1;
					}
				}
			}

			// Phase 3
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < massSpringRange.size(); i++)
			{
				if (core->mTimeslice == 0)
					continue;

				if (core->mMassSprings[massSpringRange[i]]->IsUnderControl())
				{
					core->mMassSprings[massSpringRange[i]]->ConvertImpulseToForce(core->mTimeslice);
					continue;
				}

				core->mMassSprings[massSpringRange[i]]->ApplyImpulse(core->mTimeslice);
				core->mMassSprings[massSpringRange[i]]->PreUpdate(core->mTimeslice, Collision, 1);
				core->mMassSprings[massSpringRange[i]]->UpdateStrings(core->mTimeslice, Collision, 1, true);
			}

			//exit_barrier->wait();

			// Phase 4
			core->mEnterBarrier->wait();

			for (dtkID i = 0; i < massSpringRange.size(); i++)
			{
				if (core->mTimeslice == 0 || core->mMassSprings[massSpringRange[i]]->IsUnderControl())
					continue;
				core->mMassSprings[massSpringRange[i]]->UpdateMassPoints(core->mTimeslice, Collision, 1);
				core->mMassSprings[massSpringRange[i]]->PostUpdate(Collision, 1);
			}

			core->mExitBarrier->wait();
		} while (true);
	}

	dtkPhysCore::dtkPhysCore(double clothDepth)
	{
		mStage = dtkCollisionDetectStage::New();
		mCollisionDetectResponse = dtkPhysMassSpringCollisionResponse::New();
		mThreadCollisionDetectResponse = dtkPhysMassSpringThreadCollisionResponse::New(mCollisionDetectResponse);
		mStaticMeshEliminator = dtkStaticMeshEliminator::New();

		mNumberOfThreads = 0;
		mThreadGroup = 0;
		mEnterBarrier = 0;
		mExitBarrier = 0;

		mClothDepth = clothDepth;
	}

	dtkPhysCore::~dtkPhysCore()
	{
		if (mNumberOfThreads > 0)
		{
			mLive = false;
			mEnterBarrier->wait();
			mThreadGroup->join_all();
			delete mThreadGroup;
			delete mEnterBarrier;
			delete mExitBarrier;
		}
	}

	void dtkPhysCore::Update(double timeslice)
	{
		if (mNumberOfThreads > 1)
			_Update_mt(timeslice);
		else
			_Update_s(timeslice);
	}

	void dtkPhysCore::_Update_s(double timeslice)
	{
		mTimeslice = timeslice;

		// update mass-spring model:iteration 0
		for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			if (timeslice == 0 || itr->second->IsUnderControl())
				continue;
			itr->second->PreUpdate(timeslice, Collision, 0);
			itr->second->UpdateStrings(timeslice, Collision, 0, true);
		}
		for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			if (timeslice == 0 || itr->second->IsUnderControl())
				continue;
			itr->second->TransportForce(timeslice);
			itr->second->UpdateMassPoints(timeslice, Collision, 0);
			itr->second->PostUpdate(Collision, 0);
		}

		// update hierachy
		mStage->Update();

		// update collision response result
		vector<dtkInterval<int>> emptyIntervals;
		for (map<dtkID, CollisionResponseSet>::iterator itr = mCollisionDetectResponseSets.begin();
			 itr != mCollisionDetectResponseSets.end(); itr++)
		{
			vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
			mStage->DoIntersect(itr->second.hierarchy_pair,
								intersectResults, itr->second.self, false);
			if (itr->second.responseType == THREAD_SURFACE)
			{
				mCollisionDetectResponse->Update(timeslice, intersectResults,
												 emptyIntervals,
												 mThreadCollisionDetectResponse->GetAvoidIntervals(itr->first % mPairOffset),
												 itr->second.strength);
			}
			else if (itr->second.responseType == KNOTPLANNING)
			{
				mCollisionDetectResponse->Update(timeslice, intersectResults,
												 mKnotPlanners[itr->first % mPairOffset]->GetAvoidIntervals(),
												 mKnotPlanners[itr->first % mPairOffset]->GetAvoidIntervals(), itr->second.strength);
			}
			else if (itr->second.responseType == INTERIOR_THREADHEAD)
			{
				assert(false);
			}
			else
			{
				mCollisionDetectResponse->Update(timeslice, intersectResults,
												 emptyIntervals, emptyIntervals, itr->second.strength);
			}

			if (itr->second.responseType == KNOTPLANNING)
			{
				mKnotPlanners[itr->first / mPairOffset]->KnotRecognition(intersectResults);
			}

			if (intersectResults.size() != 0 && itr->second.custom_handle != 0)
				itr->second.custom_handle(intersectResults, itr->second.pContext);
		}

		// update internal collision response result
		for (map<dtkID, CollisionResponseSet>::iterator itr = mInternalCollisionDetectResponseSets.begin();
			 itr != mInternalCollisionDetectResponseSets.end(); itr++)
		{
			vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
			mStage->DoIntersect(itr->second.hierarchy_pair,
								intersectResults, itr->second.self, false);
			mThreadCollisionDetectResponse->Update(timeslice, intersectResults);

			const std::vector<dtkInterval<int>> &internalIntervals = mThreadCollisionDetectResponse->GetInternalIntervals(itr->first % mPairOffset);
			dtkCollisionDetectHierarchyKDOPS::Ptr threadHierarchy = mThreadCollisionDetectHierarchies[itr->first % mPairOffset];
			for (dtkID n = 0; n < threadHierarchy->GetNumberOfPrimitives(); n++)
			{
				threadHierarchy->GetPrimitive(n)->mInvert = 0;
			}
			for (dtkID n = 0; n < internalIntervals.size(); n++)
			{
				for (int l = internalIntervals[n][0]; l <= internalIntervals[n][1]; l++)
				{
					threadHierarchy->GetPrimitive(l)->mInvert = 1;
				}
			}
		}

		// apply impulse into the mass points and update mass-spring model iteration : 1
		for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			if (timeslice == 0)
				continue;

			if (itr->second->IsUnderControl())
			{
				itr->second->ConvertImpulseToForce(timeslice);
				continue;
			}

			itr->second->ApplyImpulse(timeslice);
			itr->second->PreUpdate(timeslice, Collision, 1);
			itr->second->UpdateStrings(timeslice, Collision, 1, true);
		}
		for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			if (timeslice == 0 || itr->second->IsUnderControl())
				continue;
			itr->second->UpdateMassPoints(timeslice, Collision, 1);
			itr->second->PostUpdate(Collision, 1);
		}

		for (dtkID i = 0; i < mAdherePointSets.size(); i++)
		{
			AdherePointSet &adherePointSet = mAdherePointSets[i];
			GK::Point3 p_tri = barycenter(adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[0]), adherePointSet.uvw[0],
										  adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[1]), adherePointSet.uvw[1],
										  adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[2]), adherePointSet.uvw[2]);
			GK::Point3 p_p = adherePointSet.slave_pts->GetPoint(adherePointSet.slave_p);
			GK::Vector3 normal = p_p - p_tri;

			dtkPhysMassSpring::Ptr dominate_ms = mMassSprings[adherePointSet.dominate_ID];
			dtkPhysMassSpring::Ptr slave_ms = mMassSprings[adherePointSet.slave_ID];

			for (dtkID j = 0; j < 3; j++)
			{
				dtkPhysMassPoint *dominate_mp = dominate_ms->GetMassPoint(adherePointSet.dominate_tri[j]);
				GK::Vector3 dominate_vel = normal * adherePointSet.slave_ratio * adherePointSet.uvw[j];
				GK::Point3 tmp_p = adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[j]) + dominate_vel;
				dominate_mp->SetPoint(tmp_p);
				dominate_vel = dominate_vel * (1 / mTimeslice);
				dominate_mp->SetVel(dominate_mp->GetVel() + dtkT3<double>(dominate_vel[0], dominate_vel[1], dominate_vel[2]));
			}

			dtkPhysMassPoint *slave_mp = slave_ms->GetMassPoint(adherePointSet.slave_p);
			GK::Vector3 slave_vel = -normal * (1.0 - adherePointSet.slave_ratio);
			GK::Point3 tmp_p = adherePointSet.slave_pts->GetPoint(adherePointSet.slave_p) + slave_vel;
			slave_mp->SetPoint(tmp_p);
			slave_vel = slave_vel * (1 / mTimeslice);
			slave_mp->SetVel(slave_mp->GetVel() + dtkT3<double>(slave_vel[0], slave_vel[1], slave_vel[2]));
		}

		for (map<dtkID, dtkPhysParticleSystem::Ptr>::iterator itr = mParticleSystems.begin();
			 itr != mParticleSystems.end(); itr++)
		{
			if (timeslice == 0)
				continue;
			itr->second->Update(timeslice);
		}

		for (map<dtkID, ObstacleSet>::iterator itr = mObstacleSets.begin();
			 itr != mObstacleSets.end(); itr++)
		{
			vector<dtkIntersectTest::IntersectResult::Ptr> particleIntersectResult;
			if (timeslice == 0)
				continue;
			for (dtkID i = 0; i < itr->second.particlesystem->GetNumberOfParticles(); i++)
			{
				GK::Point3 particle = itr->second.particlesystem->GetPoint(i);
				itr->second.pts->SetPoint(0, particle);
				itr->second.hierarchy_pair.second->Update();
				vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
				mStage->DoIntersect(itr->second.hierarchy_pair,
									intersectResults, false, false);
				for (dtkID j = 0; j < intersectResults.size(); j++)
				{
					dtkIntersectTest::IntersectResult::Ptr result = intersectResults[j];
					GK::Vector3 normal;
					result->GetProperty(dtkIntersectTest::INTERSECT_NORMAL, normal);
					itr->second.particlesystem->SetPoint(i, particle + normal);
					itr->second.particlesystem->GetParticle(i)->AddForce(dtkDouble3(-normal[0], -normal[1], -normal[2]) * itr->second.viscosityCoef);

					particleIntersectResult.push_back(intersectResults[j]);
				}
			}
			if (particleIntersectResult.size() != 0 && itr->second.custom_handle != 0)
				itr->second.custom_handle(particleIntersectResult, itr->second.pContext);
		}

		for (std::map<dtkID, std::vector<dtkID>>::iterator itr_device = mDeviceLabels.begin();
			 itr_device != mDeviceLabels.end(); itr_device++)
		{
			dtkT3<double> forceFeedback(0, 0, 0);
			for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr_ms = mMassSprings.begin();
				 itr_ms != mMassSprings.end(); itr_ms++)
			{
				forceFeedback = forceFeedback + itr_ms->second->GetTransportForce(itr_device->first);
			}
			if (forceFeedback[0] == 0 && forceFeedback[1] == 0 && forceFeedback[2] == 0)
			{
				for (dtkID i = 0; i < itr_device->second.size(); i++)
				{
					forceFeedback = forceFeedback + mMassSprings[itr_device->second[i]]->GetImpulseForce();
				}
			}
			else
			{
				forceFeedback = forceFeedback * 1.0;
			}
			mDeviceForceFeedbacks[itr_device->first] = forceFeedback;
		}

		for (map<dtkID, dtkPhysMassSpringThread::Ptr>::iterator itr = mSutureThreads.begin();
			 itr != mSutureThreads.end(); itr++)
		{
			mKnotPlanners[itr->first]->DoKnotFormation();
			mKnotPlanners[itr->first]->UpdateKnot(timeslice);
		}

		for (map<dtkID, dtkPhysMassSpringThread::Ptr>::iterator itr = mSutureThreads.begin();
			 itr != mSutureThreads.end(); itr++)
		{
			vector<dtkDouble3> controlPoints;

			double interval = itr->second->GetInterval() * 0.5;

			for (dtkID i = 0; i < itr->second->GetNumberOfSegments() + 1; i++)
			{
				dtkDouble3 curPoint = itr->second->GetMassPoint(i * 2)->GetPosition();
				dtkDouble3 smoothedDirection(0, 0, 0);
				int num = 0;
				if (i != 0)
				{
					smoothedDirection = smoothedDirection + curPoint - itr->second->GetMassPoint(i * 2 - 2)->GetPosition();
					num++;
				}
				if (i != itr->second->GetNumberOfSegments())
				{
					smoothedDirection = smoothedDirection + itr->second->GetMassPoint(i * 2 + 2)->GetPosition() - curPoint;
					num++;
				}
				if (num == 0)
					assert(false);
				smoothedDirection = normalize(smoothedDirection);
				controlPoints.push_back(curPoint - smoothedDirection * interval);
				controlPoints.push_back(curPoint);
				controlPoints.push_back(curPoint + smoothedDirection * interval);
			}

			dtkID mSmoothedNumber = 10;
			for (dtkID i = 0; i < itr->second->GetNumberOfSegments(); i++)
			{
				double step = 1.0 / (double)mSmoothedNumber;
				double t;
				for (dtkID j = 0; j < mSmoothedNumber; j++)
				{
					t = step * j;
					dtkDouble3 smoothedPoint =
						controlPoints[i * 3 + 1] * ((1 - t) * (1 - t) * (1 - t)) + controlPoints[i * 3 + 2] * (3.0 * t * (1 - t) * (1 - t)) + controlPoints[i * 3 + 3] * (3.0 * t * t * (1 - t)) + controlPoints[i * 3 + 4] * (t * t * t);
					mThreadPoints[itr->first]->SetPoint(i * mSmoothedNumber + j, GK::Point3(smoothedPoint[0], smoothedPoint[1], smoothedPoint[2]));
				}
			}
		}
	}

	void dtkPhysCore::SetNumberOfThreads(size_t n)
	{
		if (n < 2)
			return;
		mNumberOfThreads = n;
		mLive = true;
		mThreadGroup = new thread_group();
		mEnterBarrier = new barrier(mNumberOfThreads + 1);
		mExitBarrier = new barrier(mNumberOfThreads + 1);

		Reallocate();

		// create threads
		for (dtkID i = 0; i < mNumberOfThreads; i++)
		{
			mThreadGroup->add_thread(new boost::thread(dtkphyscore_mt_update, i, this
													   //mNumberOfThreads, &(mAllocator[i]),
													   //mEnterBarrier, mExitBarrier, &mLive,
													   //&mMassSprings, &mCollisionDetectHierarchies, &mCollisionDetectResponseSets,
													   //&mStage, &mCollisionDetectResponse, &mTimeslice
													   ));
		}
	}

	// allocate different id group to different thread.
	dtkID dtkPhysCore::AllocateDetails(const multimap<dtkID, dtkID, greater<dtkID>> &sortedMap, size_t average)
	{
		vector<size_t> allocatedNumberOfMassSpringPoints;
		for (dtkID i = 0; i < mNumberOfThreads; i++)
		{
			allocatedNumberOfMassSpringPoints.push_back(0);
			mAllocator[i].push_back(std::vector<dtkID>());
		}
		dtkID posInAllocator = mAllocator[0].size() - 1;
		for (std::multimap<dtkID, dtkID, greater<dtkID>>::const_iterator itr = sortedMap.begin();
			 itr != sortedMap.end(); itr++)
		{
			dtkID allocator = 0;
			int bestMatch = dtkIntMax;
			for (dtkID j = 0; j < mNumberOfThreads; j++)
			{
				int match = abs((int)allocatedNumberOfMassSpringPoints[j] + (int)itr->first - (int)average);
				if (match < bestMatch)
				{
					bestMatch = match;
					allocator = j;
				}
			}
			mAllocator[allocator][posInAllocator].push_back(itr->second);
			allocatedNumberOfMassSpringPoints[allocator] += itr->first;
		}
		return posInAllocator;
	}

	void dtkPhysCore::Reallocate()
	{
		mAllocator.clear();
		for (dtkID i = 0; i < mNumberOfThreads; i++)
		{
			mAllocator.push_back(std::vector<std::vector<dtkID>>());
		}

		// 0: allocate mass spring
		size_t sumOfMassSpringPoints = 0;
		multimap<dtkID, dtkID, greater<dtkID>> sortedMassSprings;
		for (std::map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			dtkID num = 0;
			if (mConnectMasterMap.find(itr->first) != mConnectMasterMap.end())
			{
				set<dtkID> bundle = mConnectMasterMap[itr->first];
				for (set<dtkID>::iterator bundleItr = bundle.begin();
					 bundleItr != bundle.end(); bundleItr++)
				{
					num += mMassSprings[*bundleItr]->GetNumberOfMassPoints();
				}
			}
			else if (mConnectedMassSpring.find(itr->first) != mConnectedMassSpring.end())
			{
				continue;
			}
			else
			{
				num = itr->second->GetNumberOfMassPoints();
			}
			sumOfMassSpringPoints += num;
			sortedMassSprings.insert(pair<dtkID, dtkID>(num, itr->first));
		}

		size_t averageOfMassSpringPoints = sumOfMassSpringPoints / mNumberOfThreads;

		mAllocatePosMassSpring = AllocateDetails(sortedMassSprings, averageOfMassSpringPoints);

		for (dtkID i = 0; i < mNumberOfThreads; i++)
		{
			vector<dtkID> &massSprings = mAllocator[i][mAllocatePosMassSpring];
			vector<dtkID> newMassSprings;
			for (dtkID j = 0; j < massSprings.size(); j++)
			{
				if (mConnectMasterMap.find(massSprings[j]) != mConnectMasterMap.end())
				{
					set<dtkID> &bundle = mConnectMasterMap[massSprings[j]];
					newMassSprings.insert(newMassSprings.end(), bundle.begin(), bundle.end());
				}
				else
				{
					newMassSprings.push_back(massSprings[j]);
				}
			}
			mAllocator[i][mAllocatePosMassSpring] = newMassSprings;
		}

		// 1: allocate collision primitives update
		size_t sumOfPrimitives = 0;
		multimap<dtkID, dtkID, greater<dtkID>> sortedPrimitives;
		for (std::map<dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr>::iterator itr = mCollisionDetectHierarchies.begin();
			 itr != mCollisionDetectHierarchies.end(); itr++)
		{
			dtkID num = itr->second->GetNumberOfPrimitives();
			sumOfPrimitives += num;
			sortedPrimitives.insert(pair<dtkID, dtkID>(num, itr->first + mPairOffset * 0));
		}
		for (std::map<dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr>::iterator itr = mThreadCollisionDetectHierarchies.begin();
			 itr != mThreadCollisionDetectHierarchies.end(); itr++)
		{
			dtkID num = itr->second->GetNumberOfPrimitives();
			sumOfPrimitives += num;
			sortedPrimitives.insert(pair<dtkID, dtkID>(num, itr->first + mPairOffset * 1));
		}
		for (std::map<dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr>::iterator itr = mInteriorCollisionDetectHierarchies.begin();
			 itr != mInteriorCollisionDetectHierarchies.end(); itr++)
		{
			dtkID num = itr->second->GetNumberOfPrimitives();
			sumOfPrimitives += num;
			sortedPrimitives.insert(pair<dtkID, dtkID>(num, itr->first + mPairOffset * 2));
		}
		for (std::map<dtkID, dtkCollisionDetectHierarchyKDOPS::Ptr>::iterator itr = mThreadHeadCollisionDetectHierarchies.begin();
			 itr != mThreadHeadCollisionDetectHierarchies.end(); itr++)
		{
			dtkID num = itr->second->GetNumberOfPrimitives();
			sumOfPrimitives += num;
			sortedPrimitives.insert(pair<dtkID, dtkID>(num, itr->first + mPairOffset * 3));
		}

		size_t averageOfPrimitives = sumOfPrimitives / mNumberOfThreads;

		mAllocatePosPrimitive = AllocateDetails(sortedPrimitives, averageOfPrimitives);

		// 2: allocate collision detect
		size_t sumOfCollisionDetectPairs = 0;
		multimap<dtkID, dtkID, greater<dtkID>> sortedCollisionDetectResponseSets;
		for (std::map<dtkID, CollisionResponseSet>::iterator itr = mCollisionDetectResponseSets.begin();
			 itr != mCollisionDetectResponseSets.end(); itr++)
		{
			dtkID num = itr->second.hierarchy_pair.first->GetNumberOfPrimitives() * itr->second.hierarchy_pair.second->GetNumberOfPrimitives();
			sumOfCollisionDetectPairs += num;
			sortedCollisionDetectResponseSets.insert(pair<dtkID, dtkID>(num, itr->first));
		}
		size_t averageOfCollisionDetectPairs = sumOfCollisionDetectPairs / mNumberOfThreads;

		mAllocatePosCollisionDetect = AllocateDetails(sortedCollisionDetectResponseSets, averageOfCollisionDetectPairs);

		// 3: allocate internal collision detect
		size_t sumOfInternalCollisionDetectPairs = 0;
		multimap<dtkID, dtkID, greater<dtkID>> sortedInternalCollisionDetectResponseSets;
		for (std::map<dtkID, CollisionResponseSet>::iterator itr = mInternalCollisionDetectResponseSets.begin();
			 itr != mInternalCollisionDetectResponseSets.end(); itr++)
		{
			dtkID num = itr->second.hierarchy_pair.first->GetNumberOfPrimitives() * itr->second.hierarchy_pair.second->GetNumberOfPrimitives();
			sumOfInternalCollisionDetectPairs += num;
			sortedInternalCollisionDetectResponseSets.insert(pair<dtkID, dtkID>(num, itr->first));
		}
		size_t averageOfInternalCollisionDetectPairs = sumOfInternalCollisionDetectPairs / mNumberOfThreads;

		mAllocatePosInternalCollisionDetect = AllocateDetails(sortedInternalCollisionDetectResponseSets, averageOfInternalCollisionDetectPairs);

		return;
	}

	void dtkPhysCore::_Update_mt(double timeslice)
	{
		mTimeslice = timeslice;

		// Phase 1
		mEnterBarrier->wait();

		// Phase 2
		mEnterBarrier->wait();

		// Phase 2.1
		mEnterBarrier->wait();

		// Phase 2.2
		mEnterBarrier->wait();

		// Phase 2.3
		mEnterBarrier->wait();

		// Phase 3
		mEnterBarrier->wait();

		// Phase 4
		mEnterBarrier->wait();
		mExitBarrier->wait();

		for (dtkID i = 0; i < mAdherePointSets.size(); i++)
		{
			AdherePointSet &adherePointSet = mAdherePointSets[i];

			//重心 
			GK::Point3 p_tri = barycenter(adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[0]), adherePointSet.uvw[0],
										  adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[1]), adherePointSet.uvw[1],
										  adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[2]), adherePointSet.uvw[2]);
			GK::Point3 p_p = adherePointSet.slave_pts->GetPoint(adherePointSet.slave_p);
			GK::Vector3 normal = p_p - p_tri;

			dtkPhysMassSpring::Ptr dominate_ms = mMassSprings[adherePointSet.dominate_ID];
			dtkPhysMassSpring::Ptr slave_ms = mMassSprings[adherePointSet.slave_ID];

			//主三角形点更新 

			for (dtkID j = 0; j < 3; j++)
			{
				dtkPhysMassPoint *dominate_mp = dominate_ms->GetMassPoint(adherePointSet.dominate_tri[j]);
				GK::Vector3 dominate_vel = normal * adherePointSet.slave_ratio * adherePointSet.uvw[j];

				GK::Point3 tmp_p = adherePointSet.dominate_pts->GetPoint(adherePointSet.dominate_tri[j]) + dominate_vel;
				dominate_mp->SetPoint(tmp_p);
				dominate_vel = dominate_vel * (1 / mTimeslice);
				dominate_mp->SetVel(dominate_mp->GetVel() + dtkT3<double>(dominate_vel[0], dominate_vel[1], dominate_vel[2]));
			}

			//从点更新
			dtkPhysMassPoint *slave_mp = slave_ms->GetMassPoint(adherePointSet.slave_p);
			GK::Vector3 slave_vel = -normal * (1.0 - adherePointSet.slave_ratio);

			GK::Point3 tmp_p = adherePointSet.slave_pts->GetPoint(adherePointSet.slave_p) + slave_vel;
			slave_mp->SetPoint(tmp_p);
			slave_vel = slave_vel * (1 / mTimeslice);
			slave_mp->SetVel(slave_mp->GetVel() + dtkT3<double>(slave_vel[0], slave_vel[1], slave_vel[2]));
		}

		for (map<dtkID, dtkPhysParticleSystem::Ptr>::iterator itr = mParticleSystems.begin();
			 itr != mParticleSystems.end(); itr++)
		{
			if (timeslice == 0)
				continue;
			itr->second->Update(timeslice);
		}

		for (map<dtkID, ObstacleSet>::iterator itr = mObstacleSets.begin();
			 itr != mObstacleSets.end(); itr++)
		{
			vector<dtkIntersectTest::IntersectResult::Ptr> particleIntersectResult;
			if (timeslice == 0)
				continue;
			for (dtkID i = 0; i < itr->second.particlesystem->GetNumberOfParticles(); i++)
			{
				GK::Point3 particle = itr->second.particlesystem->GetPoint(i);
				itr->second.pts->SetPoint(0, particle);

				//更新包围盒
				itr->second.hierarchy_pair.second->Update();
				vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;

				//kDOPS相交测试
				mStage->DoIntersect(itr->second.hierarchy_pair,
									intersectResults, false, false);
				for (dtkID j = 0; j < intersectResults.size(); j++)
				{
					//更新粒子
					dtkIntersectTest::IntersectResult::Ptr result = intersectResults[j];
					GK::Vector3 normal;
					result->GetProperty(dtkIntersectTest::INTERSECT_NORMAL, normal);
					itr->second.particlesystem->SetPoint(i, particle + normal);
					itr->second.particlesystem->GetParticle(i)->AddForce(dtkDouble3(-normal[0], -normal[1], -normal[2]) * itr->second.viscosityCoef);

					particleIntersectResult.push_back(intersectResults[j]);
				}
			}
			if (particleIntersectResult.size() != 0 && itr->second.custom_handle != 0)
				itr->second.custom_handle(particleIntersectResult, itr->second.pContext);
		}

		for (std::map<dtkID, std::vector<dtkID>>::iterator itr_device = mDeviceLabels.begin();
			 itr_device != mDeviceLabels.end(); itr_device++)
		{
			dtkT3<double> forceFeedback(0, 0, 0);
			for (map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr_ms = mMassSprings.begin();
				 itr_ms != mMassSprings.end(); itr_ms++)
			{
				forceFeedback = forceFeedback + itr_ms->second->GetTransportForce(itr_device->first);
			}
			if (forceFeedback[0] == 0 && forceFeedback[1] == 0 && forceFeedback[2] == 0)
			{
				for (dtkID i = 0; i < itr_device->second.size(); i++)
				{
					forceFeedback = forceFeedback + mMassSprings[itr_device->second[i]]->GetImpulseForce();
				}
			}
			else
			{
				forceFeedback = forceFeedback * 1.0;
			}
			mDeviceForceFeedbacks[itr_device->first] = forceFeedback;
		}

		for (map<dtkID, dtkPhysMassSpringThread::Ptr>::iterator itr = mSutureThreads.begin();
			 itr != mSutureThreads.end(); itr++)
		{
			mKnotPlanners[itr->first]->DoKnotFormation();
			mKnotPlanners[itr->first]->UpdateKnot(timeslice);
		}

		for (map<dtkID, dtkPhysMassSpringThread::Ptr>::iterator itr = mSutureThreads.begin();
			 itr != mSutureThreads.end(); itr++)
		{
			vector<dtkDouble3> controlPoints;

			double interval = itr->second->GetInterval() * 0.5;

			for (dtkID i = 0; i < itr->second->GetNumberOfSegments() + 1; i++)
			{
				dtkDouble3 curPoint = itr->second->GetMassPoint(i * 2)->GetPosition();
				dtkDouble3 smoothedDirection(0, 0, 0);
				int num = 0;
				if (i != 0)
				{
					smoothedDirection = smoothedDirection + curPoint - itr->second->GetMassPoint(i * 2 - 2)->GetPosition();
					num++;
				}
				if (i != itr->second->GetNumberOfSegments())
				{
					smoothedDirection = smoothedDirection + itr->second->GetMassPoint(i * 2 + 2)->GetPosition() - curPoint;
					num++;
				}
				if (num == 0)
					assert(false);
				smoothedDirection = normalize(smoothedDirection);
				controlPoints.push_back(curPoint - smoothedDirection * interval);
				controlPoints.push_back(curPoint);
				controlPoints.push_back(curPoint + smoothedDirection * interval);
			}

			dtkID mSmoothedNumber = 10;
			for (dtkID i = 0; i < itr->second->GetNumberOfSegments(); i++)
			{
				double step = 1.0 / (double)mSmoothedNumber;
				double t;
				for (dtkID j = 0; j < mSmoothedNumber; j++)
				{
					t = step * j;
					dtkDouble3 smoothedPoint =
						controlPoints[i * 3 + 1] * ((1 - t) * (1 - t) * (1 - t)) + controlPoints[i * 3 + 2] * (3.0 * t * (1 - t) * (1 - t)) + controlPoints[i * 3 + 3] * (3.0 * t * t * (1 - t)) + controlPoints[i * 3 + 4] * (t * t * t);
					mThreadPoints[itr->first]->SetPoint(i * mSmoothedNumber + j, GK::Point3(smoothedPoint[0], smoothedPoint[1], smoothedPoint[2]));
				}
			}
		}
	}

	void dtkPhysCore::CreateSutureThread(dtkID id, double length, dtkDouble3 firstPointPos,
										 dtkPhysMassSpringThread::Orientation orientation, double mass,
										 double edgeStiff, double bendStiff, double torsionStiff,
										 double edgeDamp, double extraEdgeDamp, double bendDamp, double torsionDamp,
										 double interval, double radius, double selfCollisionStrength)
	{
		dtkID numberOfSegments = static_cast<dtkID>(length / interval) + 1;

		mSutureThreads[id] = dtkPhysMassSpringThread::New(interval, numberOfSegments, firstPointPos, orientation, mass,
														  edgeStiff, bendStiff, torsionStiff, edgeDamp, extraEdgeDamp, bendDamp, torsionDamp);

		mThreadCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);
		mThreadHeadCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);

		dtkCollisionDetectPrimitive *pri;
		for (dtkID i = 0; i < mSutureThreads[id]->GetNumberOfSegments(); i++)
		{
			pri = mThreadCollisionDetectHierarchies[id]->InsertSegment(mSutureThreads[id]->GetPoints(), dtkID2(i * 2, i * 2 + 2));
			pri->SetExtend(radius);
			pri->mMajorID = id;
			pri->mMinorID = i;
			pri->mDetailIDs[0] = i * 2;
			pri->mDetailIDs[1] = i * 2 + 2;
		}

		pri = mThreadHeadCollisionDetectHierarchies[id]->InsertSegment(mSutureThreads[id]->GetPoints(), dtkID2(0, 2));
		pri->SetExtend(0);
		pri->mMajorID = id;
		pri->mMinorID = 0;
		pri->mDetailIDs[0] = 0;
		pri->mDetailIDs[1] = 2;

		mThreadCollisionDetectHierarchies[id]->Build();
		mThreadHeadCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy(mThreadCollisionDetectHierarchies[id]);
		mStage->AddHierarchy(mThreadHeadCollisionDetectHierarchies[id]);

		mCollisionDetectResponse->SetMassSpring(id, mSutureThreads[id]);
		mCollisionDetectResponse->AddPierceSegment(id, 0);

		mThreadCollisionDetectResponse->SetThread(id, mSutureThreads[id]);

		mMassSprings[id] = mSutureThreads[id];

		mKnotPlanners[id] = dtkPhysKnotPlanner::New(mSutureThreads[id]);

		mThreadPoints[id] = dtkPointsVector::New(numberOfSegments * 10);

		CreateCollisionResponse(id, THREAD, id, THREAD, selfCollisionStrength);
	}

	void dtkPhysCore::CreateTetraMassSpring(const char *filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel, /*bool isSurface, */
											dtkStaticMeshEliminator::MeshEliminatorResultsCallback callback, void *pContext)
	{
		// Target Mesh
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		dtkStaticTetraMesh::Ptr targetMesh = dtkStaticTetraMesh::New();
		targetMesh->SetPoints(targetPts);

		dtkStaticTetraMeshReader::Ptr reader_tetramesh = dtkStaticTetraMeshReader::New();
		reader_tetramesh->SetOutput(targetMesh);
		reader_tetramesh->SetFileName(filename);
		reader_tetramesh->Read();

		mTetraMeshes[id] = targetMesh;

		dtkPhysTetraMassSpring::Ptr tetraMassSpring = dtkPhysTetraMassSpring::New(false, point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel);
		tetraMassSpring->SetTetraMesh(targetMesh);
		mMassSprings[id] = tetraMassSpring;
		mTetraMassSprings[id] = tetraMassSpring;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);
		mCollisionDetectHierarchies[id]->InsertTetraMesh(targetMesh, id, dtkCollisionDetectHierarchy::SURFACE, mClothDepth);
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectResponse->SetMassSpring(id, mMassSprings[id]);

		//获取表面
		dtkStaticTriangleMesh::Ptr surface = dtkStaticTriangleMesh::New();
		targetMesh->GetSurface(surface);
		mTriangleMeshes[id] = surface;

		mStaticMeshEliminator->AddElimniateTarget(id, mTriangleMeshes[id], targetMesh,
												  mCollisionDetectHierarchies[id], tetraMassSpring, callback, pContext);

		mAdhereCounts.insert(pair<dtkID, map<dtkID, size_t>>(id, map<dtkID, size_t>()));

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	void dtkPhysCore::CreateTriangleMassSpring(const char *filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel)
	{
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		dtkStaticTriangleMesh::Ptr targetMesh = dtkStaticTriangleMesh::New();
		targetMesh->SetPoints(targetPts);

		dtkStaticTriangleMeshReader::Ptr reader_trianglemesh = dtkStaticTriangleMeshReader::New();
		reader_trianglemesh->SetOutput(targetMesh);
		reader_trianglemesh->SetFileName(filename);
		reader_trianglemesh->Read();

		dtkPhysMassSpring::Ptr triangleMassSpring = dtkPhysMassSpring::New(point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel);
		triangleMassSpring->SetTriangleMesh(targetMesh);
		mMassSprings[id] = triangleMassSpring;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);
		mCollisionDetectHierarchies[id]->InsertTriangleMesh(targetMesh, id);
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectResponse->SetMassSpring(id, mMassSprings[id]);

		mTriangleMeshes[id] = targetMesh;

		mAdhereCounts.insert(pair<dtkID, map<dtkID, size_t>>(id, map<dtkID, size_t>()));

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	// build a primitive by some edges.
	void dtkPhysCore::CreateMassSpring(const char *filename, dtkID id, double point_mass, double stiffness, double damp, double pointDamp, double pointResistence, dtkDouble3 gravityAccel, double specialExtend)
	{
		dtkPoints::Ptr targetPts = dtkPointsVector::New();

		dtkPhysMassSpring::Ptr massSpring = dtkPhysMassSpring::New(point_mass, stiffness, damp, pointDamp, pointResistence, gravityAccel);
		massSpring->SetPoints(targetPts);

		mMassSprings[id] = massSpring;

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);

		std::ifstream file(filename);
		size_t numOfPts;
		dtkID maxID;

		file >> numOfPts >> maxID;

		// the size of targetPts ???
		for (size_t i = 0; i < numOfPts; ++i)
		{
			dtkID id;
			dtkFloat3 coord;

			file >> id >> coord.x >> coord.y >> coord.z;
			targetPts->SetPoint(id, GK::Point3(coord.x, coord.y, coord.z));
		}

		for (dtkID i = 0; i < targetPts->GetNumberOfPoints(); i++)
		{
			massSpring->AddMassPoint(i, point_mass, dtkT3<double>(0, 0, 0), pointDamp, pointResistence, gravityAccel);
		}

		size_t numOfEdges;

		file >> numOfEdges;

		// the mCollisionDetectHierarcies have only segment ???
		for (size_t i = 0; i < numOfEdges; ++i)
		{
			dtkID id0, id1;

			file >> id0 >> id1;

			massSpring->AddSpring(id0, id1, stiffness, damp);
			dtkCollisionDetectPrimitive *primitive = mCollisionDetectHierarchies[id]->InsertSegment(targetPts, dtkID2(id0, id1));
			primitive->SetExtend(specialExtend);
			primitive->mMajorID = id;
			primitive->mMinorID = i;
			primitive->mDetailIDs[0] = id0;
			primitive->mDetailIDs[1] = id1;
		}

		file.close();

		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		mStage->AddHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectResponse->SetMassSpring(id, mMassSprings[id]);

		mAdhereCounts.insert(pair<dtkID, map<dtkID, size_t>>(id, map<dtkID, size_t>()));

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	dtkPhysMassSpring::Ptr dtkPhysCore::GetMassSpring(dtkID id)
	{
		if (mMassSprings.find(id) == mMassSprings.end())
			assert(false);

		return mMassSprings[id];
	}

	dtkPhysMassSpring::Ptr dtkPhysCore::GetTetraMassSpring(dtkID id)
	{
		if (mTetraMassSprings.find(id) == mTetraMassSprings.end())
			assert(false);

		return mTetraMassSprings[id];
	}

	dtkPhysMassSpringThread::Ptr dtkPhysCore::GetMassSpringThread(dtkID id)
	{
		if (mSutureThreads.find(id) == mSutureThreads.end())
			assert(false);

		return mSutureThreads[id];
	}

	dtkStaticTriangleMesh::Ptr dtkPhysCore::GetTriangleMesh(dtkID id)
	{
		if (mTriangleMeshes.find(id) == mTriangleMeshes.end())
			assert(false);

		return mTriangleMeshes[id];
	}

	dtkCollisionDetectHierarchyKDOPS::Ptr dtkPhysCore::GetCollisionDetectHierarchy(dtkID id, CollisionHierarchyType type)
	{
		switch (type)
		{
		case SURFACE:
			if (mCollisionDetectHierarchies.find(id) == mCollisionDetectHierarchies.end())
				assert(false);

			return mCollisionDetectHierarchies[id];
		case THREAD:
			if (mThreadCollisionDetectHierarchies.find(id) == mThreadCollisionDetectHierarchies.end())
				assert(false);

			return mThreadCollisionDetectHierarchies[id];
		case INTERIOR:
			if (mInteriorCollisionDetectHierarchies.find(id) == mInteriorCollisionDetectHierarchies.end())
				assert(false);

			return mInteriorCollisionDetectHierarchies[id];
		case THREADHEAD:
			if (mThreadHeadCollisionDetectHierarchies.find(id) == mThreadHeadCollisionDetectHierarchies.end())
				assert(false);

			return mThreadHeadCollisionDetectHierarchies[id];
		default:
			assert(false);
		}
	}

	dtkPhysMassSpringThreadCollisionResponse::Ptr dtkPhysCore::GetThreadCollisionResponse()
	{

		return mThreadCollisionDetectResponse;
	}

	dtkPhysKnotPlanner::Ptr dtkPhysCore::GetKnotPlanner(dtkID plannerID)
	{
		if (mKnotPlanners.find(plannerID) == mKnotPlanners.end())
			assert(false);

		return mKnotPlanners[plannerID];
	}

	void dtkPhysCore::CreateCollisionResponse(dtkID object1_id, dtkID object2_id, double strength,
											  void (*custom_handle)(const std::vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults, void *pContext), void *pContext)
	{
		CreateCollisionResponse(object1_id, SURFACE, object2_id, SURFACE, strength, custom_handle, pContext);
	}

	void dtkPhysCore::CreateCollisionResponse(dtkID object1_id, CollisionHierarchyType obj1_type, dtkID object2_id, CollisionHierarchyType obj2_type,
											  double strength, void (*custom_handle)(const vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults, void *pContext), void *pContext)
	{
		dtkID response_id = object1_id * mPairOffset + object2_id;

		CollisionResponseSet newset;
		newset.hierarchy_pair = dtkCollisionDetectStage::HierarchyPair(
			GetCollisionDetectHierarchy(object1_id, obj1_type), GetCollisionDetectHierarchy(object2_id, obj2_type));
		newset.strength = strength;
		newset.custom_handle = custom_handle;
		newset.pContext = pContext;
		newset.self = (object1_id == object2_id);

		bool isInterior = false;
		if (newset.self && obj1_type == THREAD)
		{
			newset.responseType = KNOTPLANNING;
		}
		else if (!newset.self &&
				 ((obj1_type == THREAD && obj2_type == SURFACE) || (obj1_type == SURFACE && obj2_type == THREAD)))
		{
			newset.responseType = THREAD_SURFACE;
		}
		else if (obj1_type == INTERIOR && obj2_type == THREADHEAD)
		{
			newset.responseType = INTERIOR_THREADHEAD;
			isInterior = true;
		}

		if (!isInterior)
			mCollisionDetectResponseSets[response_id] = newset;
		else
			mInternalCollisionDetectResponseSets[response_id] = newset;

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	void dtkPhysCore::CreateSutureResponse(dtkID object_id, dtkID thread_id,
										   double strength, void (*custom_handle)(const vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults, void *pContext), void *pContext)
	{
		//create hierarchies for object interior
		mInteriorCollisionDetectHierarchies[object_id] = dtkCollisionDetectHierarchyKDOPS::New(K);
		mInteriorCollisionDetectHierarchies[object_id]->InsertTetraMesh(mTetraMeshes[object_id], object_id, dtkCollisionDetectHierarchy::INTERIOR, 0);
		mInteriorCollisionDetectHierarchies[object_id]->AutoSetMaxLevel();
		mInteriorCollisionDetectHierarchies[object_id]->Build();

		mStage->AddHierarchy(mInteriorCollisionDetectHierarchies[object_id]);
		mCollisionDetectResponse->SetMassSpring(object_id, mMassSprings[object_id]);

		CreateCollisionResponse(object_id, SURFACE, thread_id, THREAD, strength, custom_handle, pContext);
		CreateCollisionResponse(object_id, INTERIOR, thread_id, THREADHEAD, 0);
	}

	dtkPoints::Ptr dtkPhysCore::CreateCustomDetectTriangleArea(dtkID id, double half_width)
	{
		dtkPoints::Ptr targetPts = dtkPointsVector::New();
		targetPts->SetPoint(0, GK::Point3(0, 0, 0));
		targetPts->SetPoint(1, GK::Point3(1, 0, 0));
		targetPts->SetPoint(2, GK::Point3(0, 1, 0));

		mCollisionDetectHierarchies[id] = dtkCollisionDetectHierarchyKDOPS::New(K);
		dtkCollisionDetectPrimitive *triangle = mCollisionDetectHierarchies[id]->InsertTriangle(targetPts, dtkID3(0, 1, 2));
		triangle->mMajorID = id;
		triangle->mMinorID = 0;
		triangle->mDetailIDs[0] = 0;
		triangle->mDetailIDs[1] = 1;
		triangle->mDetailIDs[2] = 2;
		triangle->mInvert = 2;
		triangle->SetExtend(half_width);
		mCollisionDetectHierarchies[id]->AutoSetMaxLevel();
		mCollisionDetectHierarchies[id]->Build();

		//mStage->AddHierarchy( mCollisionDetectHierarchies[id] );

		return targetPts;
	}

	void dtkPhysCore::ExecuteCustomDetectTriangleArea(dtkID id, dtkID targetID,
													  void (*custom_handle)(const std::vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults, void *pContext), void *pContext)
	{
		vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
		mCollisionDetectHierarchies[id]->Update();
		mStage->DoIntersect(dtkCollisionDetectStage::HierarchyPair(mCollisionDetectHierarchies[targetID], mCollisionDetectHierarchies[id]),
							intersectResults, false, false);
		if (intersectResults.size() != 0)
			custom_handle(intersectResults, pContext);
	}

	size_t dtkPhysCore::ConnectMassSpring(dtkID object1_id, dtkID object2_id, double range)
	{
		size_t num = mMassSprings[object1_id]->FindTwins(mMassSprings[object2_id], range);
		if (num > 0)
		{
			mConnectMap.push_back(dtkID2(object1_id, object2_id));
			mConnectedMassSpring.insert(object1_id);
			mConnectedMassSpring.insert(object2_id);
			RebundleConnectedMassSpring();

			if (mNumberOfThreads > 1)
				Reallocate();
		}
		return num;
	}

	void dtkPhysCore::RebundleConnectedMassSpring()
	{
		mConnectMasterMap.clear();

		set<dtkID> remainMassSprings;
		for (std::map<dtkID, dtkPhysMassSpring::Ptr>::iterator itr = mMassSprings.begin();
			 itr != mMassSprings.end(); itr++)
		{
			remainMassSprings.insert(itr->first);
		}
		while (!remainMassSprings.empty())
		{
			set<dtkID> possibleBundle;
			queue<dtkID> bundleFront;
			dtkID startPoint = *(remainMassSprings.begin());
			remainMassSprings.erase(startPoint);
			possibleBundle.insert(startPoint);
			bundleFront.push(startPoint);
			while (!bundleFront.empty())
			{
				dtkID curFront = bundleFront.front();
				bundleFront.pop();
				for (dtkID i = 0; i < mConnectMap.size(); i++)
				{
					if (curFront == mConnectMap[i][0] && possibleBundle.find(mConnectMap[i][1]) == possibleBundle.end())
					{
						dtkID newFront = mConnectMap[i][1];
						remainMassSprings.erase(newFront);
						possibleBundle.insert(newFront);
						bundleFront.push(newFront);
					}
					else if (curFront == mConnectMap[i][1] && possibleBundle.find(mConnectMap[i][0]) == possibleBundle.end())
					{
						dtkID newFront = mConnectMap[i][0];
						remainMassSprings.erase(newFront);
						possibleBundle.insert(newFront);
						bundleFront.push(newFront);
					}
				}
			}

			if (possibleBundle.size() > 1)
			{
				mConnectMasterMap[*(possibleBundle.begin())] = possibleBundle;
			}
		}
	}

	size_t dtkPhysCore::AdhereMassSpring(dtkID from_id, dtkID to_id, double range)
	{
		dtkCollisionDetectHierarchyKDOPS::Ptr hierarchy_points;
		hierarchy_points = dtkCollisionDetectHierarchyKDOPS::New(K);
		dtkPoints::Ptr pts = mMassSprings[from_id]->GetPoints();
		for (dtkID i = 0; i < pts->GetNumberOfPoints(); i++)
		{
			dtkCollisionDetectPrimitive *primitive = hierarchy_points->InsertSphere(pts, i);
			primitive->mMajorID = from_id;
			primitive->mMinorID = i;
			primitive->mDetailIDs[0] = i;
			primitive->SetExtend(range - mClothDepth);
		}
		hierarchy_points->AutoSetMaxLevel();
		hierarchy_points->Build();

		vector<dtkIntersectTest::IntersectResult::Ptr> intersectResults;
		mStage->DoIntersect(dtkCollisionDetectStage::HierarchyPair(mCollisionDetectHierarchies[to_id], hierarchy_points),
							intersectResults, false, false);

		for (dtkID i = 0; i < intersectResults.size(); i++)
		{
			dtkIntersectTest::IntersectResult::Ptr result = intersectResults[i];
			dtkCollisionDetectPrimitive *pri_1;
			dtkCollisionDetectPrimitive *pri_2;
			result->GetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1);
			result->GetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2);

			AdherePointSet newset;
			newset.dominate_pts = mMassSprings[pri_1->mMajorID]->GetPoints();
			newset.dominate_tri[0] = pri_1->mDetailIDs[0];
			newset.dominate_tri[1] = pri_1->mDetailIDs[1];
			newset.dominate_tri[2] = pri_1->mDetailIDs[2];
			newset.dominate_ID = pri_1->mMajorID;
			newset.dominate_triID = pri_1->mMinorID;
			map<dtkID, size_t> &countMap = mAdhereCounts[newset.dominate_ID];
			map<dtkID, size_t>::iterator triItr = countMap.find(newset.dominate_triID);
			if (triItr != countMap.end())
			{
				triItr->second++;
			}
			else
			{
				countMap.insert(pair<dtkID, size_t>(newset.dominate_triID, 1));
			}
			newset.slave_pts = mMassSprings[pri_2->mMajorID]->GetPoints();
			newset.slave_p = pri_2->mDetailIDs[0];
			newset.slave_ID = pri_2->mMajorID;
			result->GetProperty(dtkIntersectTest::INTERSECT_WEIGHT_1, newset.uvw);
			mAdherePointSets.push_back(newset);
		}

		AdjustAdhereStatus();

		return intersectResults.size();
	}

	void dtkPhysCore::AdjustAdhereStatus()
	{
		for (dtkID i = 0; i < mAdherePointSets.size(); i++)
		{
			dtkPhysCore::AdherePointSet &adherePointSet = mAdherePointSets[i];
			adherePointSet.slave_ratio = 1.0 - pow(0.5, 1.0 / (double)mAdhereCounts[adherePointSet.dominate_ID][adherePointSet.dominate_triID]);
		}
	}

	dtkPhysParticleSystem::Ptr dtkPhysCore::CreateParticleSystem(dtkID id, double particleRadius, double particleMass, double particleLifetime)
	{
		mParticleSystems[id] = dtkPhysParticleSystem::New(particleRadius - mClothDepth, particleMass, particleLifetime);

		return mParticleSystems[id];
	}

	void dtkPhysCore::CreateObstacleForParticleSystem(dtkID particlesystem_id, dtkID object_id, double viscosityCoef,
													  void (*custom_handle)(const std::vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults, void *pContext), void *pContext)
	{
		dtkID obstacleid = particlesystem_id * mPairOffset + object_id;

		dtkCollisionDetectHierarchyKDOPS::Ptr hierarchy_points;
		hierarchy_points = dtkCollisionDetectHierarchyKDOPS::New(K);

		dtkPoints::Ptr pts = dtkPointsVector::New(1);
		dtkCollisionDetectPrimitive *primitive = hierarchy_points->InsertSphere(pts, 0);
		primitive->mMajorID = obstacleid;
		primitive->mMinorID = 0;
		primitive->mDetailIDs[0] = 0;
		primitive->SetExtend(mParticleSystems[particlesystem_id]->GetParticleRadius());

		hierarchy_points->AutoSetMaxLevel();
		hierarchy_points->Build();

		ObstacleSet newset;
		newset.pts = pts;
		newset.particlesystem = mParticleSystems[particlesystem_id];
		newset.hierarchy_pair = dtkCollisionDetectStage::HierarchyPair(mCollisionDetectHierarchies[object_id], hierarchy_points);
		newset.viscosityCoef = viscosityCoef;
		newset.custom_handle = custom_handle;
		newset.pContext = pContext;

		mObstacleSets[obstacleid] = newset;
	}

	void dtkPhysCore::DestroyMassSpring(dtkID id)
	{
		mMassSprings.erase(id);

		mStage->RemoveHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectHierarchies.erase(id);

		mCollisionDetectResponse->RemoveMassSpring(id);

		if (mNumberOfThreads >= 2)
			Reallocate();
	
	}

	void dtkPhysCore::DestroyTriangleMassSpring(dtkID id)
	{
		mMassSprings.erase(id);

		mStage->RemoveHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectHierarchies.erase(id);

		mCollisionDetectResponse->RemoveMassSpring(id);

		mTriangleMeshes.erase(id);

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	void dtkPhysCore::DestroyTetraMassSpring(dtkID id)
	{
		mMassSprings.erase(id);
		mTetraMassSprings.erase(id);

		dtkCollisionDetectHierarchy::Ptr hierarchy = mCollisionDetectHierarchies[id];
		mStage->RemoveHierarchy(hierarchy);

		vector<dtkID> deleteIDs;
		for (std::map<dtkID, CollisionResponseSet>::iterator itr = mCollisionDetectResponseSets.begin();
			 itr != mCollisionDetectResponseSets.end(); itr++)
		{
			if (itr->second.hierarchy_pair.first == hierarchy || itr->second.hierarchy_pair.second == hierarchy)
				deleteIDs.push_back(itr->first);
		}
		for (dtkID i = 0; i < deleteIDs.size(); i++)
			mCollisionDetectResponseSets.erase(deleteIDs[i]);
		mCollisionDetectHierarchies.erase(id);

		mCollisionDetectResponse->RemoveMassSpring(id);

		mTetraMeshes.erase(id);
		mTriangleMeshes.erase(id);

		mStaticMeshEliminator->RemoveElimniateTarget(id);

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	void dtkPhysCore::DestroyCollisionResponse(dtkID object1_id, dtkID object2_id)
	{
		DestroyCollisionResponse(object1_id, SURFACE, object2_id, SURFACE);
	}

	void dtkPhysCore::DestroyCollisionResponse(dtkID object1_id, CollisionHierarchyType obj1_type, dtkID object2_id, CollisionHierarchyType obj2_type)
	{
		dtkID response_id = object1_id * mPairOffset + object2_id;

		bool isInterior = false;
		if (obj1_type == INTERIOR && obj2_type == THREADHEAD)
		{
			isInterior = true;
		}

		if (!isInterior)
			mCollisionDetectResponseSets.erase(response_id);
		else
			mInternalCollisionDetectResponseSets.erase(response_id);

		if (mNumberOfThreads >= 2)
			Reallocate();
	}

	void dtkPhysCore::DestroyCustomDetectTriangleArea(dtkID id)
	{
		mStage->RemoveHierarchy(mCollisionDetectHierarchies[id]);
		mCollisionDetectHierarchies.erase(id);
	}

	void dtkPhysCore::DestroyParticleSystem(dtkID id)
	{
		mParticleSystems.erase(id);
	}

	void dtkPhysCore::DestroyObstacleForParticleSystem(dtkID particlesystem_id, dtkID object_id)
	{
		dtkID obstacleid = particlesystem_id * mPairOffset + object_id;
		mObstacleSets.erase(obstacleid);
	}

	void dtkPhysCore::DisconnectMassSpring(dtkID object1_id, dtkID object2_id)
	{
		mMassSprings[object1_id]->AbandonTwins();
		mMassSprings[object2_id]->AbandonTwins();
	}

	void dtkPhysCore::DetachAllMassSpring()
	{
		mAdherePointSets.clear();
	}

	void dtkPhysCore::EliminateTriangles(dtkID id, size_t originalFaceNum, std::vector<dtkID3> &collisionFace, bool oldMethod)
	{
		mStaticMeshEliminator->EliminateTriangles(id, originalFaceNum, collisionFace, oldMethod);
	}

	void dtkPhysCore::RegisterDevice(dtkID deviceLabel)
	{
		mDeviceLabels[deviceLabel] = std::vector<dtkID>();
	}

	void dtkPhysCore::LabelObjectAsDevice(dtkID objectID, dtkID deviceLabel)
	{
		mDeviceLabels[deviceLabel].push_back(objectID);
		mMassSprings[objectID]->SetUnderControl(true);
		mMassSprings[objectID]->RegisterLabel(deviceLabel);
	}

	const dtkT3<double> &dtkPhysCore::GetDeviceForceFeedback(dtkID deviceLabel)
	{
		return mDeviceForceFeedbacks[deviceLabel];
	}

	dtkPoints::Ptr dtkPhysCore::GetThreadPoints(dtkID id)
	{
		return mThreadPoints[id];
	}
};
