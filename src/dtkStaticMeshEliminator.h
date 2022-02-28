#ifndef DTK_STATICMESHELIMINATOR_H
#define DTK_STATICMESHELIMINATOR_H
#include <vector>
#include "dtkIDTypes.h"
#include "dtkStaticTriangleMesh.h"
#include "dtkStaticTetraMesh.h"
#include "dtkCollisionDetectHierarchy.h"
#include "dtkPhysTetraMassSpring.h"

namespace dtk
{
	class dtkStaticMeshEliminator
	{
	public:
		typedef std::shared_ptr<dtkStaticMeshEliminator> Ptr;

		static Ptr New()
		{
			return Ptr(new dtkStaticMeshEliminator());
		}

	public:
		~dtkStaticMeshEliminator();

	public:
		typedef void (*MeshEliminatorResultsCallback)(dtkID id, size_t originalFaceNum, std::vector<dtkID3> surfaceAddFace, void *pContext);

		//typedef void ( *MeshEliminatorDeleteTetraCallback )( dtkID id, size_t eraseTetraIndex);

		//Remove part of the model by given triangular patches.
		void EliminateTriangles(dtkID id, size_t originalFaceNum, std::vector<dtkID3> &cutFace, bool singleMethod = false);

		void AddElimniateTarget(dtkID i,
								dtkStaticTriangleMesh::Ptr triangleMesh,
								dtkStaticTetraMesh::Ptr tetraMesh,
								dtkCollisionDetectHierarchy::Ptr collisionDetectHierarchy,
								dtkPhysTetraMassSpring::Ptr tetraMassSpring,
								MeshEliminatorResultsCallback meshEliminatorResultsCallback,
								void *meshEliminatorResultsCallbackContext)
		{
			SetTriangleMesh(i, triangleMesh);
			SetTetraMesh(i, tetraMesh);
			SetCollisionDetectHierarchy(i, collisionDetectHierarchy);
			SetTetraMassSpring(i, tetraMassSpring);
			SetMeshEliminatorResultsCallback(i, meshEliminatorResultsCallback);
			SetMeshEliminatorResultsCallbackContext(i, meshEliminatorResultsCallbackContext);
		}

		void RemoveElimniateTarget(dtkID i)
		{
			mTriangleMeshes.erase(i);
			mTetraMeshes.erase(i);
			mCollisionDetectHierarchies.erase(i);
			mTetraMassSprings.erase(i);
			mMeshEliminatorResultsCallback.erase(i);
			mMeshEliminatorResultsCallbackContext.erase(i);
		}

		//void SetMeshEliminatorDeleteTetraCallback( dtkID i, MeshEliminatorDeleteTetraCallback meshEliminatorDeleteTetraCallback)
		//{
		//    mMeshEliminatorDeleteTetraCallback[i] = meshEliminatorDeleteTetraCallback;
		//}

	private:
		dtkStaticMeshEliminator();

		void SetTriangleMesh(dtkID i, dtkStaticTriangleMesh::Ptr triangleMesh)
		{
			mTriangleMeshes[i] = triangleMesh;
		}

		void SetTetraMesh(dtkID i, dtkStaticTetraMesh::Ptr tetraMesh)
		{
			mTetraMeshes[i] = tetraMesh;
		}

		void SetCollisionDetectHierarchy(dtkID i, dtkCollisionDetectHierarchy::Ptr collisionDetectHierarchy)
		{
			mCollisionDetectHierarchies[i] = collisionDetectHierarchy;
		}

		void SetTetraMassSpring(dtkID i, dtkPhysTetraMassSpring::Ptr tetraMassSpring)
		{
			mTetraMassSprings[i] = tetraMassSpring;
		}

		void SetMeshEliminatorResultsCallback(dtkID i, MeshEliminatorResultsCallback meshEliminatorResultsCallback)
		{
			mMeshEliminatorResultsCallback[i] = meshEliminatorResultsCallback;
		}

		void SetMeshEliminatorResultsCallbackContext(dtkID i, void *meshEliminatorResultsCallbackContext)
		{
			mMeshEliminatorResultsCallbackContext[i] = meshEliminatorResultsCallbackContext;
		}

	private:
		//Change hierarchy.
		void ChangeHierarchy(dtkCollisionDetectHierarchy::Ptr hierarchy, dtkPoints::Ptr pts, dtkID3 collisionFace, const std::vector<dtkID3> &addFace, const std::vector<dtkID3> &deleteFace);

		//Group model.
		void GroupModel(dtkCollisionDetectHierarchy::Ptr hierarchy);

		//Triangles is identical.
		bool IsIdentical(dtkID3 x, dtkID3 y);

		//Tetrahedron contains Triangles.
		bool Contains(dtkID4 x, dtkID3 y);

		bool Intersected(dtkID3 x, dtkID3 y);

	private:
		std::map<dtkID, dtkStaticTriangleMesh::Ptr> mTriangleMeshes;

		std::map<dtkID, dtkStaticTetraMesh::Ptr> mTetraMeshes;

		std::map<dtkID, dtkCollisionDetectHierarchy::Ptr> mCollisionDetectHierarchies;

		std::map<dtkID, MeshEliminatorResultsCallback> mMeshEliminatorResultsCallback;
		std::map<dtkID, void *> mMeshEliminatorResultsCallbackContext;

		//std::map< dtkID, MeshEliminatorDeleteTetraCallback > mMeshEliminatorDeleteTetraCallback;
		std::map<dtkID, dtkPhysTetraMassSpring::Ptr> mTetraMassSprings;
	};
}

#endif //DTK_STATICMESHELIMINATOR_H