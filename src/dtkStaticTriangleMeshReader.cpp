#include "dtkStaticTriangleMeshReader.h"

#include <fstream>

namespace dtk
{
	dtkStaticTriangleMeshReader::dtkStaticTriangleMeshReader()
	{
		//nothing.
	}

	void 
    dtkStaticTriangleMeshReader::SetOutput(
            dtkStaticTriangleMesh::Ptr mesh)
	{
		dtkAssert(mesh.get() != NULL, NULL_POINTER);
		mMesh = mesh;
	}

	void 
    dtkStaticTriangleMeshReader::SetFileName(
            const char* filePath)
	{
		dtkAssert(filePath != NULL, NULL_POINTER);
		mFilePath = std::string(filePath);
	}

	bool 
    dtkStaticTriangleMeshReader::Read()
	{		
		bool rtn = true;

		rtn &= mMesh.get() != NULL;
		rtn &= mFilePath.size() > 0;

        dtkPoints::Ptr pts = mMesh->GetPoints();
        rtn &= pts.get() != NULL;
		if (!rtn) 
        {
            dtkAssert(rtn, ILLEGAL_STATE);
            return false;
        }

		mMesh->Clear();
		std::ifstream file(mFilePath.c_str());

		//Read the points.
		size_t numOfPts;
		dtkID maxID;
		file >> numOfPts >> maxID;

		for (size_t i = 0; i < numOfPts; ++i)
		{
			dtkID id;
			dtkFloat3 coord;
			file >> id >> coord.x >> coord.y >> coord.z;
			pts->SetPoint(id, GK::Point3(coord.x, coord.y, coord.z));
		}

		//Read the triangle (EC Table)
		size_t numOfTris;
		file >> numOfTris;
		for (size_t i = 0; i < numOfTris; ++i)
		{
			dtkID3 verts;
			file >> verts.a >> verts.b >> verts.c;
			mMesh->InsertTriangle(verts);
		}

		//Rebuild topology
		//mMesh->Rebuild();

		return rtn;
	}
}

