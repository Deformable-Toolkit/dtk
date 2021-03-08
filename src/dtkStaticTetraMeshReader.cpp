#include "dtkStaticTetraMeshReader.h"

#include <fstream>

namespace dtk
{
	dtkStaticTetraMeshReader::dtkStaticTetraMeshReader()
	{
		//nothing.
	}

	void dtkStaticTetraMeshReader::SetOutput(dtkStaticTetraMesh::Ptr mesh)
	{
		dtkAssert(mesh.get() != NULL, NULL_POINTER);
		mMesh = mesh;
	}

	void dtkStaticTetraMeshReader::SetFileName(const char* filePath)
	{
		dtkAssert(filePath != NULL, NULL_POINTER);
		mFilePath = std::string(filePath);
	}

	bool dtkStaticTetraMeshReader::Read()
	{
		bool rtn = true;

		rtn &= mMesh.get() != NULL;
		rtn &= mFilePath.size() > 0;
		if (!rtn) 
        {
            dtkAssert(rtn, ILLEGAL_STATE);
            return false;
        }

		mMesh->Clear();
		std::ifstream file(mFilePath.c_str());
		
		//Read the points.
		dtkPoints::Ptr pts = mMesh->GetPoints();
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

		//Read the tetras. (EC Table)
		size_t numOfTetras;
		file >> numOfTetras;
		for (size_t i = 0; i < numOfTetras; ++i)
		{
			dtkID4 verts;
			file >> verts.a >> verts.b >> verts.c >> verts.d;
			mMesh->InsertTetra(verts);
		}

		//Rebuild topology
		mMesh->Rebuild();

		return rtn;
	}
}

