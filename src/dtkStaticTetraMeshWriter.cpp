#include "dtkStaticTetraMeshWriter.h"

#include <fstream>

namespace dtk
{
	dtkStaticTetraMeshWriter::dtkStaticTetraMeshWriter()
	{
		//nothing.
	}

	void dtkStaticTetraMeshWriter::SetInput(dtkStaticTetraMesh::Ptr mesh)
	{
		dtkAssert(mesh.get() != NULL, NULL_POINTER);
		mMesh = mesh;
	}

	void dtkStaticTetraMeshWriter::SetFileName(const char* filePath)
	{
		dtkAssert(filePath != NULL, NULL_POINTER);
		mFilePath = std::string(filePath);
	}

	bool dtkStaticTetraMeshWriter::Write()
	{
		bool rtn = true;

		rtn &= mMesh.get() != NULL;
		rtn &= mFilePath.size() > 0;
		if (!rtn) 
        {
            dtkAssert(rtn, ILLEGAL_STATE);
            return false;
        }

		std::ofstream file(mFilePath.c_str());

		//Write the points
		dtkPoints::Ptr pts = mMesh->GetPoints();
		file<<pts->GetNumberOfPoints()<<std::endl;
		file<<pts->GetMaxID()<<std::endl;

		dtkID id;
        GK::Point3 coord;
		pts->Begin();
		while (pts->Next(id, coord))
		{
			file.precision(16);
			file<<id<<" "<<coord.x()<<" "<<coord.y()<<" "<<coord.z()<<std::endl;
		}

		//Write the tetras. (EC Table)
		const std::vector<dtkID4> &ec = mMesh->GetECTable();
		file<<ec.size()<<std::endl;
		for (size_t i = 0; i < ec.size(); ++i)
		{
			dtkID4 verts = ec[i];
			file<<verts.a<<" "<<verts.b<<" "<<verts.c<<" "<<verts.d<<std::endl;
		}

        return rtn;
	}
}

