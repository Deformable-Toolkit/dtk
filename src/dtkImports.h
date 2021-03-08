#ifndef DTK_IMPORTS_H
#define DTK_IMPORTS_H

#include "dtkPointsImporter.h"
#include "dtkStaticTetraMeshImporter.h"
#include "dtkStaticTriangleMeshImporter.h"

#include "dtkGraphicsKernel.h"

#ifdef DTK_DEBUG
    #define DTK_IMPORTS_DEBUG
#endif

#ifdef DTK_IMPORTS_DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace dtk
{
    template <class IMPORTER_PTR>
	void Import(dtkPoints::Ptr &dst, IMPORTER_PTR &src)
	{
        dtkID maxID, id;
        size_t numOfPts;
        GK::Point3 pt;

        src->Begin(numOfPts, maxID);
        while (src->Next(id, pt))
        {
            dst->SetPoint(id, pt);
        }
        src->End();
	}

    template <class IMPORTER_PTR>
	void Import(dtkStaticTetraMesh::Ptr &dst, IMPORTER_PTR &src)
	{
#ifdef DTK_IMPORTS_DEBUG
        cout << "[Import]" << endl;
#endif
        size_t nTetras;
		dtkID4 tetra;

        size_t count = 0;
        src->Begin(nTetras);
#ifdef DTK_IMPORTS_DEBUG
        cout << "nTetras: " << nTetras << endl;
#endif
        while (src->Next(tetra))
        {
            dst->InsertTetra(tetra);
            count++;
        }
        src->End();
#ifdef DTK_IMPORTS_DEBUG
        cout << "Number of tetra inserted: " << count << endl;
        cout << "[/Import]" << endl;
        cout << endl;
#endif
	}

    template <class IMPORTER_PTR>
    void Import(dtkStaticTriangleMesh::Ptr &dst, IMPORTER_PTR &src)
    {
        size_t nTris;
        dtkID3 tri;

        src->Begin(nTris);
        while (src->Next(tri))
        {
            dst->InsertTriangle(tri);
        }
        src->End();
    }
}

#endif //DTK_IMPORTS_H
