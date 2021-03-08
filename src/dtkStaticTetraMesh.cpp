#include "dtkStaticTetraMesh.h"

#include <set>

#include "dtkTx.h"
#include "dtkGraphicsTools.h"

#ifdef DTK_DEBUG
	#include <iostream>
	using namespace std;
#endif //DTK_DEBUG

namespace dtk
{
	const dtkID dtkStaticTetraMesh::EA2V[4][3] = {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}};
	const dtkID dtkStaticTetraMesh::EAdj[4][3] = {{17,13,9}, {4,12,18}, {6,16,10}, {5, 8,14}};

	dtkStaticTetraMesh::dtkStaticTetraMesh()
		:mModified(false)
	{
		//nothing.
	}
	
	void dtkStaticTetraMesh::SetPoints(const dtkPoints::Ptr &points)
	{
        	mPts = points;
        	mModified = true;
	}

	dtkPoints::Ptr dtkStaticTetraMesh::GetPoints()
	{
		return mPts;
	}

	const GK::Point3& dtkStaticTetraMesh::GetPoint(dtkID id) const
	{
		return mPts->GetPoint(id);
	}

    void dtkStaticTetraMesh::SetPoint(dtkID id, const GK::Point3 &pt)
    {
        dtkAssert(mPts->SetPoint(id, pt), OUT_OF_RANGE);
    }

	void dtkStaticTetraMesh::Clear()
	{
		mEC.clear();
		mV2F.clear();
		if (mPts.get() != NULL) mV2F.resize(mPts->GetMaxID() + 1, dtkErrorID);
		mF2F.clear();
		mB2F.clear();
	}

	dtkID dtkStaticTetraMesh::InsertTetra(dtkID v0, dtkID v1, dtkID v2, dtkID v3,bool allowDegenerate)
	{
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

		const GK::Point3& coord0 = mPts->GetPoint(v0);
        const GK::Point3& coord1 = mPts->GetPoint(v1);
        const GK::Point3& coord2 = mPts->GetPoint(v2);
        const GK::Point3& coord3 = mPts->GetPoint(v3);

		dtkSign orient = GK::Orient3D(coord0, coord1, coord2, coord3);
        if(!allowDegenerate)
        {
		    dtkAssert(orient != dtkSign::ZERO, DEGENERACY_TETRA);
		    if (orient == dtkSign::ZERO) 
                return dtkErrorID;
        }
		if (orient == dtkSign::NEGATIVE) 
            std::swap(v2, v3);

		dtkID4 entry = dtkID4(v0, v1, v2, v3);

		mEC.push_back(entry);
		mModified = true;

		return ((dtkID)mEC.size() - 1);
	}

	dtkID dtkStaticTetraMesh::InsertTetra(const dtkID4 &tetra, bool allowDegenerate)
	{
		return InsertTetra(tetra.a, tetra.b, tetra.c, tetra.d, allowDegenerate);
	}

	dtkID dtkStaticTetraMesh::InsertTetra(dtkID v[4], bool allowDegenerate)
	{
		return InsertTetra(v[0], v[1], v[2], v[3], allowDegenerate);
	}

    dtkID dtkStaticTetraMesh::RemoveTetra(dtkID v0, dtkID v1, dtkID v2, dtkID v3)
    {
        dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

        dtkID4 remove = dtkID4(v0, v1, v2, v3);
        std::vector<dtkID4>::iterator iter;  
        iter = find(mEC.begin(), mEC.end(), remove);

        mEC.erase(iter);
        mModified = true;

        return ((dtkID)mEC.size() - 1);
    }
    
    dtkID dtkStaticTetraMesh::RemoveTetra(const dtkID4 &tetra)
    {
        return RemoveTetra(tetra.a, tetra.b, tetra.c, tetra.d);
    }

    dtkID dtkStaticTetraMesh::RemoveTetra(dtkID v[4])
    {
        return RemoveTetra(v[0], v[1], v[2], v[3]);
    }

    dtkID dtkStaticTetraMesh::modifyElement(dtkID srcIndex, int srcOrder, dtkID dest)
    {
        assert(srcIndex < mEC.size());
        assert(srcOrder <= 3 && srcOrder >= 0);
        assert(dest < mPts->GetNumberOfPoints());

        mEC[srcIndex][srcOrder] = dest;

        return srcIndex;
    }

    dtkID dtkStaticTetraMesh::modifyElement(dtkID srcIndex, dtkID4 dest)
    {
        dtkAssert(srcIndex < mEC.size());
        dtkAssert(dest.a < mPts->GetNumberOfPoints() && dest.b < mPts->GetNumberOfPoints() &&
            dest.c < mPts->GetNumberOfPoints() && dest.d < mPts->GetNumberOfPoints());

        mEC[srcIndex] = dest;

        return srcIndex;
    }

	void dtkStaticTetraMesh::Modified()
	{
		mModified = true;
	}

	bool dtkStaticTetraMesh::IsModified()
	{
		return mModified;
	}

	void dtkStaticTetraMesh::Rebuild()
	{
		if (!mModified) return;

		//Cleanup the EC table.
		std::vector<dtkID4>(mEC).swap(mEC);

		//Clear all the other tables.
		mV2F.clear();
		mV2F.resize(mPts->GetMaxID() + 1, dtkErrorID);
		mF2F.clear();
		mF2F.resize(mEC.size(), dtkID4(dtkErrorID, dtkErrorID, dtkErrorID, dtkErrorID));
		mB2F.clear();

		//Count the AHF. Test whether it is on border.
		std::map<dtkID3, dtkID> hfs;
		dtkID cellNum = (dtkID)mEC.size();
		for (dtkID cell = 0; cell < cellNum; ++cell)
		{
			dtkID4 &verts = mEC[cell];

			dtkID3 tris[5] = {dtkID3(dtkErrorID, dtkErrorID, dtkErrorID),
							  dtkID3(verts.a, verts.c, verts.b),
							  dtkID3(verts.a, verts.b, verts.d),
							  dtkID3(verts.b, verts.c, verts.d),
							  dtkID3(verts.c, verts.a, verts.d)};

			std::map<dtkID3, dtkID>::iterator iter;
			for (dtkID idx = 1; idx < 5; ++idx)
			{
				dtkID3 &t = tris[idx];
				dtkID3 sorted = SortTriVertex(t.a, t.b, t.c);

				iter = hfs.find(sorted);
				if (iter != hfs.end())	//A pair AHF founded.
				{
					dtkID ahf1, ahf2;
					
					dtkID hf2 = iter->second;
					dtkID cell2, idx2, anchor2;
					DecodeAHF(hf2, cell2, idx2, anchor2);
					
					GetTwinAHFs(ahf1, ahf2, cell, idx, cell2, idx2, mEC);

					//Update V2F
					mV2F[mEC[cell][EA2V[idx - 1][0]]] = EncodeAHF(cell, idx, 0);
					mV2F[mEC[cell][EA2V[idx - 1][1]]] = EncodeAHF(cell, idx, 1);
					mV2F[mEC[cell][EA2V[idx - 1][2]]] = EncodeAHF(cell, idx, 2);

					//Update F2F
					dtkAssert(mF2F[cell ][idx  - 1] == dtkErrorID, MUST_BE_MANIFOLD_MESH);
					dtkAssert(mF2F[cell2][idx2 - 1] == dtkErrorID, MUST_BE_MANIFOLD_MESH);
					mF2F[cell ][idx  - 1] = ahf2;
					mF2F[cell2][idx2 - 1] = ahf1;

					hfs.erase(iter);
				}
				else
				{
					dtkID hf = EncodeAHF(cell, idx, 0);
					hfs.insert(std::make_pair(sorted, hf));
				}
			}
		}

		//Deal with Border AHFs
		dtkID borderID = 0;
		std::map<dtkID3, dtkID>::iterator iter;
		for (iter = hfs.begin(); iter != hfs.end(); ++iter)
		{
			dtkID ahf1, ahf2;
			ahf1 = iter->second; //<c, i, 0>
			ahf2 = EncodeAHF(borderID, 0, 0);	//<b, 0, 0>

			mB2F.push_back(ahf1);
			
			dtkID c, i, j;
			DecodeAHF(ahf1, c, i, j);
			dtkAssert(mF2F[c][i - 1] == dtkErrorID, MUST_BE_MANIFOLD_MESH);
			mF2F[c][i - 1] = ahf2;
		
			mV2F[mEC[c][EA2V[i - 1][0]]] = EncodeAHF(borderID, 0, 0);
			mV2F[mEC[c][EA2V[i - 1][1]]] = EncodeAHF(borderID, 0, 2);
			mV2F[mEC[c][EA2V[i - 1][2]]] = EncodeAHF(borderID, 0, 1);

			++borderID;
		}

		mModified = false;
	}

    bool
    dtkStaticTetraMesh::IsValidTetra(dtkID4 verts, dtkPoints::Ptr pts)
    {
        const GK::Point3 &coord0 = pts->GetPoint(verts.a);
        const GK::Point3 &coord1 = pts->GetPoint(verts.b);
        const GK::Point3 &coord2 = pts->GetPoint(verts.c);
        const GK::Point3 &coord3 = pts->GetPoint(verts.d);

        dtkSign orient = GK::Orient3D(coord0, coord1, coord2, coord3);
        return (orient == dtkSign::POSITIVE);
    }

    dtkID4 dtkStaticTetraMesh::GetTetraById(int id)
    {
        if(id >= ((int)mEC.size()) || id < 0)
            return dtkID4(dtkErrorID,0,0,0);
        return mEC[id];
    }

    bool dtkStaticTetraMesh::GetSurface(dtkStaticTriangleMesh::Ptr& triMesh)
    {
//        dtkStaticTriangleMesh::Ptr triMesh = dtkStaticTriangleMesh::New();
        this->Rebuild();
        triMesh->SetPoints( mPts );
        std::set<dtkID3> tempFaceSet;
        std::set<dtkID3>::iterator setIt;
        std::vector<dtkID>::iterator vecIt;
        
        for( dtkID i = 0; i < mEC.size(); i++ )
        {
            dtkID4 tetra = mEC[i];
            dtkID3 tempSortedFace;
            dtkID3 tempOriFace;
            for( dtkID j = 0; j < 4; j++ )
            {
                tempOriFace = dtkID3(tetra[this->EA2V[j][0]],tetra[this->EA2V[j][1]],tetra[this->EA2V[j][2]]);
                tempSortedFace = this->SortTriVertex(tempOriFace[0],tempOriFace[1],tempOriFace[2]);
                setIt = tempFaceSet.find(tempSortedFace);
                bool findHf = false;
                for( vecIt = mB2F.begin(); vecIt != mB2F.end(); vecIt++)
                {
                    if( *vecIt == this->EncodeAHF(i, j + 1, 0) )
                    {
                        findHf = true;
                        break;
                    }
                }
                if( setIt == tempFaceSet.end() && findHf )
                {
                    tempFaceSet.insert(tempSortedFace);
                    triMesh->InsertTriangle( tempOriFace );
                }
            }
        }
		return true;
    }
}
