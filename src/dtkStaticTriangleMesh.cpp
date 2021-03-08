#include "dtkStaticTriangleMesh.h"

#include <set>
#include <map>
#include <queue>

//Debug
#include <iostream>
using namespace std;

namespace dtk
{
	dtkStaticTriangleMesh::dtkStaticTriangleMesh()
		:mModified(false)
	{
		//nothing.
	}

    dtkStaticTriangleMesh::~dtkStaticTriangleMesh()
	{
		//nothing.
	}

	void dtkStaticTriangleMesh::SetPoints(dtkPoints::Ptr points)
	{
		dtkAssert(points.get() != NULL, NULL_POINTER);
		mPts = points;
	}

	dtkPoints::Ptr dtkStaticTriangleMesh::GetPoints()
	{
		return mPts;
	}

	const GK::Point3& dtkStaticTriangleMesh::GetPoint(dtkID id) const
	{
		dtkAssert(mPts.get() != NULL, NULL_POINTER);
		return mPts->GetPoint(id);
	}

	void dtkStaticTriangleMesh::SetPoint(dtkID id, const GK::Point3 &coord)
	{
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);
		dtkAssert(mPts->SetPoint(id, coord), OUT_OF_RANGE);
	}

	void dtkStaticTriangleMesh::Clear()
	{
		mEC.clear();
		mV2E.clear();
		if (mPts.get() != NULL) mV2E.resize(mPts->GetMaxID() + 1, dtkErrorID);
		mE2E.clear();
		mB2E.clear();
	}

	dtkID dtkStaticTriangleMesh::InsertTriangle(dtkID v0, dtkID v1, dtkID v2)
	{
		//You must call SetPoints before excuting this function.
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

		const GK::Point3& coord0 = mPts->GetPoint(v0);
		const GK::Point3& coord1 = mPts->GetPoint(v1);
		const GK::Point3& coord2 = mPts->GetPoint(v2);

        dtkSign orient = GK::Orient3D(coord0, coord1, coord2);
        //dtkAssert(orient != dtkSign::ZERO, DEGENERACY_TRIANGLE);
        if (orient == dtkSign::ZERO) return dtkErrorID;
		//Disable this line to ensure clockwise, but harm other functionality
        //if (orient == dtkSign::NEGATIVE) std::swap(v1, v2);

		//Insert into EC Table.
		dtkID3 entry = dtkID3(v0, v1, v2);
		mEC.push_back(entry);

		mModified = true;

		return (dtkID)(mEC.size() - 1);
	}

	dtkID dtkStaticTriangleMesh::InsertTriangle(dtkID v[3])
	{
		return InsertTriangle(v[0], v[1], v[2]);
	}

	dtkID dtkStaticTriangleMesh::InsertTriangle(const dtkID3 &tri)
	{
		return InsertTriangle(tri.a, tri.b, tri.c);
	}

    dtkID dtkStaticTriangleMesh::InsertTriangleNotRepeat(dtkID v0, dtkID v1, dtkID v2)
    {
        //You must call SetPoints before excuting this function.
        dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

        if (IsRepeat(v0, v1, v2) == true)
        {
            return (dtkID)(mEC.size() - 1);
        }

        const GK::Point3& coord0 = mPts->GetPoint(v0);
        const GK::Point3& coord1 = mPts->GetPoint(v1);
        const GK::Point3& coord2 = mPts->GetPoint(v2);

        dtkSign orient = GK::Orient3D(coord0, coord1, coord2);
        //dtkAssert(orient != dtkSign::ZERO, DEGENERACY_TRIANGLE);
        if (orient == dtkSign::ZERO) return dtkErrorID;
        //Disable this line to ensure clockwise, but harm other functionality
        if (orient == dtkSign::NEGATIVE) std::swap(v1, v2);

        //Insert into EC Table.
        dtkID3 entry = dtkID3(v0, v1, v2);
        mEC.push_back(entry);

        mModified = true;

        return (dtkID)(mEC.size() - 1);
    }

    dtkID dtkStaticTriangleMesh::InsertTriangleNotRepeat(dtkID v[3])
    {
        return InsertTriangleNotRepeat(v[0], v[1], v[2]);
    }

    dtkID dtkStaticTriangleMesh::InsertTriangleNotRepeat(const dtkID3 &tri)
    {
        return InsertTriangleNotRepeat(tri.a, tri.b, tri.c);
    }

    bool dtkStaticTriangleMesh::IsRepeat(dtkID v0, dtkID v1, dtkID v2)
    {
        for (unsigned int i = 0; i < mEC.size(); i++)
        {
            if ((mEC[i].a == v0 && mEC[i].b == v1 && mEC[i].c == v2) ||
                (mEC[i].a == v0 && mEC[i].b == v2 && mEC[i].c == v1) ||
                (mEC[i].a == v1 && mEC[i].b == v0 && mEC[i].c == v2) ||
                (mEC[i].a == v1 && mEC[i].b == v2 && mEC[i].c == v0) ||
                (mEC[i].a == v2 && mEC[i].b == v0 && mEC[i].c == v1) ||
                (mEC[i].a == v2 && mEC[i].b == v1 && mEC[i].c == v0))
            {
                return true;
            }
        }
        return false;
    }

    dtkID dtkStaticTriangleMesh::RemoveTriangle(dtkID v0, dtkID v1, dtkID v2)
    {
        //You must call SetPoints before excuting this function.
        dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

        dtkID3 remove = dtkID3(v0, v1, v2);
        std::vector<dtkID3>::iterator iter;  
        iter = find(mEC.begin(), mEC.end(), remove);

        mEC.erase(iter);
        mModified = true;

        return ((dtkID)mEC.size() - 1);
    }

    dtkID dtkStaticTriangleMesh::RemoveTriangle(dtkID v[3])
    {
        return RemoveTriangle(v[0], v[1], v[2]);
    }

    dtkID dtkStaticTriangleMesh::RemoveTriangle(const dtkID3 &tri)
    {
        return RemoveTriangle(tri.a, tri.b, tri.c);
    }

    dtkID dtkStaticTriangleMesh::RemoveTriangleDisordered(dtkID v0, dtkID v1, dtkID v2)
    {
        for (unsigned int i = 0; i < mEC.size(); i++)
        {
            if (mEC[i].a == v0 && mEC[i].b == v1 && mEC[i].c == v2){ return RemoveTriangle(v0, v1, v2); }
            else if (mEC[i].a == v0 && mEC[i].b == v2 && mEC[i].c == v1){ return RemoveTriangle(v0, v2, v1); }
            else if (mEC[i].a == v1 && mEC[i].b == v0 && mEC[i].c == v2){ return RemoveTriangle(v1, v0, v2); }
            else if (mEC[i].a == v1 && mEC[i].b == v2 && mEC[i].c == v0){ return RemoveTriangle(v1, v2, v0); }
            else if (mEC[i].a == v2 && mEC[i].b == v0 && mEC[i].c == v1){ return RemoveTriangle(v2, v0, v1); }
            else if (mEC[i].a == v2 && mEC[i].b == v1 && mEC[i].c == v0){ return RemoveTriangle(v2, v1, v0); }
        }
        return 0;
    }

    dtkID dtkStaticTriangleMesh::modifyElement(dtkID srcIndex, int srcOrder, dtkID dest)
    {
        assert(srcIndex < mEC.size());
        assert(srcOrder <= 2 && srcOrder >= 0);
        assert(dest < mPts->GetNumberOfPoints());

        mEC[srcIndex][srcOrder] = dest;

        return srcIndex;
    }

    dtkID dtkStaticTriangleMesh::modifyElement(dtkID srcIndex, dtkID3 dest)
    {
        dtkAssert(srcIndex < mEC.size());
        dtkAssert(dest.a < mPts->GetNumberOfPoints() && dest.b < mPts->GetNumberOfPoints() && dest.c < mPts->GetNumberOfPoints());

        mEC[srcIndex] = dest;

        return srcIndex;
    }

	void dtkStaticTriangleMesh::Modified()
	{
		mModified = true;
	}

	bool dtkStaticTriangleMesh::IsModified() const
	{
		return mModified;
	}

	void dtkStaticTriangleMesh::Rebuild()
	{
		const static dtkID2 edges[3] = {dtkID2(1, 2), dtkID2(2, 3), dtkID2(3, 1)};
		const static dtkFloat3 upz = dtkFloat3(0.0f, 0.0f, 1.0f);

		if (!mModified) return;
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);

		std::vector<dtkID3>(mEC).swap(mEC);

		// clear mV2E, mE2E, mB2E, rebulid mV2E, mE2E, mB2E according the new mEC.
		dtkID maxID = mPts->GetMaxID();
		mV2E.clear();
		mV2E.resize(maxID + 1, dtkErrorID);
		mE2E.clear();
		mE2E.resize(mEC.size(), dtkID3(dtkErrorID, dtkErrorID, dtkErrorID));
		mB2E.clear();

		//Rebuild. dtkDWORD is 64-bits data type, hes represent half-edges.
		std::map<dtkDWORD, dtkID> hes;
		// once a half-edge is finished, it will add into finishedEdge.
		std::set<dtkDWORD> finishedEdge;
		
		//Find the adjacent triangle of each triangle.
		//And find the top-most vertex and its incident triangle.
		dtkID firstTri;
        GK::Float maxZ = -dtkFloatMax;
		for (size_t i = 0; i < mEC.size(); ++i)
		{
			const dtkID3 &tri = mEC[i];

			for (int j = 0; j < 3; ++j)
			{
				// get three half-edges from a triangle with three vertexs. 
				dtkID2 edge = sort(mEC[i][edges[j].a - 1], mEC[i][edges[j].b - 1]); 
				// combine the from point and to point into key.
				dtkDWORD key = Merge(edge.a, edge.b);
				
				//One edge must incident to no more than 2 triangles.
				dtkAssert(finishedEdge.find(key) == finishedEdge.end(), MUST_BE_MANIFOLD_MESH);

				std::map<dtkDWORD, dtkID>::iterator pos;
				pos = hes.find(key);
				if (pos != hes.end())	//a pair edge found, because edge with a, b are sorted.
				{
					// return the mapped value.
					dtkID he = pos->second;
					dtkID d, s;
					// decode the code of half-edge
					// d present triangle index, s present half-edge index in local triangle.
					dtkStaticTriangleMesh::DecodeHE(he, d, s);

 					dtkAssert(mE2E[d][s - 1] == dtkErrorID, MUST_BE_MANIFOLD_MESH);
 					dtkAssert(mE2E[i][j    ] == dtkErrorID, MUST_BE_MANIFOLD_MESH);
					
					//-------------------------------------------------------------------------
					//there should be: 
					// dthID he_ij = dtkStaticTriangleMesh::EncodeHE((dtkID)i, (dtkID)(j + 1));
					// mE2E[d][s-1] = he_ij;
					// mE2E[i][j]   = he;
					//-------------------------------------------------------------------------
					mE2E[d][s - 1] = (dtkID)i;
					mE2E[i][j]     = d;
					
					hes.erase(pos);
					finishedEdge.insert(key);
				}
				else
				{
					// add the new half-edge into hes.
					dtkID he = dtkStaticTriangleMesh::EncodeHE((dtkID)i, (dtkID)(j + 1));
					hes.insert(std::make_pair(key, he));
				}

				//------------------------------------------------
				// for what ???
				const GK::Point3& coord = mPts->GetPoint(tri[j]);
                if (coord.z() > maxZ)
				{
					maxZ = coord.z();
					firstTri = (dtkID) i;
				}
				//-------------------------------------------------
			}
		}

		// Set the border HEs
		// the way to justify board edge: no pair half-edge, which stay in hes.
		for (std::map<dtkDWORD, dtkID>::iterator iter = hes.begin(); iter != hes.end(); ++iter)
		{
			dtkDWORD e = iter->first;
			dtkID he = iter->second;

			dtkID c, i;
			dtkStaticTriangleMesh::DecodeHE(he, c, i);

			mB2E.push_back(he);
			mE2E[c][i - 1] = dtkStaticTriangleMesh::EncodeHE((dtkID)mB2E.size() - 1, 0);
			//i % 3 ???
			mV2E[mEC[c][i % 3]] = he;
		}

		
		std::vector<bool> setted;
		std::queue<dtkID> detected;
		setted.resize(mEC.size(), false);
	
		const GK::Point3 &coord0 = mPts->GetPoint(mEC[firstTri][0]);
		const GK::Point3 &coord1 = mPts->GetPoint(mEC[firstTri][1]);
		const GK::Point3 &coord2 = mPts->GetPoint(mEC[firstTri][2]);

		// if the triangle's direction is negetive, modify the mE2E and mEC.but why not modify the mB2E?
        if (GK::Orient3D(coord0, coord1, coord2) == dtkSign::NEGATIVE)
        {
			std::swap(mEC[firstTri][1], mEC[firstTri][2]);
			std::swap(mE2E[firstTri][0], mE2E[firstTri][2]);
		}
		setted[firstTri] = true;

		detected.push(firstTri);
		while (!detected.empty())
		{
			dtkID curTri = detected.front();
			detected.pop();

			for (int i = 0; i < 3; ++i)
			{
				dtkID neighbour = mE2E[curTri][i];
				dtkID s, t, u;
				
				s = mEC[curTri][(i + 0) % 3];
				t = mEC[curTri][(i + 1) % 3];
				u = mEC[curTri][(i + 2) % 3];
				dtkDWORD e = Merge(s, t);

				dtkID2 sorted = sort(s, t);

				if (hes.find(Merge(sorted.a, sorted.b)) != hes.end()) continue;	//Is boundary edge, has been dealt before.

				if (!setted[neighbour])
				{
					dtkDWORD e0, e1, e2;
					e0 = Merge(mEC[neighbour][1], mEC[neighbour][0]);
					e1 = Merge(mEC[neighbour][2], mEC[neighbour][1]);
					e2 = Merge(mEC[neighbour][0], mEC[neighbour][2]);

					//Need to change the orientation.
					if (e != e0 && e != e1 && e != e2)
					{
						std::swap(mEC[neighbour][1], mEC[neighbour][2]);
						std::swap(mE2E[neighbour][0], mE2E[neighbour][2]);
					}
					setted[neighbour] = true;

					detected.push(neighbour);
				}

				dtkID idx;
				if (t == mEC[neighbour][0]) idx = 1;
				if (t == mEC[neighbour][1]) idx = 2;
				if (t == mEC[neighbour][2]) idx = 3;

				mE2E[curTri][i] = dtkStaticTriangleMesh::EncodeHE(neighbour, idx);
			}

			for (int i = 0; i < 3; ++i)
			{
				dtkID he = mV2E[mEC[curTri][i]];

				if (!IsBoundaryHE(he)) 
					mV2E[mEC[curTri][i]] = dtkStaticTriangleMesh::EncodeHE(curTri, i + 1);
			}
		}

		mModified = false;
	}
}
