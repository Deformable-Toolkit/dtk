#ifndef DTK_STATICTETRAMESH_H
#define DTK_STATICTETRAMESH_H

#include <set>
#include <memory>
#include <boost/utility.hpp>

#include "dtkConfig.h"
#include "dtkIDTypes.h"
#include "dtkPoints.h"
#include "dtkGraphicsKernel.h"

#include "dtkStaticTriangleMesh.h"

namespace dtk
{
	//This class is implemented following "Compact Array-Based Mesh Data Structures"
	/**
	* @class <dtkStaticTetraMesh> 
	* @brief 四面体网格
	* @author <>
	* @note This class is implemented following "Compact Array-Based Mesh Data Structures"
	*/
	class dtkStaticTetraMesh: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkStaticTetraMesh> Ptr;

		static dtkStaticTetraMesh::Ptr New()
		{
			return Ptr(new dtkStaticTetraMesh());
		};

	public:
        
        void SetPoints(const dtkPoints::Ptr &points);
		dtkPoints::Ptr GetPoints();

		const GK::Point3& GetPoint(dtkID id) const;
		void SetPoint(dtkID id, const GK::Point3 &coord);
        size_t GetNumberOfPoints() const { 
            if(mPts)
                return mPts->GetNumberOfPoints();
            else
                return 0;
        }


		void Clear();

		dtkID InsertTetra(dtkID v0, dtkID v1, dtkID v2, dtkID v3,bool allowDegenated = false);
		dtkID InsertTetra(dtkID v[4],bool allowDegenerate = false);
		dtkID InsertTetra(const dtkID4 &tetra, bool allowDegenerate = false);

        dtkID RemoveTetra(dtkID v0, dtkID v1, dtkID v2, dtkID v3);
        dtkID RemoveTetra(dtkID v[4]);
        dtkID RemoveTetra(const dtkID4 &tetra);

        dtkID modifyElement(dtkID srcIndex, int srcOrder, dtkID dest);
        dtkID modifyElement(dtkID srcIndex, dtkID4 dest);

		void Modified();
		bool IsModified();
		void Rebuild();

		const std::vector<dtkID4>& GetECTable () const	{return mEC;}
		const std::vector<dtkID >& GetV2FTable() const	{return mV2F;}
		const std::vector<dtkID4>& GetF2FTable() const	{return mF2F;}
		const std::vector<dtkID >& GetB2FTable() const	{return mB2F;}

        static bool IsValidTetra(dtkID4 verts, dtkPoints::Ptr pts);
        dtkID4 GetTetraById(int id);
        int GetTetraNum() { return (int)mEC.size(); }

        bool GetSurface(dtkStaticTriangleMesh::Ptr&);

	public:
		//AHF: <cell, index, anchor> 
		//Help functions to comprehensive the mesh data-structure.

		//Generate the AHF <c, i, j>
		static dtkID EncodeAHF(dtkID c, dtkID i, dtkID j)
		{
			return ((c << 5) | (i << 2) | j);
		}

		//Generate the AHF <cell, localIdx>, and localIdx = <i, j>
		static dtkID EncodeAHF(dtkID cell, dtkID localIdx)
		{
			return (cell << 5) | localIdx;
		}

        //Get the components of a AHF<c, localIdx>
        static void DecodeAHF(dtkID hf, dtkID &c, dtkID &localIdx)
        {
            static const dtkID cellMask     = 0xFFFFFFE0; 
			static const dtkID localIdxMask = 0x0000001F;
			
            c = (hf & cellMask) >> 5;
            localIdx = (hf & localIdxMask);
        }

		//Get the components of a AHF <c, i, j>
		//Return true if it is on boundary.
		static bool DecodeAHF(dtkID hf, dtkID &c, dtkID &i, dtkID &j)
		{
			static const dtkID cellMask = 0xFFFFFFE0;
			static const dtkID idxMask = 0x0000001C;
			static const dtkID anchorMask = 0x00000003;

			c = (hf & cellMask) >> 5;
			i = (hf & idxMask) >> 2;
			j = (hf & anchorMask);

			return (i == 0);
		}

		//Get the component of a LocalIdx <i, j>
		static void DecodeLocalIdx(dtkID localIdx, dtkID &i, dtkID &j)
		{
			static const dtkID idxMask = 0x0000001C;
			static const dtkID anchorMask = 0x00000003;

			i = (localIdx & idxMask) >> 2;
			j = (localIdx & anchorMask);
		}

		//Test whether the AHF is a boundary AHF.
		//Return true if it is.
		static bool IsBoundaryAHF(const dtkID &ahf)
		{
			static const dtkID mask = 0x0000001C;
			return ((ahf & mask) == 0);
		}

		//Return the vertices of an AHF.
		//The first element is the anchor vertex.
		static dtkID3 GetAHFVertex(const dtkID &ahf,
			const std::vector<dtkID4> &ec, const std::vector<dtkID> &b2f)
		{
			dtkID3 retVal;

			dtkID c, i, j;
			DecodeAHF(ahf, c, i, j);

			if (i != 0)
			{
				retVal.a = ec[c][EA2V[i - 1][(j + 0) % 3]];
				retVal.b = ec[c][EA2V[i - 1][(j + 1) % 3]];
				retVal.c = ec[c][EA2V[i - 1][(j + 2) % 3]];
			}
			else
			{
				dtkID d, s, t;
				DecodeAHF(b2f[c], d, s, t);

				j = (3 - j + t) % 3;
				retVal.a = ec[d][EA2V[s - 1][(j + 0) % 3]];
				retVal.b = ec[d][EA2V[s - 1][(j + 2) % 3]];
				retVal.c = ec[d][EA2V[s - 1][(j + 1) % 3]];
			}

			return retVal;
		}

		dtkID3 GetAHFVertex(const dtkID &ahf)
		{
			return GetAHFVertex(ahf, mEC, mB2F);
		}

        dtkID2 GetEdgeVertex(dtkID ahf, dtkStaticTetraMesh::Ptr mesh)
        {
            return GetEdgeVertex(ahf, mEC, mF2F, mB2F);
        }

        static dtkID2 GetEdgeVertex(dtkID ahf, 
                                const std::vector<dtkID4> &ec, 
                                const std::vector<dtkID4> &f2f, 
                                const std::vector<dtkID > &b2f)
        {
            if (dtkStaticTetraMesh::IsBoundaryAHF(ahf))
            {
                dtkID twin = GetTwinAHF(ahf, f2f, b2f);
                ahf = GetPrevAHF(twin);
            }
    
            dtkID c, i, j;
            dtkStaticTetraMesh::DecodeAHF(ahf, c, i, j);
            
            return dtkID2(ec[c][EA2V[i - 1][j]], 
                          ec[c][EA2V[i - 1][(j + 1)%3]]);
        }
                

		//Get the twin of selected AHF.
		//The twined AHF will anchored at the same vertex.
		static dtkID GetTwinAHF(const dtkID &ahf, const std::vector<dtkID4> &f2f, const std::vector<dtkID> &b2f)
		{
			dtkID c, i, j;
			DecodeAHF(ahf, c, i, j);

			dtkID d, s, t, tmp;
			tmp = (i == 0? b2f[c]: f2f[c][i - 1]);
			DecodeAHF(tmp, d, s, t);
			
			return EncodeAHF(d, s, (3 - j + t) % 3);
		}

		dtkID GetTwinAHF(const dtkID &ahf)
		{
			return GetTwinAHF(ahf, mF2F, mB2F);
		}

		//Get the HF <c, i> of ahf = AHF<c, i, j>. 
		//The anchor information will lost.
		static dtkID GetHF(const dtkID &ahf)
		{
			return (ahf & 0xFFFFFFFC);
		}

        //Get the other AHF which share an edge with inputted one, 
        //note that the shared edge starts from the anchor, and 
        //follow the direction defined by the inputted AHF.
        static dtkID GetAdjacentAHF(dtkID c, dtkID i, dtkID j)
        {
            dtkAssert(0 != i, UNKNOW_ERROR);
            return EncodeAHF(c, EAdj[i - 1][j]);
        }

       static dtkID GetAdjacentAHF(dtkID ahf)
        {
            dtkID d, s, t;
            DecodeAHF(ahf, d, s, t);

            return GetAdjacentAHF(d, s, t);
        }

        //Get the previous AHF which in the same facet.
        //For example, input a AHF <c, i, j> which represent a triangle 
        //anchored at p0, t(p0, p1, p2), the returned AHF <c, i, (j+2)%3>
        //represents the triangle t'(p2, p0, p1) which anchored at p2.
        static dtkID GetPrevAHF(dtkID c, dtkID i, dtkID j)
        {
            return EncodeAHF(c, i, (j+2)%3);
        }

        static dtkID GetPrevAHF(dtkID ahf)
        {
            dtkID d, s, t;
            DecodeAHF(ahf, d, s, t);
            return GetPrevAHF(d, s, t);
        }

        //Get the next AHF which in the same facet.
        //For example, input a AHF <c, i, j> which represent a triangle 
        //anchored at p0, t(p0, p1, p2), the returned AHF <c, i, (j+1)%3>
        //represents the triangle t'(p1, p2, p0) which anchored at p1.
        static dtkID GetNextAHF(dtkID c, dtkID i, dtkID j)
        {
            return EncodeAHF(c, i, (j+1)%3);
        }

        static dtkID GetNextAHF(dtkID ahf)
        {
            dtkID d, s, t;
            DecodeAHF(ahf, d, s, t);
            return GetNextAHF(d, s, t);
        }

		//Sort the three vertices' indexes in ascending order.
		static dtkID3 SortTriVertex(dtkID v0, dtkID v1, dtkID v2)
		{
			if (v2 < v1) std::swap(v1, v2);
			if (v1 < v0) std::swap(v0, v1);
			if (v2 < v1) std::swap(v1, v2);

			return dtkID3(v0, v1, v2);
		}

		//Get Twin AHFs of two cells.
		static void GetTwinAHFs(dtkID &ahf1, dtkID &ahf2, 
								dtkID cell1, dtkID idx1, 
								dtkID cell2, dtkID idx2, 
								const std::vector<dtkID4> &ec)
		{
			dtkID anchor1, anchor2;

			dtkID pt1 = ec[cell1][EA2V[idx1 - 1][0]];
			dtkID pt2 = ec[cell2][EA2V[idx2 - 1][0]];
			for (int i = 0; i < 3; ++i)
			{
				if (ec[cell1][EA2V[idx1 - 1][i]] == pt2) anchor2 = i;
				if (ec[cell2][EA2V[idx2 - 1][i]] == pt1) anchor1 = i;
			}

			ahf1 = EncodeAHF(cell1, idx1, anchor1);
			ahf2 = EncodeAHF(cell2, idx2, anchor2);
		}

		const static dtkID	EA2V[4][3];  /**<< 四面体里的每个三角形 */
		const static dtkID	EAdj[4][3];

	private:
		dtkStaticTetraMesh();

	private:
		bool mModified;
		dtkPoints::Ptr mPts;

		std::vector<dtkID4>	mEC;
		std::vector<dtkID>	mV2F;
		std::vector<dtkID4> mF2F;
		std::vector<dtkID>	mB2F;
	};
}

#endif //DTK_STATICTETRAMESH_H

