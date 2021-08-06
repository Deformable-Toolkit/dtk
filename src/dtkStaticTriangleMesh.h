#ifndef DTK_STATICTRIANGLEMESH_H
#define DTK_STATICTRIANGLEMESH_H

#include <vector>
#include <memory>

#include <boost/utility.hpp>

#include "dtkConfig.h"
#include "dtkIDTypes.h"
#include "dtkPoints.h"
#include <cstddef>
#include <vector>

namespace dtk
{
	//This class is implemented following "Compact Array-Based Mesh Data Structures"
	/**
	* @class <dtkStaticTriangleMesh> 
	* @brief 三角网格
	* @author <>
	* @note This class is implemented following "Compact Array-Based Mesh Data Structures"
	* 
	*/
	class dtkStaticTriangleMesh: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkStaticTriangleMesh> Ptr;

		static dtkStaticTriangleMesh::Ptr New()
		{
			return dtkStaticTriangleMesh::Ptr(new dtkStaticTriangleMesh());
		}

	public:
		virtual ~dtkStaticTriangleMesh();

		void SetPoints(dtkPoints::Ptr points);
		dtkPoints::Ptr GetPoints();

		const GK::Point3& GetPoint(dtkID id) const;
        size_t GetNumberOfPoints(){ return mPts->GetNumberOfPoints(); }
		void SetPoint(dtkID id, const GK::Point3 &coord);

		void Clear();
		dtkID InsertTriangle(dtkID v0, dtkID v1, dtkID v2);
		dtkID InsertTriangle(dtkID v[3]);
		dtkID InsertTriangle(const dtkID3 &tri);

        dtkID InsertTriangleNotRepeat(dtkID v0, dtkID v1, dtkID v2);
        dtkID InsertTriangleNotRepeat(dtkID v[3]);
        dtkID InsertTriangleNotRepeat(const dtkID3 &tri);

        bool IsRepeat(dtkID v0, dtkID v1, dtkID v2);

        dtkID RemoveTriangle(dtkID v0, dtkID v1, dtkID v2);
        dtkID RemoveTriangle(dtkID v[3]);
        dtkID RemoveTriangle(const dtkID3 &tri);

        dtkID RemoveTriangleDisordered(dtkID v0, dtkID v1, dtkID v2);

        dtkID modifyElement(dtkID srcIndex, int srcOrder, dtkID dest);
        dtkID modifyElement(dtkID srcIndex, dtkID3 dest);

		void Modified();
		bool IsModified() const;
		void Rebuild();

        bool IsValidVertex(dtkID &vertexID) const
        {
            if (vertexID < 0 || vertexID > mV2E.size()) return false;
            return (mV2E[vertexID] != dtkErrorID);
        }

		const std::vector<dtkID3>& GetECTable () const	{return mEC;}
		const std::vector<dtkID >& GetV2ETable() const	{return mV2E;}
		const std::vector<dtkID3>& GetE2ETable() const	{return mE2E;}
		const std::vector<dtkID >& GetB2ETable() const	{return mB2E;}

	private:
		dtkStaticTriangleMesh();

	public:

		//Generate the HF<tri, idx>.
		static dtkID EncodeHE(dtkID tri, dtkID idx)
		{	
			return (tri << 2) | idx;
		}

		//Get the components of a HE<tri, idx>.
		//Return true if it is on the boundary
		static bool DecodeHE(dtkID he, dtkID &tri, dtkID &idx)
		{
			static const dtkWORD mask = 0xFFFFFFFC;
			tri = (he & mask) >> 2;
			idx = (he & ~mask);

			return (idx == 0);
		}

		static bool IsBoundaryHE(const dtkID& he)
		{
			static const dtkWORD mask = 0x00000003;
			return ((he & mask) == 0);
		}

		static dtkID2 GetHEVertex(const dtkID &he, const std::vector<dtkID3> &ec, const std::vector<dtkID> &b2e)
		{
			dtkID2 retVal;
			dtkID c, i;
			dtkStaticTriangleMesh::DecodeHE(he, c, i);

			if (i == 0)
			{
				dtkID twin = b2e[c];
				retVal = GetHEVertex(twin, ec, b2e);
				std::swap(retVal.a, retVal.b);
			}
			else
			{
				retVal.a = ec[c][i - 1];
				retVal.b = ec[c][i % 3];
			}

			return retVal;
		}

		static dtkID3 SortTriVertex(dtkID v0, dtkID v1, dtkID v2)
		{
			if (v2 < v1) std::swap(v1, v2);
			if (v1 < v0) std::swap(v0, v1);
			if (v2 < v1) std::swap(v1, v2);

			return dtkID3(v0, v1, v2);
		}

    private:

        static dtkDWORD Merge(const dtkID &from, const dtkID &to)
        {
            dtkDWORD key(0);

            key = ((dtkDWORD)from) << 32;
            key |= to;

            return key;
        }

		static void Split(const dtkDWORD &key, dtkID &from, dtkID &to)
		{
			static const dtkDWORD mask = 0x00000000FFFFFFFF;

			from = static_cast<dtkID>((key & ~mask) >> 32);
			to   = static_cast<dtkID>(key & mask);
		}

	private:
		bool mModified;
		dtkPoints::Ptr mPts;

		std::vector<dtkID3>		mEC;   // present a triangle
		std::vector<dtkID>		mV2E;   
		std::vector<dtkID3>		mE2E;
		std::vector<dtkID>		mB2E;
	};
}

#endif //DTK_STATICTRIANGLEMESH_H
