#ifndef DTK_POINTSVECTOR_H
#define DTK_POINTSVECTOR_H

#include "dtkConfig.h"

#include <memory>
#include "dtkPoints.h"

namespace dtk
{
	class dtkPointsVector: public dtkPoints
	{
	public:
		typedef std::shared_ptr<dtkPointsVector> Ptr;

		static dtkPointsVector::Ptr New()
		{
			return dtkPointsVector::Ptr(new dtkPointsVector());
		}

		static dtkPointsVector::Ptr New(size_t size)
		{
			return dtkPointsVector::Ptr(new dtkPointsVector(size));
		}

		static dtkPointsVector::Ptr New(const std::vector<GK::Point3> &coords)
		{
			return dtkPointsVector::Ptr(new dtkPointsVector(coords));
		}

	public:
		inline const GK::Point3& GetPoint(dtkID id) const
		{
            //dtkAssert(id < mCoords.size(), OUT_OF_RANGE);
            //assert( id < mCoords.size() );
			return mCoords[id];
		}

		inline bool SetPoint(dtkID id, const GK::Point3 &coord)
		{
			if (id < mCoords.size())
			{
				mCoords[id] = coord;
				return true;
			}
			else
			{
				//expand the vector
				mCoords.resize(id + 1);
				mCoords[id] = coord;
				return true;
			}
		}

		inline void InsertPoint(dtkID id, const GK::Point3 & coord)
		{
			dtkAssert(id <= mCoords.size() && id >= 0);
			mCoords.insert(mCoords.begin() + id, coord);
		}

		inline void DeletePoint(dtkID id)
		{
			dtkAssert(id >=0 && id < mCoords.size());
			mCoords.erase(mCoords.begin() + id);
		}

		size_t GetNumberOfPoints() const
		{
			return mCoords.size();
		}

		dtkID GetMaxID() const
		{
			return static_cast<dtkID>(mCoords.size() - 1);
		}

		void Begin() const
		{
			mCurPos = 0;
		}

		bool Next(dtkID &id, GK::Point3 &coord) const
		{
			bool retVal = false;

			if (mCurPos < mCoords.size())
			{
				id = static_cast<dtkID>(mCurPos);
				coord = mCoords[mCurPos++];
				retVal = true;
			}
			
			return retVal;
		}

	private:
		dtkPointsVector()
		{
			//nothing.
		}

		dtkPointsVector(size_t size)
		{
			mCoords.resize(size);
		}

		dtkPointsVector(const std::vector<GK::Point3> &coords)
		{
			mCoords = coords;
		}

	private:
		std::vector<GK::Point3> mCoords;

		mutable size_t mCurPos;
	};
}

#endif //DTK_POINTSVECTOR_H
