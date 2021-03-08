#ifndef DTK_COLLISIONDETECTPRIMITIVE_H
#define DTK_COLLISIONDETECTPRIMITIVE_H

#include <memory>
#include <boost/utility.hpp>

#include <vector>

#include "dtkConfig.h"
#include "dtkIDTypes.h"

#include "dtkPoints.h"

namespace dtk
{
	class dtkCollisionDetectPrimitive
	{
    public:
	    enum dtkCollisionDetectPrimitiveType
	    {
		    TRIANGLE = 0,
		    SEGMENT,
			SPHERE
	    };

	public:
        typedef dtkCollisionDetectPrimitiveType Type;

        dtkCollisionDetectPrimitive( Type type, dtkPoints::Ptr pts, ... );

		~dtkCollisionDetectPrimitive(){}

        void Update();

		inline void Modified()
	    {
		    mModified = true;
	    }

		inline bool IsModified() const
	    {
		    return mModified;
	    }

        inline size_t GetNumberOfPoints() const
        {
            return mNumberOfPoints;
        }

        inline const GK::Point3& GetCentroid() const
        {
            return mCentroid;
        }

        inline const GK::Object& GetObject() const
        {
            return mObject;
        }

        inline const GK::Point3& GetPoint( dtkID id ) const
        {
            return mPts->GetPoint( mIDs[id] );
        }

        inline void SetIntersected( bool intersect )
        {
            mIntersected = intersect;
        }

        inline bool IsIntersected() const
        {
            return mIntersected;
        }

		inline void SetExtend( double extend )
		{
			mExtend = extend;
		}

		inline double GetExtend()
		{
			return mExtend;
		}

        inline Type GetType()
        {
            return mType;
        }

    public:
        Type mType;
        std::vector< dtkID > mIDs;
        dtkPoints::Ptr mPts;

        dtkID mInvert;  // represent the positive and inverse of triangle 

        // Custom ID
        dtkID mMajorID; // the object id, such as liver ID
        dtkID mMinorID; // the triangle id
        dtkID mDetailIDs[3]; // the points of triangle id
        size_t mNumberOfPoints;

		dtkID mCustomID; // for region recognization, Yuguang Yang

		int mLocalID;

		bool mActive;

    private:
        GK::Object mObject;
        GK::Point3 mCentroid;

        bool mModified;
        bool mIntersected;
		double mExtend;
	};

	inline std::ostream& operator<<(std::ostream& stream, const dtkCollisionDetectPrimitive& primitive)
	{
		stream << "Primitive";
        switch( primitive.mType )
        {
            case dtkCollisionDetectPrimitive::TRIANGLE:
                stream << "( Triangle )";
                break;
            case dtkCollisionDetectPrimitive::SEGMENT:
                stream << "( Segment )";
                break;
			case dtkCollisionDetectPrimitive::SPHERE:
				stream << "( Sphere )";
				break;
            default:
                dtkAssert( false, NOT_IMPLEMENTED );
                break;
        }

        stream << "[ ";
        for( dtkID i = 0; i < primitive.GetNumberOfPoints(); i++ )
        {
            stream << primitive.mIDs[i] << " ";
            GK::Point3 point = primitive.mPts->GetPoint( primitive.mIDs[i] );
            stream << "( ";
            for( dtkID j = 0; j < 3; j++ )
                stream << point[j] << " ";
            stream << ") ";
        }
        stream << "]";
		return stream;
	}
}

#endif //DTK_COLLISIONDETECTPRIMITIVE_H
