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
    /**
    * @class <dtkCollisionDetectPrimitive> 
    * @brief 碰撞检测图元
    * @author <>
    * @note
    * 碰撞检测图元，描述每个碰撞检测的物体。
    */
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

        /**
         * @brief 更新质心及图元对象顶点
         */
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
        Type mType;  /**< 图元类型 */
        std::vector< dtkID > mIDs; /**< 点ID集 */
        dtkPoints::Ptr mPts; /**< 点集 */

        dtkID mInvert;  /**< represent the positive and inverse of triangle 三角形正反、内外朝向 */

        // Custom ID
        dtkID mMajorID; /**< the object id, such as liver ID,主Id */
        dtkID mMinorID; /**< the triangle id,副id. */
        dtkID mDetailIDs[3]; /**< the points of triangle id, 三角形图元的三个点的ID */
        size_t mNumberOfPoints; /**< 图元点数量 */

		dtkID mCustomID; /**< for region recognization, Yuguang Yang */

		int mLocalID;

		bool mActive;  /**< 图元是否活动状态 */

    private:
        GK::Object mObject; /**< 图元对象 */
        GK::Point3 mCentroid;  /**< 质心 */

        bool mModified;   /**< 是否更改 */
        bool mIntersected; /**< 是否与其他图元相交 */
		double mExtend; /**< 是否有相交间隔 */
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
