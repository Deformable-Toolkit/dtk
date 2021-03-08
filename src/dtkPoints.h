#ifndef DTK_POINTS_H
#define DTK_POINTS_H

#include <memory>
#include <boost/utility.hpp>

#include "dtkConfig.h"
#include "dtkTx.h"
#include "dtkIDTypes.h"

#include "dtkGraphicsKernel.h"

namespace dtk
{
    //! A points container
    /*! It is used in many graphical element in DTK,
     *  such as Mesh, Graph
     */
	class dtkPoints: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkPoints> Ptr;

	public:
		virtual ~dtkPoints() {}
        virtual const GK::Point3& GetPoint(dtkID id) const = 0;
		virtual bool SetPoint(dtkID id, const GK::Point3 &coord) = 0;

		virtual size_t GetNumberOfPoints() const = 0;
		virtual dtkID GetMaxID() const = 0;

		virtual void Begin() const = 0;
		virtual bool Next(dtkID &id, GK::Point3 &coord) const = 0;

		virtual void InsertPoint(dtkID id, const GK::Point3 & coord) = 0;
		virtual void DeletePoint(dtkID id) = 0;

	};
}

#include "dtkPointsVector.h"

#endif //DTK_POINTS
