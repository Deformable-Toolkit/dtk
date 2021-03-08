#ifndef DTK_PROPERTY_H
#define DTK_PROPERTY_H

#include <map>

#include <memory>
#include <boost/utility.hpp>
#include <boost/any.hpp>

#include "dtkAssert.h"

namespace dtk
{
    //! An any-type container
	template <class KeyType>
	class dtkProperty: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkProperty<KeyType> > Ptr;

		static typename dtkProperty<KeyType>::Ptr New()
		{
			return typename dtkProperty<KeyType>::Ptr(new dtkProperty<KeyType>);
		}

	public:

		template <class ValueType>
		void SetProperty(const KeyType &key, const ValueType &value)
		{
			if (mProps.find(key) != mProps.end())
				mProps[key] = boost::any(value);
			else
				mProps.insert(std::make_pair(key, boost::any(value)));
		}

		template <class ValueType>
		bool GetProperty(const KeyType &key, ValueType &value) const
		{
			typename std::map<KeyType, boost::any>::const_iterator iter;
			iter = mProps.find(key);

			bool found = (iter != mProps.end());
			assert( found );
			if (found)
			{
				const boost::any &val = iter->second;
                assert(val.type() == typeid(ValueType));
				value = boost::any_cast<ValueType>(iter->second);
			}
			return found;
		}

		void Clear()
		{
			mProps.clear();
		}

	private:
		dtkProperty() {}

		std::map<KeyType, boost::any> mProps;
	};

    //A readonly property set.
    template <class KeyType>
    class dtkReadonlyProperty: public boost::noncopyable
    {
    public:
        typedef std::shared_ptr<dtkReadonlyProperty<KeyType> > Ptr;

        static typename dtkReadonlyProperty<KeyType>::Ptr New(
                typename dtkProperty<KeyType>::Ptr prop)
        {
            return typename dtkReadonlyProperty<KeyType>::Ptr(
                             new dtkReadonlyProperty<KeyType>(prop));
        };

    public:

        template <class ValueType>
        bool GetProperty(const KeyType &key, ValueType &value) const
        {
            return mProps->GetProperty(key, value);
        }

    private:

        dtkReadonlyProperty(const typename dtkProperty<KeyType>::Ptr &prop)
            : mProps(prop)
        {}

        const typename dtkProperty<KeyType>::Ptr mProps;
    };
}

#endif //DTK_PROPERTY_H
