#ifndef DTK_ARRAY_H
#define DTK_ARRAY_H

#include <vector>
#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
    template <class T>
    class dtkArray: boost::noncopyable
    {
    public:
        typedef std::shared_ptr< dtkArray<T> > Ptr;

        static typename dtkArray::Ptr New(size_t size = 100)
        {
            return typename dtkArray<T>::Ptr(new dtkArray<T>(size));
        }

    public:

        const T& At(size_t offset) const
        {
            dtkAssert(offset < mData.size(), OUT_OF_RANGE);
            return mData[offset];
        }

        T& At(size_t offset)
        {
            dtkAssert(offset < mData.size(), OUT_OF_RANGE);
            return mData[offset];
        }

        T GetAt(size_t offset) const
        {
            dtkAssert(offset < mData.size(), OUT_OF_RANGE);
            return mData[offset];
        }

        void SetAt(size_t offset, const T& data)
        {
            if (offset >= mData.size())
                mData.resize(offset * 2);

            mData[offset] = data;
        }

        void PushBack(const T& data)
        {
            mData.push_back(data);
        }

        void Reserve(size_t size)
        {
            mData.reserve(size);
        }

        size_t Size() const
        {
            return mData.size();
        }

    private:

        dtkArray(size_t size)
        {
            mData.reserve(size);
        }

        std::vector<T>          mData;
    };
}

#endif //DTK_ARRAY_H
