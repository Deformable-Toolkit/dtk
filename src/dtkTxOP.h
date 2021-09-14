#ifndef DTK_TXOP_H
#define DTK_TXOP_H

#include "dtkConfig.h"
#include "dtkTx.h"

#include <cmath>

namespace dtk
{
	template <class T>
	inline dtkT2<T> operator+(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	{
		return dtkT2<T>(lhs.x+rhs.x, lhs.y+rhs.y);
	}

	template <class T>
	inline dtkT3<T> operator+(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		return dtkT3<T>(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z);
	}

	template <class T>
	inline dtkT4<T> operator+(const dtkT4<T>& lhs, const dtkT4<T>& rhs)
	{
		return dtkT4<T>(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z, lhs.w+rhs.w);
	}

	template <class T>
	inline dtkT2<T> operator-(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	{
		return dtkT2<T>(lhs.x-rhs.x, lhs.y-rhs.y);
	}

	template <class T>
	inline dtkT3<T> operator-(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		return dtkT3<T>(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z);
	}

	template <class T>
	inline dtkT4<T> operator-(const dtkT4<T>& lhs, const dtkT4<T>& rhs)
	{
		return dtkT4<T>(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z, lhs.w-rhs.w);
	}

	template <class T>
	inline dtkT2<T> operator*(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	{
		return dtkT2<T>(lhs.x*rhs.x, lhs.y*rhs.y);
	}

	template <class T>
	inline dtkT2<T> operator*(const dtkT2<T>& lhs, const T &rhs)
	{
		return dtkT2<T>(lhs.x * rhs, lhs.y * rhs);
	}

	template <class T>
	inline dtkT2<T> operator*(const T& lhs, const dtkT2<T> &rhs)
	{
		return rhs * lhs;
	}

	template <class T>
	inline dtkT3<T> operator*(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		return dtkT3<T>(lhs.x*rhs.x, lhs.y*rhs.y, lhs.z*rhs.z);
	}

	template <class T>
	inline dtkT3<T> operator*(const dtkT3<T>& lhs, const T& rhs)
	{
		return dtkT3<T>(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
	}

	template <class T>
	inline dtkT3<T> operator*(const T& lhs, const dtkT3<T>& rhs)
	{
		return rhs * lhs;
	}

    template <class T>
	inline dtkT3<T> operator/(const dtkT3<T>& lhs, const T& rhs)
	{
		return dtkT3<T>(lhs.x/rhs, lhs.y/rhs, lhs.z/rhs);
	}

	template <class T>
	inline dtkT4<T> operator/(const dtkT4<T>& lhs, const T& rhs)
	{
		return dtkT4<T>(lhs.x/rhs, lhs.y/rhs, lhs.z/rhs, lhs.w/rhs);
	}

	template <class T>
	inline dtkT3<T> operator/(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		return dtkT3<T>(lhs.x/rhs.x, lhs.y/rhs.y, lhs.z/rhs.z);
	}

	template <class T>
	inline dtkT4<T> operator*(const dtkT4<T>& lhs, const dtkT4<T>& rhs)
	{
		return dtkT4<T>(lhs.x*rhs.x, lhs.y*rhs.y, lhs.z*rhs.z, lhs.w*rhs.w);
	}

	template <class T>
	inline dtkT4<T> operator*(const dtkT4<T>& lhs, const T& rhs)
	{
		return dtkT4<T>(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
	}

	template <class T>
	inline dtkT4<T> operator*(const T& lhs, const dtkT4<T>& rhs)
	{
		return rhs * lhs;
	}

	template <class T>
	inline dtkT2<T> normalize(const dtkT2<T> &v)
	{
		T m = sqrt(dot(v, v));
	//	if (m <= 0)
		if (!(m>0))
			assert(m>0);
		return dtkT2<T>( v.x/m, v.y/m);
	}

	template <class T>
	inline dtkT3<T> normalize(const dtkT3<T> &v)
	{
		T m = sqrt(dot(v, v));
	//	if (m <= 0)
		if (!(m>0))
			assert(m>0);
		return dtkT3<T>( v.x/m, v.y/m, v.z/m);
	}

	template <class T>
	inline dtkT4<T> normalize(const dtkT4<T> &v)
	{
		T m = sqrt(dot(v, v));
	//	if (m <= 0)
		if (!(m>0))
			assert(m>0);
		return dtkT4<T>( v.x/m, v.y/m, v.z/m, v.w/m);
	}

	template <class T>
	inline T length(const dtkT2<T> &v)
	{
		return sqrt(dot(v, v));
	}

	template <class T>
	inline T length(const dtkT3<T> &v)
	{
		return sqrt(dot(v, v));
	}

	template <class T>
	inline T length(const dtkT4<T> &v)
	{
		return sqrt(dot(v, v));
	}

	template <class T>
	inline T dot(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	{
		return lhs.x*rhs.x + lhs.y*rhs.y;
	}

	template <class T>
	inline T dot(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
	}

	template <class T>
	inline T dot(const dtkT4<T>& lhs, const dtkT4<T>& rhs)
	{
		return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z + lhs.w*rhs.w;
	}

	template <class T>
	inline dtkT2<T> cross(const T lhs, const dtkT2<T>& rhs)
	{
		return lhs * dtkT2<T>(-rhs.y, rhs.x);
	}

	template <class T>
	inline T cross(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	{
		return lhs.x * rhs.y - rhs.x * lhs.y;
	}

	template <class T>
	inline dtkT3<T> cross(const dtkT3<T>& lhs, const dtkT3<T>& rhs)
	{
		dtkT3<T> retVal;

		retVal.x = lhs.y * rhs.z - rhs.y * lhs.z;
		retVal.y = rhs.x * lhs.z - lhs.x * rhs.z;
		retVal.z = lhs.x * rhs.y - rhs.x * lhs.y;

		return retVal;
	}

	template <class T>
	inline dtkT2<T> normal(const dtkT2<T> &v)
	{
		dtkT2<T> res(v.y, -v.x);
		return normalize(res);
	}


	template <class T>
	inline dtkT3<T> operator-(const dtkT3<T> &v)
	{
		return dtkT3<T>(-v.x, -v.y, -v.z);
	}

        
    template <class T>
    inline dtkT3<T> barycentricWeight(const dtkT3<T> &p0, const dtkT3<T> &p1, const dtkT3<T> &p2, const dtkT3<T> &p3)
    {
        dtkT3<T> uvw;

        dtkT3<T> v03 = p0 - p3;
        dtkT3<T> v13 = p1 - p3;
        dtkT3<T> v23 = p2 - p3;

        T dot1313 = dot( v13, v13 );
        T dot1323 = dot( v13, v23 );
        T dot2323 = dot( v23, v23 );
        T dot1303 = dot( v13, v03 );
        T dot2303 = dot( v23, v03 );

        assert( dot1313 > dtkFloatEpslon );
        assert( dot2323 > dtkFloatEpslon );

        double det = dot1313 * dot2323 - dot1323 * dot1323;
        double det0 = dot1303 * dot2323 - dot1323 * dot2303;
        double det1 = dot1313 * dot2303 - dot1303 * dot1323;
        uvw[0] = det0 / det;
        uvw[1] = det1 / det;
        uvw[2] = 1.0 - uvw[0] - uvw[1];

        return uvw;
    }
}

#endif //DTK_TXOP_H
