#ifndef DTK_TX_H
#define DTK_TX_H

#include "dtkConfig.h"
#include "dtkAssert.h"
#include "dtkError.h"
#include "dtkErrorManager.h"

namespace dtk
{
	//------------------------------------------------------
	template <class T>
	struct dtkT2
	{
		T x, y;

		dtkT2(const T& px=0, const T& py=0)
			:x(px), y(py)
		{}

		dtkT2(const dtkT2<T>& rhs)
			:x(rhs.x), y(rhs.y)
		{}

		
		const T& operator[](const int& n) const
		{
			switch (n)
			{
				case 0: return x;
				case 1:	return y;
				default:
					dtkAssert(false, OUT_OF_RANGE);
					return y;
			}
		}

		T& operator[](const int& n)
		{
			return
				const_cast<T&>
				(
					static_cast<const dtkT2<T>&>(*this)[n]
				);
		}
	};

	typedef dtkT2<int>		dtkInt2;
	typedef dtkT2<float>	dtkFloat2;
	typedef dtkT2<double>	dtkDouble2;

	//------------------------------------------------------
	template <class T>
	struct dtkT3
	{
		T x, y, z;

		dtkT3(const T& px=0, const T& py=0, const T& pz=0)
			:x(px), y(py), z(pz)
		{}

		dtkT3(const dtkT3<T>& rhs)
			:x(rhs.x), y(rhs.y), z(rhs.z)
		{}

		const T& operator[](const int& n) const
		{
			switch (n)
			{
				case 0: return x;
				case 1: return y;
				case 2: return z;
				default:
					dtkAssert(false, OUT_OF_RANGE);
					return z;
			}
		}

		T& operator[](const int& n)
		{
			return const_cast<T&>(
					static_cast<const dtkT3<T>&>(*this)[n]
				   );
		}

	};

	typedef dtkT3<int>		dtkInt3;
	typedef dtkT3<float>	    dtkFloat3;
	typedef dtkT3<double>	dtkDouble3;

	//------------------------------------------------------
	template <class T>
	struct dtkT4
	{
		T x, y, z, w;

		dtkT4(const T& px=0, const T& py=0, const T& pz=0, const T& pw=0)
			:x(px), y(py), z(pz), w(pw)
		{}

		dtkT4(const dtkT4<T>& rhs)
			:x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w)
		{}

		const T& operator[](const int& n) const
		{
			switch (n)
			{
				case 0: return x;
				case 1: return y;
				case 2: return z;
				case 3: return w;
				default:
					dtkAssert(false, OUT_OF_RANGE);
					return w;
			}
		}

		T& operator[](const int& n)
		{
			return const_cast<T&>(
					static_cast<const dtkT4<T>&>(*this)[n]
					);
		}

	};

	typedef dtkT4<int>		dtkInt4;
	typedef dtkT4<float>	dtkFloat4;
	typedef dtkT4<double>	dtkDouble4;
}

#include "dtkTxIO.h"
#include "dtkTxOP.h"

#endif //DTK_TX_H
