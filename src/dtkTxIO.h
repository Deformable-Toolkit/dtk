#ifndef DTK_TXIO_H
#define DTK_TXIO_H

//STL
#include <iostream>

//DTK
#include "dtkConfig.h"
#include "dtkTx.h"

namespace dtk
{
	template <class T>
	inline std::ostream& operator<<(std::ostream& stream, const dtkT2<T>& data)
	{
		stream<<'('<<data.x<<','<<data.y<<')';
		return stream;
	}

	template <class T>
	inline std::ostream& operator<<(std::ostream& stream, const dtkT3<T>& data)
	{
		stream<<'('<<data.x<<','<<data.y<<','<<data.z<<')';
		return stream;
	}

	template <class T>
	inline std::ostream& operator<<(std::ostream& stream, const dtkT4<T>& data)
	{
		stream<<'('<<data.x<<','<<data.y<<','<<data.z<<','<<data.w<<')';
		return stream;
	}
}

#endif //DTK_TXIO_H
