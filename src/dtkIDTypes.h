#ifndef DTK_IDTYPES_H
#define DTK_IDTYPES_H

#include "dtkConfig.h"
#include "dtkAssert.h"

#include <iostream>

namespace dtk
{	
	/**
	 * @brief ID
	 * @note note that the dtkID must be a 32-bit unsigned-int.
	 * The Array-Based Mesh Data Structure depends on this feacture.
	*/
	typedef dtkWORD dtkID;
	
	const dtkID dtkErrorID = 0xFFFFFFFF;

	struct dtkID2
	{
		dtkID a, b;

		dtkID2(const dtkID &x=0, const dtkID &y=0)
			:a(x), b(y)
		{}

		dtkID2(const dtkID2 &rhs)
			:a(rhs.a), b(rhs.b)
		{}

		const dtkID& operator[](const int &n) const
		{
			switch (n)
			{
			case 0: return a;
			case 1: return b;
			default:
				dtkAssert(false, OUT_OF_RANGE);
				return b;
			}
		}

		dtkID& operator[](const int &n)
		{
			return const_cast<dtkID&>(
					static_cast<const dtkID2&>(*this)[n]
					);
		}

		bool operator==(const dtkID2 &rhs) const
		{
			return (a == rhs.a && b == rhs.b);
		}

		bool operator<(const dtkID2 &rhs) const
		{
			if (a < rhs.a) return true;
			if (a > rhs.a) return false;

			if (b < rhs.b) return true;
			if (b > rhs.b) return false;

			return false;
		}
	};

    inline std::ostream& operator<<(std::ostream &out, const dtkID2 &id)
    {
        out<<"{"<<id.a<<","<<id.b<<"}";
        return out;
    }

    inline dtkID2 sort(const dtkID &id0, const dtkID &id1)
    {
        return (id0 < id1? dtkID2(id0, id1): dtkID2(id1, id0));
    }

    inline dtkID2 sort(const dtkID2 &id)
    {
        return sort(id.a, id.b);
    }

	struct dtkID3
	{
		dtkID a, b, c;

		dtkID3(const dtkID &x=0, const dtkID &y=0, const dtkID &z=0)
			:a(x), b(y), c(z)
		{}

		dtkID3(const dtkID3 &rhs)
			:a(rhs.a), b(rhs.b), c(rhs.c)
		{}

		const dtkID& operator[](const int &n) const
		{
			switch (n)
			{
			case 0: return a;
			case 1:	return b;
			case 2:	return c;
			default:
				dtkAssert(false, OUT_OF_RANGE);
				return b;
			}
		}

		dtkID& operator[](const int &n)
		{
			return const_cast<dtkID&>(
					static_cast<const dtkID3&>(*this)[n]
				   );
		}

		bool operator==(const dtkID3 &rhs) const
		{
			return (a == rhs.a && b == rhs.b && c == rhs.c);
		}

		bool operator<(const dtkID3 &rhs) const
		{
			if (a < rhs.a) return true;
			if (a > rhs.a) return false;

			if (b < rhs.b) return true;
			if (b > rhs.b) return false;

			if (c < rhs.c) return true;
			if (c > rhs.c) return false;

			return false;
		}
	};

    inline dtkID3 sort(const dtkID &id0, const dtkID &id1, const dtkID &id2)
    {
        dtkID3 rtn(id0, id1, id2);
        if (rtn.a > rtn.b) std::swap(rtn.a, rtn.b);
        if (rtn.a > rtn.c) std::swap(rtn.a, rtn.c);
        if (rtn.b > rtn.c) std::swap(rtn.b, rtn.c);

        return rtn;
    }

    inline dtkID3 sort(const dtkID3 &id)
    {
        return sort(id.a, id.b, id.c);
    }

    inline std::ostream& operator<<(std::ostream &out, const dtkID3 &id)
    {
        out<<"{"<<id.a<<","<<id.b<<","<<id.c<<"}";
        return out;
    }

	struct dtkID4
	{
		dtkID a, b, c, d;

		dtkID4(const dtkID &ta=0, const dtkID &tb=0, 
			   const dtkID &tc=0, const dtkID &td=0)
			:a(ta), b(tb), c(tc), d(td)
		{}

		dtkID4(const dtkID4 &rhs)
			:a(rhs.a), b(rhs.b), c(rhs.c), d(rhs.d)
		{}

		const dtkID& operator[](const int &n) const
		{
			switch (n)
			{
			case 0:	return a;
			case 1:	return b;
			case 2:	return c;
			case 3:	return d;
			default:
				dtkAssert(false, OUT_OF_RANGE);
				return d;
			}
		}

		dtkID& operator[](const int &n)
		{
			return const_cast<dtkID&>(
					static_cast<const dtkID4&>(*this)[n]
				   );
		}

		bool operator==(const dtkID4 &rhs) const
		{
			return (a==rhs.a && b==rhs.b && c==rhs.c && d==rhs.d);
		}

		bool operator<(const dtkID4 &rhs) const
		{
			if (a < rhs.a) return true;
			if (a > rhs.a) return false;

			if (b < rhs.b) return true;
			if (b > rhs.b) return false;

			if (c < rhs.c) return true;
			if (c > rhs.c) return false;

			if (d < rhs.d) return true;
			if (d > rhs.d) return false;

            
			return false;
		}
	};

    inline dtkID4 sort(const dtkID &a, const dtkID &b, const dtkID &c, const dtkID &d)
    {
        dtkID4 rtn(a, b, c, d);

        if (rtn.a > rtn.b) std::swap(rtn.a, rtn.b);
        if (rtn.a > rtn.c) std::swap(rtn.a, rtn.c);
        if (rtn.a > rtn.d) std::swap(rtn.a, rtn.d);
        if (rtn.b > rtn.c) std::swap(rtn.b, rtn.c);
        if (rtn.b > rtn.d) std::swap(rtn.b, rtn.d);
        if (rtn.c > rtn.d) std::swap(rtn.d, rtn.c);

        return rtn;
    }

    inline dtkID4 sort(const dtkID4 &id)
    {
        return sort(id.a, id.b, id.c, id.d);
    }

    inline std::ostream& operator<<(std::ostream &out, const dtkID4 &id)
    {
        out<<"{"<<id.a<<","<<id.b<<","<<id.c<<","<<id.d<<"}";
        return out;
    }
}

#endif //DTK_IDTYPES_H
