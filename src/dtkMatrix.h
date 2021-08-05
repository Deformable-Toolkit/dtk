#ifndef DTK_MATRIX_H
#define DTK_MATRIX_H

#include "dtkConfig.h"
#include <vector>
#include <cassert>

#ifdef DTK_GLM
	#include <glm/glm.hpp>
	#include <glm/gtc/matrix_transform.hpp>
	#include <glm/gtc/type_ptr.hpp>
#endif

namespace dtk
{
	template<class T, class MatrixT=std::vector<T> >
	struct dtkMatrix
	{
		// STL-friendly typedefs

		typedef typename MatrixT::iterator iterator;
		typedef typename MatrixT::const_iterator const_iterator;
		typedef typename MatrixT::size_type size_type;
		typedef long difference_type;
		typedef T& reference;
		typedef const T& const_reference;
		typedef T value_type;
		typedef T* pointer;
		typedef const T* const_pointer;
		typedef typename MatrixT::reverse_iterator reverse_iterator;
		typedef typename MatrixT::const_reverse_iterator const_reverse_iterator;

		// the actual representation

		unsigned int ni, nj;
		MatrixT a;

		// the interface

		dtkMatrix(void)
			: ni(0), nj(0)
		{}

		dtkMatrix(int ni_, int nj_)
			: ni(ni_), nj(nj_), a(ni_*nj_)
		{ assert(ni_>=0 && nj>=0); }

		dtkMatrix(int ni_, int nj_, MatrixT& a_)
			: ni(ni_), nj(nj_), a(a_)
		{ assert(ni_>=0 && nj>=0); }

		dtkMatrix(int ni_, int nj_, const T& value)
			: ni(ni_), nj(nj_), a(ni_*nj_, value)
		{ assert(ni_>=0 && nj>=0); }

		dtkMatrix(int ni_, int nj_, const T& value, size_type max_n_)
			: ni(ni_), nj(nj_), a(ni_*nj_, value, max_n_)
		{ assert(ni_>=0 && nj>=0); }

		void init(const T* values)
		{
			for(unsigned int i = 0; i < ni*nj; i++)
				a[i] = values[i];
		}

		//dtkMatrix(int ni_, int nj_, T* data_)
		//	: ni(ni_), nj(nj_), a(ni_*nj_, data_)
		//{ assert(ni_>=0 && nj>=0); }

		//dtkMatrix(int ni_, int nj_, T* data_, size_type max_n_)
		//	: ni(ni_), nj(nj_), a(ni_*nj_, data_, max_n_)
		//{ assert(ni_>=0 && nj>=0); }

		template<class OtherMatrixT>
		dtkMatrix(dtkMatrix<T, OtherMatrixT>& other)
			: ni(other.ni), nj(other.nj), a(other.a)
		{}

		~dtkMatrix(void)
		{
#ifndef NDEBUG
			ni=nj=0;
#endif
		}

		const T& operator()(unsigned int i, unsigned int j) const
		{
			assert(i>=0 && i<ni && j>=0 && j<nj);
			return a[i+ni*j];
		}

		T& operator()(unsigned int i, unsigned int j)
		{
			assert(i>=0 && i<ni && j>=0 && j<nj);
			return a[i+ni*j];
		}

		const T& operator[](unsigned int i) const
		{
			assert(i>=0 && i<ni * nj);
			return a[i];
		}

		T& operator[](unsigned int i)
		{
			assert(i>=0 && i<ni * nj);
			return a[i];
		}

		bool operator==(const dtkMatrix<T>& x) const
		{ return ni==x.ni && nj==x.nj && a==x.a; }

		bool operator!=(const dtkMatrix<T>& x) const
		{ return ni!=x.ni || nj!=x.nj || a!=x.a; }

		bool operator<(const dtkMatrix<T>& x) const
		{
			if(ni<x.ni) return true; else if(ni>x.ni) return false;
			if(nj<x.nj) return true; else if(nj>x.nj) return false;
			return a<x.a;
		}

		bool operator>(const dtkMatrix<T>& x) const
		{
			if(ni>x.ni) return true; else if(ni<x.ni) return false;
			if(nj>x.nj) return true; else if(nj<x.nj) return false;
			return a>x.a;
		}

		bool operator<=(const dtkMatrix<T>& x) const
		{
			if(ni<x.ni) return true; else if(ni>x.ni) return false;
			if(nj<x.nj) return true; else if(nj>x.nj) return false;
			return a<=x.a;
		}

		bool operator>=(const dtkMatrix<T>& x) const
		{
			if(ni>x.ni) return true; else if(ni<x.ni) return false;
			if(nj>x.nj) return true; else if(nj<x.nj) return false;
			return a>=x.a;
		}

		void assign(const T& value)
		{ a.assign(value); }

		void assign(int ni_, int nj_, const T& value)
		{
			a.assign(ni_*nj_, value);
			ni=ni_;
			nj=nj_;
		}

		void assign(int ni_, int nj_, const T* copydata)
		{
			a.assign(ni_*nj_, copydata);
			ni=ni_;
			nj=nj_;
		}

		const T& at(int i, int j) const
		{
			assert(i>=0 && i<ni && j>=0 && j<nj);
			return a[i+ni*j];
		}

		T& at(int i, int j)
		{
			assert(i>=0 && i<ni && j>=0 && j<nj);
			return a[i+ni*j];
		}

		const T& back(void) const
		{ 
			assert(a.size());
			return a.back();
		}

		T& back(void)
		{
			assert(a.size());
			return a.back();
		}

		const_iterator begin(void) const
		{ return a.begin(); }

		iterator begin(void)
		{ return a.begin(); }

		size_type capacity(void) const
		{ return a.capacity(); }

		void clear(void)
		{
			a.clear();
			ni=nj=0;
		}

		bool empty(void) const
		{ return a.empty(); }

		const_iterator end(void) const
		{ return a.end(); }

		iterator end(void)
		{ return a.end(); }

		void fill(int ni_, int nj_, const T& value)
		{
			a.fill(ni_*nj_, value);
			ni=ni_;
			nj=nj_;
		}

		const T& front(void) const
		{
			assert(a.size());
			return a.front();
		}

		T& front(void)
		{
			assert(a.size());
			return a.front();
		}

		size_type max_size(void) const
		{ return a.max_size(); }

		reverse_iterator rbegin(void)
		{ return reverse_iterator(end()); }

		const_reverse_iterator rbegin(void) const
		{ return const_reverse_iterator(end()); }

		reverse_iterator rend(void)
		{ return reverse_iterator(begin()); }

		const_reverse_iterator rend(void) const
		{ return const_reverse_iterator(begin()); }

		void reserve(int reserve_ni, int reserve_nj)
		{ a.reserve(reserve_ni*reserve_nj); }

		void resize(int ni_, int nj_)
		{
			assert(ni_>=0 && nj_>=0);
			a.resize(ni_*nj_);
			ni=ni_;
			nj=nj_;
		}

		void resize(int ni_, int nj_, const T& value)
		{
			assert(ni_>=0 && nj_>=0);
			a.resize(ni_*nj_, value);
			ni=ni_;
			nj=nj_;
		}

		void set_zero(void)
		{ a.set_zero(); }

		size_type size(void) const
		{ return a.size(); }

		void swap(dtkMatrix<T>& x)
		{
			std::swap(ni, x.ni);
			std::swap(nj, x.nj);
			a.swap(x.a);
		}

		void trim(void)
		{ a.trim(); }
	};

	// some common arrays
	typedef dtkMatrix<double, std::vector<double> >                         dtkMatrixDouble;
	typedef dtkMatrix<float, std::vector<float> >                           dtkMatrixFloat;
	typedef dtkMatrix<long long, std::vector<long long> >                   dtkMatrixLLong;
	typedef dtkMatrix<unsigned long long, std::vector<unsigned long long> > dtkMatrixULLong;
	typedef dtkMatrix<int, std::vector<int> >                               dtkMatrixInt;
	typedef dtkMatrix<unsigned int, std::vector<unsigned int> >             dtkMatrixUInt;
	typedef dtkMatrix<short, std::vector<short> >                           dtkMatrixShort;
	typedef dtkMatrix<unsigned short, std::vector<unsigned short> >         dtkMatrixUShort;
	typedef dtkMatrix<char, std::vector<char> >                             dtkMatrixChar;
	typedef dtkMatrix<unsigned char, std::vector<unsigned char> >           dtkMatrixUChar;

#ifdef DTK_GLM
	typedef glm::mat2x2 dtkMatrix22;
	typedef glm::mat2x3 dtkMatrix23;
	typedef glm::mat3x2 dtkMatrix32;
	typedef glm::mat3x3 dtkMatrix33;
	typedef glm::mat2x4 dtkMatrix24;
	typedef glm::mat4x2 dtkMatrix42;
	typedef glm::mat3x4 dtkMatrix34;
	typedef glm::mat4x3 dtkMatrix43;
	typedef glm::mat4x4 dtkMatrix44;
#endif

}

#include "dtkMatrixOP.h"

#endif //DTK_MATRIX_H
