#ifndef DTK_MATRIXOP_H
#define DTK_MATRIXOP_H

#include "dtkConfig.h"
#include "dtkTx.h"
#include "dtkMatrix.h"

#include <cmath>
#include <cassert>

namespace dtk
{
	template <class T>
	inline dtkMatrix<T> transform_identity_matrix()
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = 1;
		m(1,1) = 1;
		m(2,2) = 1;
		m(3,3) = 1;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> rotate_x_matrix(T angle)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = 1;
		m(1,1) = cos(angle);
		m(1,2) = -sin(angle);
		m(2,1) = sin(angle);
		m(2,2) = cos(angle);
		m(3,3) = 1;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> rotate_y_matrix(T angle)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = cos(angle);
		m(0,2) = sin(angle);
		m(1,1) = 1;
		m(2,0) = -sin(angle);
		m(2,2) = cos(angle);
		m(3,3) = 1;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> rotate_z_matrix(T angle)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = cos(angle);
		m(0,1) = -sin(angle);
		m(1,0) = sin(angle);
		m(1,1) = cos(angle);
		m(2,2) = 1;
		m(3,3) = 1;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> translate_matrix(T x, T y, T z)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = 1;
		m(1,1) = 1;
		m(2,2) = 1;
		m(3,3) = 1;
		m(0,3) = x;
		m(1,3) = y;
		m(2,3) = z;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> scale_matrix(T x, T y, T z)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = x;
		m(1,1) = y;
		m(2,2) = z;
		m(3,3) = 1;
		return m;
	}

	template <class T>
	inline dtkMatrix<T> align_z_matrix(dtkT3<T>& uz)
	{
		dtkT3<T> z = normalize(uz);
		dtkMatrix<T> m(4, 4, 0);
		T d = sqrt(z[1]*z[1] + z[2]*z[2]);
		if(d > 0.00001)
		{
			m(0,0) = d;
			m(1,0) = 0;
			m(2,0) = z[0];
			m(0,1) = -z[1]*z[0]/d;
			m(1,1) = z[2]/d;
			m(2,1) = z[1];
			m(0,2) = -z[2]*z[0]/d;
			m(1,2) = -z[1]/d;
			m(2,2) = z[2];
			m(3,3) = 1;
			return m;
		}
		else
		{
			if(z[0] > 0)
				return rotate_y_matrix(-(float)dtkPI/2.0f);
			else
				return rotate_y_matrix((float)dtkPI/2.0f);
		}
	}

	//template <class T>
	//inline dtkT2<T> operator+(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	//{
	//	return dtkT2<T>(lhs.x+rhs.x, lhs.y+rhs.y);
	//}

	//template <class T>
	//inline dtkT2<T> operator-(const dtkT2<T>& lhs, const dtkT2<T>& rhs)
	//{
	//	return dtkT2<T>(lhs.x-rhs.x, lhs.y-rhs.y);
	//}

	template <class T>
	inline dtkMatrix<T> operator*(const dtkMatrix<T>& lhs, const dtkMatrix<T>& rhs)
	{
		assert(lhs.nj == rhs.ni);
		dtkMatrix<T> m(lhs.ni, rhs.nj, 0);
		for(unsigned int i = 0; i < m.ni; i++)
			for(unsigned int j = 0; j < m.nj; j++)
				for(unsigned int c = 0; c < rhs.ni; c++)
					m(i,j) += lhs(i,c) * rhs(c,j);

		return m;
	}

	template <class T>
	inline dtkMatrix<T> operator*(T lhs, const dtkMatrix<T>& rhs)
	{
		dtkMatrix<T> m(rhs.ni, rhs.nj, 0);
		for(unsigned int i = 0; i < m.ni; i++)
			for(unsigned int j = 0; j < m.nj; j++)
				m(i, j) = lhs * rhs(i, j);
		return m;
	}

	template <class T>
	inline dtkMatrix<T> operator+(const dtkMatrix<T>& lhs, const dtkMatrix<T>& rhs)
	{
		assert(lhs.ni == rhs.ni);
		assert(lhs.nj == rhs.nj);

		dtkMatrix<T> m(rhs.ni, rhs.ni, 0);
		for(unsigned int i = 0; i < m.ni; i++)
			for(unsigned int j = 0; j < m.nj; j++)
				m(i, j) = lhs(i, j) + rhs(i, j);
		return m;
	}

	template <class T>
	inline dtkMatrix<T> operator-(const dtkMatrix<T>& lhs, const dtkMatrix<T>& rhs)
	{
		assert(lhs.ni == rhs.ni);
		assert(lhs.nj == rhs.nj);

		dtkMatrix<T> m(rhs.ni, rhs.ni, 0);
		for(unsigned int i = 0; i < m.ni; i++)
			for(unsigned int j = 0; j < m.nj; j++)
				m(i, j) = lhs(i, j) - rhs(i, j);
		return m;
	}
	


	template <class T>
	inline dtkT4<T> operator*(const dtkMatrix<T>& lhs, const dtkT4<T>& rhs)
	{
		assert(lhs.nj == 4);
		dtkT4<T> v;
		for(unsigned int i = 0; i < lhs.ni; i++)
			for(unsigned int c = 0; c < lhs.nj; c++)
				v[i] += lhs(i,c) * rhs[c];

		return v;
	}

	template <class T>
	inline dtkT4<T> operator*( const dtkT4<T>& lhs, const dtkMatrix<T>& rhs)
	{
		assert(rhs.ni == 4);
		dtkT4<T> v;
		for(unsigned int i = 0; i < rhs.nj; i++)
			for(unsigned int c = 0; c < rhs.ni; c++)
				v[i] += lhs[c] * rhs(c,i);

		return v;
	}

	template <class T>
	inline dtkT3<T> operator*(const dtkMatrix<T>& lhs, const dtkT3<T>& rhs)
	{
		assert(lhs.nj == 4);
		dtkT4<T> r(rhs[0], rhs[1], rhs[2], 1);
		dtkT4<T> v = lhs*r;
		dtkT3<T> result(v[0]/v[3], v[1]/v[3], v[2]/v[3]);
		return result;
	}

	template <class T>
	inline dtkT3<T> operator* (const dtkT3<T> &lhs, const dtkMatrix<T> &rhs)
	{
		assert(rhs.ni == 3);
		dtkT3<T> v;
		for (unsigned int i = 0; i < rhs.nj; i++)
			for(unsigned int c = 0; c < rhs.ni; c++)
				v[i] += lhs[c] * rhs(c, i);
		return v;
	}

	template <class T>
	inline dtkMatrix<T> transpose(const dtkMatrix<T>& im)
	{
		dtkMatrix<T> m(im.nj, im.ni);
		for(unsigned int i = 0; i < m.ni; i++)
			for(unsigned int j = 0; j < m.nj; j++)
				m(i,j) = im(j, i);

		return m;
	}

	template <class T>
	inline dtkMatrix<T> reverseTransMatrix(const dtkMatrix<T>& im)
	{
		dtkMatrix<T> m(4, 4, 0);
		m(0,0) = 1;
		m(1,1) = 1;
		m(2,2) = 1;
		m(3,3) = 1;
		m(0,3) = -im(0,3);
		m(1,3) = -im(1,3);
		m(2,3) = -im(2,3);
		return m;
	}

	//template <class T>
	//inline dtkT2<T> operator*(const dtkT2<T>& lhs, const T &rhs)
	//{
	//	return dtkT2<T>(lhs.x * rhs, lhs.y * rhs);
	//}

	//template <class T>
	//inline dtkT2<T> operator*(const T& lhs, const dtkT2<T> &rhs)
	//{
	//	return rhs * lhs;
	//}

	//template <class T>
	//inline dtkT3<T> operator-(const dtkT3<T> &v)
	//{
	//	return dtkT3<T>(-v.x, -v.y, -v.z);
	//}
	#ifdef DTK_GLM

		dtkDouble2 operator*(const dtkMatrix22 &a, const dtkDouble2 &b) {
			return {a[0][0] * b[0] + a[0][1] * b[1],
					a[1][0] * b[0] + a[1][1] * b[1]};
		}

		dtkMatrix22 operator*(const dtkMatrix22 &a, const dtkMatrix22 &b) {
			return {a[0][0] * b[0][0] + a[0][1] * b[1][0],
					a[0][0] * b[0][1] + a[0][1] * b[1][1],
					a[1][0] * b[0][0] + a[1][1] * b[1][0],
					a[1][0] * b[0][1] + a[1][1] * b[1][1]};
		}

		void operator*=(dtkMatrix22 &a, const dtkMatrix22 &b) {
			a = a * b;
		}

		dtkMatrix22 operator*(const dtkMatrix22 &a, double b) {
        return {a[0].x * b, a[0].y * b,
                a[1].x * b, a[1].y * b};
		}

		dtkMatrix22 operator*(double a, const dtkMatrix22 &b) {
			return b * a;
		}

		dtkMatrix22 rotate(double theta) {
			const auto _sin = std::sin(theta);
			const auto _cos = std::cos(theta);
			return dtkMatrix22{_cos, -_sin, _sin, _cos};
		}

	#endif
}

#endif //DTK_MATRIXOP_H
