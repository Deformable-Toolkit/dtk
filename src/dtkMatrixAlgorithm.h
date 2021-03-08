/*
 interface: general matrix algorithm
 include :	Determinant
			cholesky decompression
*/
#ifndef _DTK_MATRIX_ALGORITHM_
#define _DTK_MATRIX_ALGORITHM_

#include <dtkConfig.h>

namespace dtk
{
	namespace MatrixAlgo
	{
		//Determinant of 4x4 matrix
		extern double Det4x4(double *A);

		extern double Det4x4(double a11, double a12, double a13, double a14, 
							double a21, double a22, double a23, double a24,
							double a31, double a32, double a33, double a34,
							double a41, double a42, double a43, double a44);

		extern double Det4x4(float *A);

		extern double Det4x4(float a11, float a12, float a13, float a14, 
							float a21, float a22, float a23, float a24,
							float a31, float a32, float a33, float a34,
							float a41, float a42, float a43, float a44);

		//Determinant of 3x3 matrix
		extern double Det3x3(double *A);

		extern double Det3x3(double a11, double a12, double a13,
							double a21, double a22, double a23, 
							double a31, double a32, double a33);

		extern double Det3x3(float *A);

		extern double Det3x3(float a11, float a12, float a13,
							float a21, float a22, float a23, 
							float a31, float a32, float a33);

		/////////////////////////////////////////////////////////////////////////////////////
		/*
		cholesky decompression: for symmetric matrix A
		A = L*L', where A must be positive definition
		or A = L*D*L', where L's diagonal elements are 1 & A can be indefinited
		*/
		extern double* dtkCholesky(double* Kii,	// full stored, row major
								size_t length,
								double* LowKii	// low triangle stored, row major
								);

		extern void dtkCholesky2(double* A,		// full stored, row major
								size_t length, 
								double* L,		// low triangle stored, row major
								double* D		// diagonal vector
								);

		/*++
		cholesky_pack(), cholesky_pack2(), invert()
		A is stored low triangle, column major
		Caution: results replace A!
		--*/
		extern bool cholesky_pack(double* A, const size_t &n);

		extern bool cholesky_pack2(double* A, const size_t &n);	// D replace A's diagonal

		extern bool cholesky_pack3(double* A, const size_t &n);	// K = L*(D^-1)*L'

		// A is replaced by its invert matrix
		extern bool invert(double *A, const size_t &n);	// A must be positive definition

		// solution of A*x = b
		// A must be decompressed by cholesky_pack3 firstly
		extern bool solution_chol3(const size_t &n, double* A, const size_t &nb, double *b, double *x);

		//////////////////////////////////////////////////////////////////////
		// 求实对称矩阵特征值与特征向量的雅可比过关法
		//
		// Parameters：
		// 1. const size_t n - degree of A
		// 2. const double *A - matrix A, row major
		// 3. double *dblEigenValue - 1D vector, for storing eigenvalues
		// 4. double *mtxEigenVector - for storing eigenvectors(column major)
		// 5. double eps - accuracy, default = 0.000001
		//
		// Return：bool
		//////////////////////////////////////////////////////////////////////
		bool JacobiEigenv2(const size_t &n, double *A, double *dblEigenValue, double *mtxEigenVector, double eps = 0.000001);
		
		/*
		Solve eigenvalue problem: K*V = M*V*E, using Lanczos method
		E : eigenvalues
		V : eigenvectors
		*/
		extern bool getEigen(const size_t &n,	// degree of K & M
							const double *K,	// low-triangle matrix
							const double *M,	// consistent mass vector
							size_t ne,	// number of eigenvalue
							double *E,	// eigenvalues
							double *V	// n * ne matrix for eigenvectors
							);
	}
}

#endif
