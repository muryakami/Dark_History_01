#ifndef _JacobiAnother2_HPP_
#define _JacobiAnother2_HPP_

#include <math.h>
#include <iostream>

/*!
* Jacobi法による固有値の算出
* @param[inout] a 実対称行列．計算後，対角要素に固有値が入る
* @param[out] v 固有ベクトル(aと同じサイズ)
* @param[in] n 行列のサイズ(n×n)
* @param[in] eps 収束誤差
* @param[in] iter_max 最大反復回数
* @return 反復回数
*/
int eigenJacobiMethod(float *a, float *v, int n, float eps = 1e-8, int iter_max = 100)
{
	float *bim, *bjm;
	float bii, bij, bjj, bji;

	bim = new float[n];
	bjm = new float[n];

	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			v[i*n + j] = (i == j) ? 1.0 : 0.0;
		}
	}

	int cnt = 0;
	for (;;) {
		int i, j;

		float x = 0.0;
		for (int ia = 0; ia < n; ++ia) {
			for (int ja = 0; ja < n; ++ja) {
				int idx = ia*n + ja;
				if (ia != ja && fabs(a[idx]) > x) {
					i = ia;
					j = ja;
					x = fabs(a[idx]);
				}
			}
		}

		float aii = a[i*n + i];
		float ajj = a[j*n + j];
		float aij = a[i*n + j];

		float alpha, beta;
		alpha = (aii - ajj) / 2.0;
		beta = sqrt(alpha*alpha + aij*aij);

		float st, ct;
		ct = sqrt((1.0 + fabs(alpha) / beta) / 2.0);    // sinθ
		st = (((aii - ajj) >= 0.0) ? 1.0 : -1.0)*aij / (2.0*beta*ct);    // cosθ

																		 // A = PAPの計算
		for (int m = 0; m < n; ++m) {
			if (m == i || m == j) continue;

			float aim = a[i*n + m];
			float ajm = a[j*n + m];

			bim[m] = aim*ct + ajm*st;
			bjm[m] = -aim*st + ajm*ct;
		}

		bii = aii*ct*ct + 2.0*aij*ct*st + ajj*st*st;
		bij = 0.0;

		bjj = aii*st*st - 2.0*aij*ct*st + ajj*ct*ct;
		bji = 0.0;

		for (int m = 0; m < n; ++m) {
			a[i*n + m] = a[m*n + i] = bim[m];
			a[j*n + m] = a[m*n + j] = bjm[m];
		}
		a[i*n + i] = bii;
		a[i*n + j] = bij;
		a[j*n + j] = bjj;
		a[j*n + i] = bji;

		// V = PVの計算
		for (int m = 0; m < n; ++m) {
			float vmi = v[m*n + i];
			float vmj = v[m*n + j];

			bim[m] = vmi*ct + vmj*st;
			bjm[m] = -vmi*st + vmj*ct;
		}
		for (int m = 0; m < n; ++m) {
			v[m*n + i] = bim[m];
			v[m*n + j] = bjm[m];
		}

		float e = 0.0;
		for (int ja = 0; ja < n; ++ja) {
			for (int ia = 0; ia < n; ++ia) {
				if (ia != ja) {
					e += fabs(a[ja*n + ia]);
				}
			}
		}
		if (e < eps) break;

		cnt++;
		if (cnt > iter_max) break;
	}

	delete[] bim;
	delete[] bjm;

	return cnt;
}

void RunEigenJacobiMethod(void) {
	int n = 3;
	/*float **A = new float *[n];
	float **V = new float *[n];
	for (int i = 0; i < n; i++) {
		A[i] = new float[n];
		V[i] = new float[n];
	}*/

	float A[3][3] = {};
	float V[3][3] = {};

	//A[0][0] = Sxx;	A[0][1] = Sxy;	A[0][2] = Sxz;
	//A[1][0] = Sxy;	A[1][1] = Syy;	A[1][2] = Syz;
	//A[2][0] = Sxz;	A[2][1] = Syz;	A[2][2] = Szz;

	//A[0][0] = 1.0f;	A[0][1] = 1.0f;	A[0][2] = 0.0f;
	//A[1][0] = 1.0f;	A[1][1] = 2.0f;	A[1][2] = 1.0f;
	//A[2][0] = 2.0f;	A[2][1] = 5.0f;	A[2][2] = 3.0f;

	A[0][0] = 1.0f;	A[0][1] = 1.0f;	A[0][2] = 2.0f;
	A[1][0] = 1.0f;	A[1][1] = 2.0f;	A[1][2] = 5.0f;
	A[2][0] = 0.0f;	A[2][1] = 1.0f;	A[2][2] = 3.0f;

	int result = eigenJacobiMethod(*A, *V, n);

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			std::cout << A[i][j] << "\t";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			std::cout << V[i][j] << "\t";
		}
		std::cout << std::endl;
	}
}

#endif // _JacobiAnother2_HPP_