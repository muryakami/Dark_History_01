#ifndef _Principal_HPP_
#define _Principal_HPP_

#include "Jacobi.hpp"
#include <Math.h>

int principal(int p, int n, double **x, double *r, double **a, double eps, int ct)
{
	double **A1, **A2, **C, mean, **X1, **X2, s2;
	int i1, i2, i3, sw = 0;
	// 領域の確保
	C = new double *[p];
	A1 = new double *[p];
	A2 = new double *[p];
	X1 = new double *[p];
	X2 = new double *[p];
	for (i1 = 0; i1 < p; i1++) {
		C[i1] = new double[p];
		A1[i1] = new double[p];
		A2[i1] = new double[p];
		X1[i1] = new double[p];
		X2[i1] = new double[p];
	}
	// データの基準化
	for (i1 = 0; i1 < p; i1++) {
		mean = 0.0;
		s2 = 0.0;
		for (i2 = 0; i2 < n; i2++) {
			mean += x[i1][i2];
			s2 += x[i1][i2] * x[i1][i2];
		}
		mean /= n;
		s2 /= n;
		s2 = n * (s2 - mean * mean) / (n - 1);
		s2 = sqrt(s2);
		for (i2 = 0; i2 < n; i2++)
			x[i1][i2] = (x[i1][i2] - mean) / s2;
	}
	// 分散共分散行列の計算
	for (i1 = 0; i1 < p; i1++) {
		for (i2 = i1; i2 < p; i2++) {
			s2 = 0.0;
			for (i3 = 0; i3 < n; i3++)
				s2 += x[i1][i3] * x[i2][i3];
			s2 /= (n - 1);
			C[i1][i2] = s2;
			if (i1 != i2)
				C[i2][i1] = s2;
		}
	}
	// 固有値と固有ベクトルの計算（ヤコビ法）
	sw = Jacobi(p, ct, eps, C, A1, A2, X1, X2);

	if (sw == 0) {
		for (i1 = 0; i1 < p; i1++) {
			r[i1] = A1[i1][i1];
			for (i2 = 0; i2 < p; i2++)
				a[i1][i2] = X1[i2][i1];
		}
	}
	// 領域の解放
	for (i1 = 0; i1 < p; i1++) {
		delete[] C[i1];
		delete[] A1[i1];
		delete[] A2[i1];
		delete[] X1[i1];
		delete[] X2[i1];
	}
	delete[] C;
	delete[] A1;
	delete[] A2;
	delete[] X1;
	delete[] X2;

	return sw;
}

#endif