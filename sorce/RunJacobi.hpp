#ifndef _RunJacobi_HPP_
#define _RunJacobi_HPP_

#include "Jacobi.hpp"
#include <array>

using namespace std;


//array<array<double, 4>, 3> run_Jacobi(int n, vector<double> VCM) {
array<array<double, 4>, 3> run_Jacobi(double Sxx, double Sxy, double Sxz, double Syy, double Syz, double Szz) {
	double **A, **A1, **A2, **X1, **X2, eps;
	int i1, i2, ind, ct, n;
	// データの設定
	ct = 1000;
	eps = 1.0e-10;
	n = 3;
	A = new double *[n];
	A1 = new double *[n];
	A2 = new double *[n];
	X1 = new double *[n];
	X2 = new double *[n];
	for (i1 = 0; i1 < n; i1++) {
		A[i1] = new double[n];
		A1[i1] = new double[n];
		A2[i1] = new double[n];
		X1[i1] = new double[n];
		X2[i1] = new double[n];
	}

	A[0][0] = Sxx;
	A[0][1] = Sxy;
	A[0][2] = Sxz;

	A[1][0] = Sxy;
	A[1][1] = Syy;
	A[1][2] = Syz;

	A[2][0] = Sxz;
	A[2][1] = Syz;
	A[2][2] = Szz;


	// 計算
	ind = Jacobi(n, ct, eps, A, A1, A2, X1, X2);

	array<array<double, 4>, 3> result;

	int type2 = 0;
	if (A1[0][0] < A1[1][1]) {
		type2 += 1;
	}
	if (A1[1][1] < A1[2][2]) {
		type2 += 2;
	}
	if (A1[2][2] < A1[0][0]) {
		type2 += 4;
	}
	int type3 = 0;
	switch (type2) {
	case 1:
		type3 = 7;
		break;
	case 2:
		type3 = 11;
		break;
	case 3:
		type3 = 5;
		break;
	case 4:
		type3 = 21;
		break;
	case 5:
		type3 = 19;
		break;
	case 6:
		type3 = 15;
		break;
	default:
		break;
	}
	int index = type3 / 9;
	result[0][0] = A1[index][index];
	result[0][1] = X1[0][index];
	result[0][2] = X1[1][index];
	result[0][3] = X1[2][index];
	type3 -= index * 9;
	index = type3 / 3;
	result[1][0] = A1[index][index];
	result[1][1] = X1[0][index];
	result[1][2] = X1[1][index];
	result[1][3] = X1[2][index];
	type3 -= index * 3;
	index = type3;
	result[2][0] = A1[index][index];
	result[2][1] = X1[0][index];
	result[2][2] = X1[1][index];
	result[2][3] = X1[2][index];

	/*
	for (int i = 0; i < n; i++) {
	for (int j = 0; j < n; j++) {
	cout << "A[" << i << "][" << j << "]: " << A[i][j] << "\t";
	}
	cout << endl;
	}
	cout << endl;
	for (int i = 0; i < n; i++) {
	for (int j = 0; j < n; j++) {
	cout << "A1[" << i << "][" << j << "]: " << A1[i][j] << "\t";
	}
	cout << endl;
	}
	cout << endl;
	for (int i = 0; i < n; i++) {
	for (int j = 0; j < n; j++) {
	cout << "X1[" << i << "][" << j << "]: " << X1[i][j] << "\t";
	}
	cout << endl;
	}
	cout << endl;
	cout << endl;
	*/

	for (i1 = 0; i1 < n; i1++) {
		delete[] A[i1];
		delete[] A1[i1];
		delete[] A2[i1];
		delete[] X1[i1];
		delete[] X2[i1];
	}
	delete[] A;
	delete[] A1;
	delete[] A2;
	delete[] X1;
	delete[] X2;

	return result;
}

array<array<double, 3>, 2> run_Jacobi2(double Sxx, double Sxy, double Syy) {
	double **A, **A1, **A2, **X1, **X2, eps;
	int i1, i2, ind, ct, n;
	// データの設定
	ct = 1000;
	eps = 1.0e-10;
	n = 2;
	A = new double *[n];
	A1 = new double *[n];
	A2 = new double *[n];
	X1 = new double *[n];
	X2 = new double *[n];
	for (i1 = 0; i1 < n; i1++) {
		A[i1] = new double[n];
		A1[i1] = new double[n];
		A2[i1] = new double[n];
		X1[i1] = new double[n];
		X2[i1] = new double[n];
	}

	A[0][0] = Sxx;
	A[0][1] = Sxy;

	A[1][0] = Sxy;
	A[1][1] = Syy;


	// 計算
	ind = Jacobi(n, ct, eps, A, A1, A2, X1, X2);

	array<array<double, 3>, 2> result;

	int index = 1;
	if (A1[0][0] < A1[1][1]) {
		index = 0;
	}
	result[0][0] = A1[index][index];
	result[0][1] = X1[0][index];
	result[0][2] = X1[1][index];
	index++;
	index %= 2;
	result[1][0] = A1[index][index];
	result[1][1] = X1[0][index];
	result[1][2] = X1[1][index];


	for (i1 = 0; i1 < n; i1++) {
		delete[] A[i1];
		delete[] A1[i1];
		delete[] A2[i1];
		delete[] X1[i1];
		delete[] X2[i1];
	}
	delete[] A;
	delete[] A1;
	delete[] A2;
	delete[] X1;
	delete[] X2;

	return result;
}


array<array<double, 3>, 2> run_Jacobi2_test(double A00, double A01, double A10, double A11) {
	double **A, **A1, **A2, **X1, **X2, eps;
	int i1, i2, ind, ct, n;
	// データの設定
	ct = 1000;
	eps = 1.0e-10;
	n = 2;
	A = new double *[n];
	A1 = new double *[n];
	A2 = new double *[n];
	X1 = new double *[n];
	X2 = new double *[n];
	for (i1 = 0; i1 < n; i1++) {
		A[i1] = new double[n];
		A1[i1] = new double[n];
		A2[i1] = new double[n];
		X1[i1] = new double[n];
		X2[i1] = new double[n];
	}

	A[0][0] = A00;
	A[0][1] = A01;

	A[1][0] = A10;
	A[1][1] = A11;


	// 計算
	ind = Jacobi(n, ct, eps, A, A1, A2, X1, X2);

	array<array<double, 3>, 2> result;

	int index = 0;
	if (A1[0][0] < A1[1][1]) {
		index = 1;
	}
	result[0][0] = A1[index][index];
	result[0][1] = X1[0][index];
	result[0][2] = X1[1][index];
	index++;
	index %= 2;
	result[1][0] = A1[index][index];
	result[1][1] = X1[0][index];
	result[1][2] = X1[1][index];




	for (i1 = 0; i1 < n; i1++) {
		delete[] A[i1];
		delete[] A1[i1];
		delete[] A2[i1];
		delete[] X1[i1];
		delete[] X2[i1];
	}
	delete[] A;
	delete[] A1;
	delete[] A2;
	delete[] X1;
	delete[] X2;

	return result;
}

array<array<double, 4>, 3> run_Jacobi_test(double Sxx, double Sxy, double Sxz, double Syx, double Syy, double Syz, double Szx, double Szy, double Szz) {
	double **A, **A1, **A2, **X1, **X2, eps;
	int i1, i2, ind, ct, n;
	// データの設定
	ct = 1000;
	eps = 1.0e-10;
	n = 3;
	A = new double *[n];
	A1 = new double *[n];
	A2 = new double *[n];
	X1 = new double *[n];
	X2 = new double *[n];
	for (i1 = 0; i1 < n; i1++) {
		A[i1] = new double[n];
		A1[i1] = new double[n];
		A2[i1] = new double[n];
		X1[i1] = new double[n];
		X2[i1] = new double[n];
	}

	A[0][0] = Sxx;
	A[0][1] = Sxy;
	A[0][2] = Sxz;

	A[1][0] = Syx;
	A[1][1] = Syy;
	A[1][2] = Syz;

	A[2][0] = Szx;
	A[2][1] = Szy;
	A[2][2] = Szz;


	// 計算
	ind = Jacobi(n, ct, eps, A, A1, A2, X1, X2);

	array<array<double, 4>, 3> result;

	int type2 = 0;
	if (A1[0][0] < A1[1][1]) {
		type2 += 1;
	}
	if (A1[1][1] < A1[2][2]) {
		type2 += 2;
	}
	if (A1[2][2] < A1[0][0]) {
		type2 += 4;
	}
	int type3 = 0;
	switch (type2) {
	case 1:
		type3 = 7;
		break;
	case 2:
		type3 = 11;
		break;
	case 3:
		type3 = 5;
		break;
	case 4:
		type3 = 21;
		break;
	case 5:
		type3 = 19;
		break;
	case 6:
		type3 = 15;
		break;
	default:
		break;
	}
	int index = type3 / 9;
	result[0][0] = A1[index][index];
	result[0][1] = X1[0][index];
	result[0][2] = X1[1][index];
	result[0][3] = X1[2][index];
	type3 -= index * 9;
	index = type3 / 3;
	result[1][0] = A1[index][index];
	result[1][1] = X1[0][index];
	result[1][2] = X1[1][index];
	result[1][3] = X1[2][index];
	type3 -= index * 3;
	index = type3;
	result[2][0] = A1[index][index];
	result[2][1] = X1[0][index];
	result[2][2] = X1[1][index];
	result[2][3] = X1[2][index];

	for (i1 = 0; i1 < n; i1++) {
		delete[] A[i1];
		delete[] A1[i1];
		delete[] A2[i1];
		delete[] X1[i1];
		delete[] X2[i1];
	}
	delete[] A;
	delete[] A1;
	delete[] A2;
	delete[] X1;
	delete[] X2;

	return result;
}

#endif // _RunJacobi_HPP_