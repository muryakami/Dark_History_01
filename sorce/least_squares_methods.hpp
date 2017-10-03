#ifndef _leastSquaresMethods_HPP_
#define _leastSquaresMethods_HPP_

#include "gaussianElimination.hpp"
#include <array>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

array<float, 4> least_squares_method_sphere(vector<array<float, 3>> matrix) {
	array<float, 4> coefficients;
	float A[N_DIM][N_DIM] = { 0 };
	float B[N_DIM] = { 0 };
	float C[N_DIM] = { 0 };
	for (array<float, 3> p : matrix) {
		float x = p[0];
		float y = p[1];
		float z = p[2];
		// 左辺(係数行列)
		A[0][0] += 2.0 * x * x;
		A[0][1] += 2.0 * x * y;
		A[0][2] += 2.0 * x * z;
		A[0][3] += 2.0 * x;

		A[1][0] += 2.0 * y * x;
		A[1][1] += 2.0 * y * y;
		A[1][2] += 2.0 * y * z;
		A[1][3] += 2.0 * y;

		A[2][0] += 2.0 * z * x;
		A[2][1] += 2.0 * z * y;
		A[2][2] += 2.0 * z * z;
		A[2][3] += 2.0 * z;

		A[3][0] += 2.0 * x;
		A[3][1] += 2.0 * y;
		A[3][2] += 2.0 * z;
		A[3][3] += 2.0 * 1.0;

		// 右辺
		B[0] += x * (x * x + y * y + z * z);
		B[1] += y * (x * x + y * y + z * z);
		B[2] += z * (x * x + y * y + z * z);
		B[3] += x * x + y * y + z * z;
	}
	solve(A, B, 4);	// 連立方程式を計算

	// a,b,c,dのパラメータがB[0],B[1],B[2],B[3]に代入されているので
	// a,b,c,rを代入して関数を終わる
	coefficients[0] = B[0];
	coefficients[1] = B[1];
	coefficients[2] = B[2];
	coefficients[3] = sqrt(B[0] * B[0] + B[1] * B[1] + B[2] * B[2] + 2 * B[3]);

	return coefficients;
}

array<float, 3> least_squares_method_cylinder2(vector<array<float, 3>> matrix) {
	array<float, 3> coefficients;
	float A[N_DIM][N_DIM] = { 0 };
	float B[N_DIM] = { 0 };
	for (array<float, 3> p : matrix) {
		float x = p[0];
		float y = p[1];

		// 左辺(係数行列)
		A[0][0] += 2.0 * x * x;
		A[0][1] += 2.0 * x * y;
		A[0][2] += 2.0 * x;

		A[1][0] += 2.0 * y * x;
		A[1][1] += 2.0 * y * y;
		A[1][2] += 2.0 * y;

		A[2][0] += 2.0 * x;
		A[2][1] += 2.0 * y;
		A[2][2] += 2.0 * 1.0;

		// 右辺
		B[0] += x * (x * x + y * y);
		B[1] += y * (x * x + y * y);
		B[2] += x * x + y * y;
	}

	solve(A, B, 3);	// 連立方程式を計算

	// a,b,cのパラメータがB[0],B[1],B[2]に代入されているので
	// a,b,rを代入して関数を終わる
	coefficients[0] = B[0];
	coefficients[1] = B[1];
	coefficients[2] = sqrt(B[0] * B[0] + B[1] * B[1] + 2 * B[2]);

	return coefficients;
}

array<float, 3> least_squares_method_cylinder0(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	array<float, 3> coefficients;
	float A[N_DIM][N_DIM] = { 0 };
	float B[N_DIM] = { 0 };

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		float x = it->x;
		float y = it->z;

		// 左辺(係数行列)
		A[0][0] += 2.0 * x * x;
		A[0][1] += 2.0 * x * y;
		A[0][2] += 2.0 * x;

		A[1][0] += 2.0 * y * x;
		A[1][1] += 2.0 * y * y;
		A[1][2] += 2.0 * y;

		A[2][0] += 2.0 * x;
		A[2][1] += 2.0 * y;
		A[2][2] += 2.0 * 1.0;

		// 右辺
		B[0] += x * (x * x + y * y);
		B[1] += y * (x * x + y * y);
		B[2] += x * x + y * y;
	}

	solve(A, B, 3);	// 連立方程式を計算

	// a,b,cのパラメータがB[0],B[1],B[2]に代入されているので
	// a,b,rを代入して関数を終わる
	coefficients[0] = B[0];
	coefficients[1] = B[1];
	coefficients[2] = sqrt(B[0] * B[0] + B[1] * B[1] + 2 * B[2]);

	return coefficients;
}

#endif