#ifndef _CalPCA_HPP_
#define _CalPCA_HPP_

#include "RunJacobi.hpp"
#include "Principal.hpp"
#include "BasicCalculation.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void testPCA(int p, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	double **x, *r, **a;
	//int i1, i2, n, p, sw;

	//scanf("%d %d", &p, &n);   // 変数の数とデータの数
	int i1, i2, sw;
	int n = cloud->size();


	r = new double[p];
	x = new double *[p];
	a = new double *[p];
	for (i1 = 0; i1 < p; i1++) {
		x[i1] = new double[n];
		a[i1] = new double[p];
	}

	//for (i1 = 0; i1 < n; i1++) {   // データ
	//for (i2 = 0; i2 < p; i2++)
	//scanf("%lf", &x[i2][i1]);
	//}

	// とりあえず3次元 2次元も考える
	for (i1 = 0; i1 < n; i1++) {   // データ
		for (i2 = 0; i2 < p; i2++)
			x[i2][i1] = cloud->at(i1).data[i2];
	}


	sw = principal(p, n, x, r, a, 1.0e-10, 200);

	if (sw == 0) {
		for (i1 = 0; i1 < p; i1++) {
			cout << "主成分： " << r[i1];
			cout << " 係数： ";
			for (i2 = 0; i2 < p; i2++)
				cout << a[i1][i2];
			cout << endl;
		}
	}
	else
		cout << "***error  解を求めることができませんでした" << endl;

	for (i1 = 0; i1 < p; i1++) {
		delete[] x[i1];
		delete[] a[i1];
	}
	delete[] x;
	delete[] a;
	delete[] r;
}

array<array<double, 3>, 3> cal_PCA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointXYZ average = cal_cloud_average(cloud);
	vector<array<float, 3>> deviation;
	deviation.reserve(cloud->size());
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		array<float, 3> point;
		point[0] = i->x - average.x;
		point[1] = i->y - average.y;
		point[2] = i->z - average.z;
		deviation.push_back(point);
	}

	double Sxx = 0;
	double Sxy = 0;
	double Sxz = 0;
	double Syy = 0;
	double Syz = 0;
	double Szz = 0;
	for (int i = 0; i < deviation.size(); i++) {
		Sxx += deviation[i][0] * deviation[i][0];
		Sxy += deviation[i][0] * deviation[i][1];
		Sxz += deviation[i][0] * deviation[i][2];
		Syy += deviation[i][1] * deviation[i][1];
		Syz += deviation[i][1] * deviation[i][2];
		Szz += deviation[i][2] * deviation[i][2];
	}

	array<array<double, 4>, 3> eigenvalue = run_Jacobi(Sxx, Sxy, Sxz, Syy, Syz, Szz);

	array<array<double, 3>, 3> eigenvector;
	for (int j = 0; j < 3; j++) {
		eigenvector[j][0] = eigenvalue[j][1];
		eigenvector[j][1] = eigenvalue[j][2];
		eigenvector[j][2] = eigenvalue[j][3];
	}

	//cout << "eigenvalue[0][0]: " << eigenvalue[0][0] << endl;
	//cout << "eigenvalue[1][0]: " << eigenvalue[1][0] << endl;
	//cout << "eigenvalue[2][0]: " << eigenvalue[2][0] << endl;

	rpc3d << "固有値：" << eigenvalue[0][0] << endl;
	rpc3d << "固有ベクトル：" << eigenvalue[0][1] << "\t" << eigenvalue[0][2] << "\t" << eigenvalue[0][3] << endl;
	rpc3d << "固有値：" << eigenvalue[1][0] << endl;
	rpc3d << "固有ベクトル：" << eigenvalue[1][1] << "\t" << eigenvalue[1][2] << "\t" << eigenvalue[1][3] << endl;
	rpc3d << "固有値：" << eigenvalue[2][0] << endl;
	rpc3d << "固有ベクトル：" << eigenvalue[2][1] << "\t" << eigenvalue[2][2] << "\t" << eigenvalue[2][3] << endl;
	rpc3d << endl;

	return eigenvector;
}


array<array<double, 2>, 2> cal_PCA2(vector<array<float, 2>> cloud) {
	//array<array<double, 2>, 2> cal_PCA2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	array<float, 2> average;
	average[0] = 0.0f;
	average[1] = 0.0f;
	for (vector<array<float, 2>>::iterator it = cloud.begin(); it != cloud.end(); it++) {
		average[0] += it->at(0);
		average[1] += it->at(1);
	}
	average[0] /= cloud.size();
	average[1] /= cloud.size();

	vector<array<float, 2>> deviation;
	deviation.reserve(cloud.size());
	for (vector<array<float, 2>>::iterator i = cloud.begin(); i != cloud.end(); i++) {
		array<float, 2> point;
		point[0] = i->at(0) - average[0];
		point[1] = i->at(1) - average[1];
		deviation.push_back(point);
	}

	//pcl::PointXYZ average = cal_cloud_average(cloud);
	//vector<array<float, 2>> deviation;
	//deviation.reserve(cloud->size());
	//for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) {
	//	array<float, 2> point;
	//	point[0] = i->x - average.x;
	//	point[1] = i->y - average.y;
	//	deviation.push_back(point);
	//}

	double Sxx = 0;
	double Sxy = 0;
	double Syy = 0;
	for (int i = 0; i < deviation.size(); i++) {
		Sxx += deviation[i][0] * deviation[i][0];
		Sxy += deviation[i][0] * deviation[i][1];
		Syy += deviation[i][1] * deviation[i][1];
	}

	array<array<double, 3>, 2> eigenvalue = run_Jacobi2(Sxx, Sxy, Syy);

	array<array<double, 2>, 2> eigenvector;
	for (int j = 0; j < 2; j++) {
		eigenvector[j][0] = eigenvalue[j][1];
		eigenvector[j][1] = eigenvalue[j][2];
	}

	//cout << "eigenvalue[0][0]: " << eigenvalue[0][0] << endl;
	//cout << "eigenvalue[1][0]: " << eigenvalue[1][0] << endl;

	rpc2d << "固有値：" << eigenvalue[0][0] << endl;
	rpc2d << "固有ベクトル：" << eigenvalue[0][1] << "\t" << eigenvalue[0][2] << endl;
	rpc2d << "固有値：" << eigenvalue[1][0] << endl;
	rpc2d << "固有ベクトル：" << eigenvalue[1][1] << "\t" << eigenvalue[1][2] << endl;
	rpc2d << endl;
	rpc2d << endl;
	rpc2d << endl;

	return eigenvector;
}

#endif // _CalPCA_HPP_