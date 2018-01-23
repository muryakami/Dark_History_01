#ifndef _InterpolationSTL_HPP_
#define _InterpolationSTL_HPP_

#include "Output.h"
#include "ReadSTL.hpp"
#include "BasicCalculation.hpp"
#include "MT.h"
#include "RandUnitVector.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>


// 三角形の要素内補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_triangle3(TRIANGLE tri) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// 点情報
	pcl::PointXYZ A;
	A.x = tri.vertex[0].x;
	A.y = tri.vertex[0].y;
	A.z = tri.vertex[0].z;
	pcl::PointXYZ B;
	B.x = tri.vertex[1].x;
	B.y = tri.vertex[1].y;
	B.z = tri.vertex[1].z;
	pcl::PointXYZ C;
	C.x = tri.vertex[2].x;
	C.y = tri.vertex[2].y;
	C.z = tri.vertex[2].z;

	// 基底ベクトル
	pcl::PointXYZ eu;
	eu.x = B.x - A.x;
	eu.y = B.y - A.y;
	eu.z = B.z - A.z;
	//float eu_norm = sqrt(dot_product3_cal(eu, eu));

	// 基底ベクトル
	pcl::PointXYZ ev;
	ev.x = C.x - A.x;
	ev.y = C.y - A.y;
	ev.z = C.z - A.z;
	//float ev_norm = sqrt(dot_product3_cal(ev, ev));

	// 三角形の面積（外積の大きさの1/2）
	pcl::PointXYZ cross = cross_product3_cal(eu, ev);
	float cross_norm = sqrt(dot_product3_cal(cross, cross)); //平行四辺形の面積

	float numPoint = cross_norm * 2;
	int countPoint = 0;
	init_genrand((unsigned)time(NULL));
	while (true) {
		if (numPoint < countPoint) break;
		float up = genrand_real1();
		float vp = genrand_real1();
		if (1 < up + vp) continue;
		pcl::PointXYZ p;
		p.x = A.x + up*eu.x + vp*ev.x;
		p.y = A.y + up*eu.y + vp*ev.y;
		p.z = A.z + up*eu.z + vp*ev.z;

		countPoint++;
		//if (p.z < 100) continue; //
		//if (p.z > 67) continue; //
		//if (p.z < 116) continue; //
		cloud_ptr->push_back(p);
	}

	return cloud_ptr;
}


// 三角形の要素内補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_triangle2(TRIANGLE tri) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// 点情報
	pcl::PointXYZ A;
	A.x = tri.vertex[0].x;
	A.y = tri.vertex[0].y;
	A.z = tri.vertex[0].z;
	pcl::PointXYZ B;
	B.x = tri.vertex[1].x;
	B.y = tri.vertex[1].y;
	B.z = tri.vertex[1].z;
	pcl::PointXYZ C;
	C.x = tri.vertex[2].x;
	C.y = tri.vertex[2].y;
	C.z = tri.vertex[2].z;

	// 基底ベクトル
	pcl::PointXYZ eu;
	eu.x = B.x - A.x;
	eu.y = B.y - A.y;
	eu.z = B.z - A.z;
	float eu_norm = sqrt(dot_product3_cal(eu, eu));

	pcl::PointXYZ ev;
	ev.x = C.x - A.x;
	ev.y = C.y - A.y;
	ev.z = C.z - A.z;
	float ev_norm = sqrt(dot_product3_cal(ev, ev));

	for (float up = 0; up <= 1 * eu_norm; up += 1) {
		for (float vp = 0; vp <= (1 - up / eu_norm)*ev_norm; vp += 1) {
			pcl::PointXYZ p;
			p.x = A.x + up*(eu.x / eu_norm) + vp*(ev.x / ev_norm);
			p.y = A.y + up*(eu.y / eu_norm) + vp*(ev.y / ev_norm);
			p.z = A.z + up*(eu.z / eu_norm) + vp*(ev.z / ev_norm);

			//if (p.z < 100) continue; //
			//if (p.z > 67) continue; //
			cloud_ptr->push_back(p);
		}
	}

	return cloud_ptr;
}

// STLデータの内挿補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_stl2(string filename) {

	// 点群情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// データ読み込み
	STLDATA stl_object(filename);

	// 内挿補間
	pcl::PointCloud<pcl::PointXYZ> merged_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr triangle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	merged_cloud = *triangle_cloud;
	for (int i = 0; i<stl_object.getDatanum(); i++) {
		TRIANGLE tri;
		stl_object.getData(i, &tri);
		//merged_cloud += *interpolation_triangle2(tri);
		merged_cloud += *interpolation_triangle3(tri);
	}

	// ダウンサンプリング
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(merged_cloud.makeShared());
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_ptr);

	return cloud_ptr;
}




inline float error() {
	return (rand() % 21 - 10)*0.01 * 3; // 3
	//return (rand() % 21 - 10)*0.01 * 0;
}

// 三角形の要素内補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_triangle(TRIANGLE tri, bool TF) {

	float max_error = sqrt(4.0f); // 最大誤差 0.36
	init_genrand((unsigned)time(NULL));
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// 点情報
	pcl::PointXYZ A;
	A.x = tri.vertex[0].x;
	A.y = tri.vertex[0].y;
	A.z = tri.vertex[0].z;
	pcl::PointXYZ B;
	B.x = tri.vertex[1].x;
	B.y = tri.vertex[1].y;
	B.z = tri.vertex[1].z;
	pcl::PointXYZ C;
	C.x = tri.vertex[2].x;
	C.y = tri.vertex[2].y;
	C.z = tri.vertex[2].z;

	// 基底ベクトル
	pcl::PointXYZ eu;
	eu.x = B.x - A.x;
	eu.y = B.y - A.y;
	eu.z = B.z - A.z;
	float eu_norm = sqrt(dot_product3_cal(eu, eu));

	pcl::PointXYZ ev;
	ev.x = C.x - A.x;
	ev.y = C.y - A.y;
	ev.z = C.z - A.z;
	float ev_norm = sqrt(dot_product3_cal(ev, ev));

	/*for (float up = 0; up <= 1; up += 0.05) {
	for (float vp = 0; vp <= (1 - up); vp += 0.05) {
	pcl::PointXYZ p;
	p.x = A.x + up*eu.x + vp*ev.x;
	p.y = A.y + up*eu.y + vp*ev.y;
	p.z = A.z + up*eu.z + vp*ev.z;
	cloud_ptr->push_back(p);
	}
	}*/

	for (float up = 0; up <= 1 * eu_norm; up += 1) {
		for (float vp = 0; vp <= (1 - up / eu_norm)*ev_norm; vp += 1) {
			pcl::PointXYZ p;
			p.x = A.x + up*(eu.x / eu_norm) + vp*(ev.x / ev_norm);
			p.y = A.y + up*(eu.y / eu_norm) + vp*(ev.y / ev_norm);
			p.z = A.z + up*(eu.z / eu_norm) + vp*(ev.z / ev_norm);
			if (TF) {
				//p.x += error();
				//p.y += error();
				//p.z += error();

				float error = max_error * genrand_real1();
				randUnitVector(sampleTest, sampleTime);
				sampleTest->x *= error;
				sampleTest->y *= error;
				sampleTest->z *= error;
				p.x += sampleTest->x;
				p.y += sampleTest->y;
				p.z += sampleTest->z;
			}

			//if (p.z < 100) continue; //
			//if (p.z > 67) continue; //

			cloud_ptr->push_back(p);
		}
	}

	return cloud_ptr;
}

// STLデータの内挿補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_stl(string filename, bool TF) {

	// 点群情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// データ読み込み
	STLDATA stl_object(filename);

	// 内挿補間
	pcl::PointCloud<pcl::PointXYZ> merged_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr triangle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	merged_cloud = *triangle_cloud;
	for (int i = 0; i<stl_object.getDatanum(); i++) {
		TRIANGLE tri;
		stl_object.getData(i, &tri);
		merged_cloud += *interpolation_triangle(tri, TF);
	}

	// ダウンサンプリング
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(merged_cloud.makeShared());
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_ptr);

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr make_occlusion(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // 視点

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		// 平面の法線
		pcl::PointXYZ n(it->normal_x, it->normal_y, it->normal_z);
		// 法線から視点へのベクトル
		pcl::PointXYZ vppi(view_point.x - it->x, view_point.y - it->y, view_point.z - it->z);
		if (dot_product3_cal(n, vppi) < 0) continue; // 視点から見て内側に法線を持つ点（視点から見える部分）
		//if (dot_product3_cal(n, vppi) > 0) continue; // 視点から見て外側に法線を持つ点（視点から見えない部分）
		cloud_normals->push_back(*it);
	}

	return cloud_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr make_occlusion(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointXYZ view_point, bool TF)
{
	//pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // 視点

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		// 平面の法線
		pcl::PointXYZ n(it->normal_x, it->normal_y, it->normal_z);
		// 法線から視点へのベクトル
		pcl::PointXYZ vppi(view_point.x - it->x, view_point.y - it->y, view_point.z - it->z);
		if (TF) {
			if (dot_product3_cal(n, vppi) < 0) continue; // 視点から見て内側に法線を持つ点（視点から見える部分）
		} else {
			if (dot_product3_cal(n, vppi) > 0) continue; // 視点から見て外側に法線を持つ点（視点から見えない部分）
		}
		cloud_normals->push_back(*it);
	}

	return cloud_normals;
}


#endif // _InterpolationSTL_HPP_