#ifndef _PreprocessingSTL_HPP_
#define _PreprocessingSTL_HPP_

#include "ReadSTL.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>

pcl::PointCloud<pcl::PointNormal>::Ptr stl_points(STLDATA stl) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;
	for (int i = 0; i<stl.getDatanum(); i++) {
		TRIANGLE tri;
		stl.getData(i, &tri);
		for (int j = 0; j < 3; j++) {
			point.normal_x = tri.normal.nx;
			point.normal_y = tri.normal.ny;
			point.normal_z = tri.normal.nz;
			point.x = tri.vertex[j].x;
			point.y = tri.vertex[j].y;
			point.z = tri.vertex[j].z;
			cloud_ptr->points.push_back(point);
		}
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//点のset情報
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointNormal>);
	while (!cloud_ptr->empty()) {
		int count = 0;

		pcl::PointNormal keypoint;
		keypoint.normal_x = keypoint.normal_y = keypoint.normal_z = 0.0f;
		keypoint.x = cloud_ptr->at(0).x;
		keypoint.y = cloud_ptr->at(0).y;
		keypoint.z = cloud_ptr->at(0).z;

		auto itr = cloud_ptr->begin();
		while (itr != cloud_ptr->end()) {
			bool equalPointXYZ = keypoint.x == itr->x && keypoint.y == itr->y && keypoint.z == itr->z;
			if (equalPointXYZ) // 削除条件に合う要素が見つかった
			{
				keypoint.normal_x += itr->normal_x;
				keypoint.normal_y += itr->normal_y;
				keypoint.normal_z += itr->normal_z;
				count++;
				itr = cloud_ptr->erase(itr); // erase()の戻り値をitで受ける
			}
			else
			{
				itr++;
			}
		}
		keypoint.normal_x /= count;
		keypoint.normal_y /= count;
		keypoint.normal_z /= count;

		//keypoint.x += 30.0f;
		//keypoint.y += 30.0f;
		//keypoint.z += -40.0f;

		keypoints_ptr->push_back(keypoint);
	}

	keypoints_ptr->width = (int)keypoints_ptr->points.size();
	keypoints_ptr->height = 1;

	return keypoints_ptr;
}



pcl::PointCloud<pcl::PointNormal>::Ptr stl_points0(STLDATA stl) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;
	for (int i = 0; i<stl.getDatanum(); i++) {
		TRIANGLE tri;
		stl.getData(i, &tri);
		for (int j = 0; j < 3; j++) {
			point.normal_x = tri.normal.nx;
			point.normal_y = tri.normal.ny;
			point.normal_z = tri.normal.nz;
			point.x = tri.vertex[j].x;
			point.y = tri.vertex[j].y;
			point.z = tri.vertex[j].z;
			cloud_ptr->push_back(point);
		}
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr stl_nAveraging(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 0.01f;

	kdtree.setInputCloud(cloud_in);

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
		pcl::PointNormal searchPoint = *it;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			float ave_nx = 0.0f;
			float ave_ny = 0.0f;
			float ave_nz = 0.0f;
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				ave_nx += cloud_in->points[pointIdxRadiusSearch[i]].normal_x;
				ave_ny += cloud_in->points[pointIdxRadiusSearch[i]].normal_y;
				ave_nz += cloud_in->points[pointIdxRadiusSearch[i]].normal_z;
			}
			float norm = sqrt(ave_nx*ave_nx + ave_ny*ave_ny + ave_nz*ave_nz);

			point.x = it->x;
			point.y = it->y;
			point.z = it->z;
			point.normal_x = ave_nx / norm;
			point.normal_y = ave_ny / norm;
			point.normal_z = ave_nz / norm;

			cloud_ptr->points.push_back(point);
		}
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr stl_decimating(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 0.01f;

	kdtree.setInputCloud(cloud_in);

	while (!cloud_in->empty()) {
		kdtree.setInputCloud(cloud_in);
		pcl::PointNormal searchPoint = cloud_in->at(0);
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
			for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
				cloud_in->erase(cloud_in->begin() + pointIdxRadiusSearch[i]);
		}
		cout << "cloud_in->size(): " << cloud_in->size() << endl;
		cloud_ptr->push_back(searchPoint);
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr stl_decimating0(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;

	while (!cloud_in->empty()) {
		point = cloud_in->at(0);
		auto itr = cloud_in->begin();
		while (itr != cloud_in->end()) {
			bool equalPointXYZ = point.x == itr->x && point.y == itr->y && point.z == itr->z;
			if (equalPointXYZ) // 削除条件に合う要素が見つかった
			{
				itr = cloud_in->erase(itr); // erase()の戻り値をitで受ける
			}
			else
			{
				itr++;
			}
		}
		//keypoint.x += 30.0f;
		//keypoint.y += 30.0f;
		//keypoint.z += -40.0f;
		cout << "cloud_in->size(): " << cloud_in->size() << endl;
		cloud_ptr->push_back(point);
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr stl_pointsXYZ(STLDATA stl) {

	//点情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
	for (int i = 0; i<stl.getDatanum(); i++) {
		TRIANGLE tri;
		stl.getData(i, &tri);
		for (int j = 0; j < 3; j++) {
			point.x = tri.vertex[j].x;
			point.y = tri.vertex[j].y;
			point.z = tri.vertex[j].z;
			/////// 工具先端のみ ///////
			// ドリル
			//if (point.y > -6) continue;	// 外れ値の除去をすればこれでOK
			//if (point.y > -7) continue;	// 外れ値の除去なし
			//if (point.y > -20) continue;
			//if (point.y < -21) continue;
			// バイト
			//if (point.y > -35) continue;	// 奥行きのみ
			if (point.y > -27) continue;
			if (point.z < 65) continue;
			////////////////////////////
			cloud_ptr->push_back(point);
		}
	}
	cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

#endif // _PreprocessingSTL_HPP_