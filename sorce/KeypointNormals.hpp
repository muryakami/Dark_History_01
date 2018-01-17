#ifndef _KeypointNormal_HPP_
#define _KeypointNormal_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>

// 特徴点の法線生成 (近傍点の重心)
pcl::PointCloud<pcl::PointNormal>::Ptr myKeypoint_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			Eigen::Vector4f xyz_centroid; // 重心を計算
			pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}

// 特徴点の法線生成 (近傍点の重心)
pcl::PointCloud<pcl::PointXYZINormal>::Ptr myKeypoint_normals2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsXYZI) {

	// 点情報
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZINormal>);

	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f; // 3.0f 5.0f 特徴点検出のサポート球と合わせないと余計な点が出る
	kdtree.setInputCloud(cloud);
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypointsXYZI->begin(); it != keypointsXYZI->end(); it++) {
		pcl::PointXYZ searchPoint(it->x, it->y, it->z);
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1) { // 1にすれば孤立点を無視できる？ 0 1
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			Eigen::Vector4f xyz_centroid; // 重心を計算
			pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ average_point(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

			//pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			array<array<double, 3>, 3> point_pca = cal_PCA(neighborhood_cloud);
			pcl::PointXYZINormal keypoint;
			keypoint.x = it->x;
			keypoint.y = it->y;
			keypoint.z = it->z;
			keypoint.intensity = it->intensity;
			keypoint.normal_x = point_pca[0][0];
			keypoint.normal_y = point_pca[0][1];
			keypoint.normal_z = point_pca[0][2];
			pcl::PointXYZ n(point_pca[0][0], point_pca[0][1], point_pca[0][2]);
			pcl::PointXYZ vppi(average_point.x - it->x, average_point.y - it->y, average_point.z - it->z);
			if (dot_product3_cal(n, vppi) > 0) {
				keypoint.normal_x *= (-1);
				keypoint.normal_y *= (-1);
				keypoint.normal_z *= (-1);
			}

			cloud_ptr->push_back(keypoint);
		}
	}

	return cloud_ptr;
}

#endif // _KeypointNormal_HPP_