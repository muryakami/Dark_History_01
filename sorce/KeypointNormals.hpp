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

#endif // _KeypointNormal_HPP_