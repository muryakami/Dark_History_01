#ifndef _NormalDirection_HPP_
#define _NormalDirection_HPP_

#include "CalPCA.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

pcl::PointNormal normal_direction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point, pcl::PointXYZ average_point) {
	array<array<double, 3>, 3> point_pca = cal_PCA(cloud);
	pcl::PointNormal normal;
	normal.x = attention_point.x;
	normal.y = attention_point.y;
	normal.z = attention_point.z;
	normal.normal_x = point_pca[0][0];
	normal.normal_y = point_pca[0][1];
	normal.normal_z = point_pca[0][2];

	pcl::PointXYZ n(point_pca[0][0], point_pca[0][1], point_pca[0][2]);
	pcl::PointXYZ vppi(average_point.x - attention_point.x, average_point.y - attention_point.y, average_point.z - attention_point.z);
	if (dot_product3_cal(n, vppi) > 0) {
		normal.normal_x *= (-1);
		normal.normal_y *= (-1);
		normal.normal_z *= (-1);
	}

	return normal;
}

#endif // _NormalDirection_HPP_