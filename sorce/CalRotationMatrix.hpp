#ifndef _CalRotationMatrix_HPP_
#define _CalRotationMatrix_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "BasicCalculation.hpp"

// 回転マトリクスの計算 (ロドリゲスの公式)
Eigen::Matrix4d calRotationMatrix(pcl::PointXYZ before, pcl::PointXYZ after) {

	// 回転軸と回転角の計算
	pcl::PointXYZ axisR = cross_product3_cal(before, after);
	float cosR = dot_product3_cal(before, after);
	float sinR = sqrt(1.0 - cosR*cosR);

	// 回転マトリクス
	Eigen::Matrix4d rotate_matrix;
	rotate_matrix(0, 0) = (1 - cosR)*axisR.x*axisR.x + cosR;
	rotate_matrix(0, 1) = (1 - cosR)*axisR.x*axisR.y - sinR*axisR.z;
	rotate_matrix(0, 2) = (1 - cosR)*axisR.z*axisR.x + sinR*axisR.y;
	rotate_matrix(0, 3) = 0.0f;	// 平行移動 x

	rotate_matrix(1, 0) = (1 - cosR)*axisR.x*axisR.y + sinR*axisR.z;
	rotate_matrix(1, 1) = (1 - cosR)*axisR.y*axisR.y + cosR;
	rotate_matrix(1, 2) = (1 - cosR)*axisR.y*axisR.z - sinR*axisR.x;
	rotate_matrix(1, 3) = 0.0f;	// 平行移動 y

	rotate_matrix(2, 0) = (1 - cosR)*axisR.z*axisR.x - sinR*axisR.y;
	rotate_matrix(2, 1) = (1 - cosR)*axisR.y*axisR.z + sinR*axisR.x;
	rotate_matrix(2, 2) = (1 - cosR)*axisR.z*axisR.z + cosR;
	rotate_matrix(2, 3) = 0.0f;	// 平行移動 z

	// アフィン変換のためのダミー次元
	rotate_matrix(3, 0) = 0.0f;
	rotate_matrix(3, 1) = 0.0f;
	rotate_matrix(3, 2) = 0.0f;
	rotate_matrix(3, 3) = 1.0f;

	return rotate_matrix;
}


#endif // _CalRotationMatrix_HPP_