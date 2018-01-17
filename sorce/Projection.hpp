#ifndef _Projection_HPP_
#define _Projection_HPP_

#include <pcl/point_types.h>

// 射影計算
pcl::PointXYZ Projection1(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// 正射影のための準備
	pcl::PointXYZ axisR = cross_product3_cal(originPoint, axis);
	float cosR = dot_product3_cal(originPoint, axis);
	float sinR = sqrt(1.0 - cosR*cosR);

	array<array<float, 3>, 3> rotate_matrix;
	rotate_matrix[0][0] = (1 - cosR)*axisR.x*axisR.x + cosR;
	rotate_matrix[0][1] = (1 - cosR)*axisR.x*axisR.y - sinR*axisR.z;
	rotate_matrix[0][2] = (1 - cosR)*axisR.z*axisR.x + sinR*axisR.y;
	rotate_matrix[1][0] = (1 - cosR)*axisR.x*axisR.y + sinR*axisR.z;
	rotate_matrix[1][1] = (1 - cosR)*axisR.y*axisR.y + cosR;
	rotate_matrix[1][2] = (1 - cosR)*axisR.y*axisR.z - sinR*axisR.x;
	rotate_matrix[2][0] = (1 - cosR)*axisR.z*axisR.x - sinR*axisR.y;
	rotate_matrix[2][1] = (1 - cosR)*axisR.y*axisR.z + sinR*axisR.x;
	rotate_matrix[2][2] = (1 - cosR)*axisR.z*axisR.z + cosR;

	// 正射影
	rotatePoint.x = rotate_matrix[0][0] * originPoint.x + rotate_matrix[0][1] * originPoint.y + rotate_matrix[0][2] * originPoint.z;
	rotatePoint.y = rotate_matrix[1][0] * originPoint.x + rotate_matrix[1][1] * originPoint.y + rotate_matrix[1][2] * originPoint.z;
	rotatePoint.z = rotate_matrix[2][0] * originPoint.x + rotate_matrix[2][1] * originPoint.y + rotate_matrix[2][2] * originPoint.z;

	return rotatePoint;
}


// 射影計算
pcl::PointXYZ Projection2(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// 正射影のための準備
	float cosR = dot_product3_cal(originPoint, axis);
	float norm = dot_product3_cal(originPoint, originPoint);
	float coefficient = cosR / norm;

	axis.x *= coefficient;
	axis.y *= coefficient;
	axis.z *= coefficient;

	rotatePoint.x = originPoint.x - axis.x;
	rotatePoint.y = originPoint.y - axis.y;
	rotatePoint.z = originPoint.z - axis.z;

	return rotatePoint;
}

// 射影計算
pcl::PointXYZ Projection3(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// 正射影のための準備
	float cosR = dot_product3_cal(originPoint, axis);
	float norm = dot_product3_cal(originPoint, originPoint);
	float coefficient = cosR / norm;

	rotatePoint.x = coefficient * axis.x;
	rotatePoint.y = coefficient * axis.y;
	rotatePoint.z = coefficient * axis.z;

	return rotatePoint;
}

// 射影計算
pcl::PointXYZ Projection4(pcl::PointXYZ point, pcl::PointNormal plane) {
	// point：射影したい点
	// plane：平面（平面上の点，法線ベクトルで構成）

	// 法線ベクトルを正規化
	pcl::PointXYZ planeNormal(plane.normal_x, plane.normal_y, plane.normal_z);
	float norm = sqrt(dot_product3_cal(planeNormal, planeNormal));
	planeNormal.x /= norm;
	planeNormal.y /= norm;
	planeNormal.z /= norm;

	// 射影したい点から平面上の点への移動ベクトル
	pcl::PointXYZ moveVector(plane.x - point.x, plane.y - point.y, plane.z - point.z);

	// 移動ベクトルの法線方向成分を計算(点から平面までの距離)
	float d = dot_product3_cal(moveVector, planeNormal);
	pcl::PointXYZ distanceVector(d*planeNormal.x, d*planeNormal.y, d*planeNormal.z);

	// 射影したい点を平面に移動
	pcl::PointXYZ projectionPoint(point.x+distanceVector.x, point.y + distanceVector.y, point.z + distanceVector.z);

	return projectionPoint;
}

#endif // _Projection_HPP_