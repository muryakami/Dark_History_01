#ifndef _Projection_HPP_
#define _Projection_HPP_

#include <pcl/point_types.h>

// �ˉe�v�Z
pcl::PointXYZ Projection1(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// ���ˉe�̂��߂̏���
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

	// ���ˉe
	rotatePoint.x = rotate_matrix[0][0] * originPoint.x + rotate_matrix[0][1] * originPoint.y + rotate_matrix[0][2] * originPoint.z;
	rotatePoint.y = rotate_matrix[1][0] * originPoint.x + rotate_matrix[1][1] * originPoint.y + rotate_matrix[1][2] * originPoint.z;
	rotatePoint.z = rotate_matrix[2][0] * originPoint.x + rotate_matrix[2][1] * originPoint.y + rotate_matrix[2][2] * originPoint.z;

	return rotatePoint;
}


// �ˉe�v�Z
pcl::PointXYZ Projection2(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// ���ˉe�̂��߂̏���
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

// �ˉe�v�Z
pcl::PointXYZ Projection3(pcl::PointXYZ originPoint, pcl::PointXYZ axis) {

	pcl::PointXYZ rotatePoint;

	// ���ˉe�̂��߂̏���
	float cosR = dot_product3_cal(originPoint, axis);
	float norm = dot_product3_cal(originPoint, originPoint);
	float coefficient = cosR / norm;

	rotatePoint.x = coefficient * axis.x;
	rotatePoint.y = coefficient * axis.y;
	rotatePoint.z = coefficient * axis.z;

	return rotatePoint;
}

// �ˉe�v�Z
pcl::PointXYZ Projection4(pcl::PointXYZ point, pcl::PointNormal plane) {
	// point�F�ˉe�������_
	// plane�F���ʁi���ʏ�̓_�C�@���x�N�g���ō\���j

	// �@���x�N�g���𐳋K��
	pcl::PointXYZ planeNormal(plane.normal_x, plane.normal_y, plane.normal_z);
	float norm = sqrt(dot_product3_cal(planeNormal, planeNormal));
	planeNormal.x /= norm;
	planeNormal.y /= norm;
	planeNormal.z /= norm;

	// �ˉe�������_���畽�ʏ�̓_�ւ̈ړ��x�N�g��
	pcl::PointXYZ moveVector(plane.x - point.x, plane.y - point.y, plane.z - point.z);

	// �ړ��x�N�g���̖@�������������v�Z(�_���畽�ʂ܂ł̋���)
	float d = dot_product3_cal(moveVector, planeNormal);
	pcl::PointXYZ distanceVector(d*planeNormal.x, d*planeNormal.y, d*planeNormal.z);

	// �ˉe�������_�𕽖ʂɈړ�
	pcl::PointXYZ projectionPoint(point.x+distanceVector.x, point.y + distanceVector.y, point.z + distanceVector.z);

	return projectionPoint;
}

#endif // _Projection_HPP_