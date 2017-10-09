#ifndef _Errata_HPP_
#define _Errata_HPP_

#include "BasicCalculation.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>

using namespace std;

typedef struct _myQuaternion {
	float angle;
	pcl::PointXYZ axis;
	bool operator < (const _myQuaternion& rhs) const {
		// angle��r
		if (angle < rhs.angle) return true;
		if (angle > rhs.angle) return false;
		// axis_x��r
		if (axis.x < rhs.axis.x) return true;
		if (axis.x > rhs.axis.x) return false;
		// axis_y��r
		if (axis.y < rhs.axis.y) return true;
		if (axis.y > rhs.axis.y) return false;
		// axis_z��r
		if (axis.z < rhs.axis.z) return true;
		if (axis.z > rhs.axis.z) return false;
		return false;
	}
} myQuaternion;
int errata2(pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr cloud) {
	map<myQuaternion, int> rotationHistogram;
	for (pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		pcl::PointXYZ A1 = it->first.first;
		pcl::PointXYZ B1 = it->first.second;
		pcl::PointXYZ A2 = it->second.first;
		pcl::PointXYZ B2 = it->second.second;
		float p_dis = sqrt(euclidean_distance(A1, B1));
		float q_dis = sqrt(euclidean_distance(A2, B2));
		pcl::PointXYZ p((B1.x - A1.x) / p_dis, (B1.y - A1.y) / p_dis, (B1.z - A1.z) / p_dis);
		pcl::PointXYZ q((B2.x - A2.x) / q_dis, (B2.y - A2.y) / q_dis, (B2.z - A2.z) / q_dis);

		myQuaternion quaternion;
		quaternion.angle = dot_product3_cal(p, q);
		quaternion.axis = cross_product3_cal(p, q);

		//quaternion.angle = round01(quaternion.angle);
		quaternion.angle = acos(quaternion.angle);			// radian
		quaternion.angle = quaternion.angle * 180 / M_PI;	// �x
		quaternion.angle = round10(quaternion.angle);		// �ʎq��

		pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
		float norm = sqrt(euclidean_distance(quaternion.axis, origin));
		quaternion.axis.x /= norm;	// ���K��
		quaternion.axis.y /= norm;
		quaternion.axis.z /= norm;
		quaternion.axis.x = round01(quaternion.axis.x);	// �ʎq��
		quaternion.axis.y = round01(quaternion.axis.y);
		quaternion.axis.z = round01(quaternion.axis.z);

		auto itr = rotationHistogram.find(quaternion);	// quaternion ���ݒ肳��Ă��邩�H
		if (itr != rotationHistogram.end()) {
			//�ݒ肳��Ă���ꍇ�̏���
			itr->second++;
		}
		else {
			//�ݒ肳��Ă��Ȃ��ꍇ�̏���
			rotationHistogram.emplace(quaternion, 1);
		}
	}

	int max_i = -1;
	myQuaternion max_quaternion; // ��]���Ɖ�]�p�x�@�H��p�����킩��
	for (map<myQuaternion, int>::iterator it = rotationHistogram.begin(); it != rotationHistogram.end(); it++) {
		if (max_i < it->second) {
			max_i = it->second;
			max_quaternion = it->first;
		}
	}

	return max_i;
}

int errata3(pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr cloud) {
	map<float, int> rotationHistogram;
	for (pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		pcl::PointXYZ A1 = it->first.first;
		pcl::PointXYZ B1 = it->first.second;
		pcl::PointXYZ A2 = it->second.first;
		pcl::PointXYZ B2 = it->second.second;
		float p_dis = sqrt(euclidean_distance(A1, B1));
		float q_dis = sqrt(euclidean_distance(A2, B2));
		pcl::PointXYZ p((B1.x - A1.x) / p_dis, (B1.y - A1.y) / p_dis, (B1.z - A1.z) / p_dis);
		pcl::PointXYZ q((B2.x - A2.x) / q_dis, (B2.y - A2.y) / q_dis, (B2.z - A2.z) / q_dis);

		float angle = dot_product3_cal(p, q);

		//quaternion.angle = round01(quaternion.angle);
		angle = acos(angle);			// radian
		angle = angle * 180 / M_PI;	// �x
		angle = round10(angle);		// �ʎq��

		auto itr = rotationHistogram.find(angle);	// angle ���ݒ肳��Ă��邩�H
		if (itr != rotationHistogram.end()) {
			//�ݒ肳��Ă���ꍇ�̏���
			itr->second++;
		}
		else {
			//�ݒ肳��Ă��Ȃ��ꍇ�̏���
			rotationHistogram.emplace(angle, 1);
		}
	}

	int max_i = -1;
	float max_angle; // ��]���Ɖ�]�p�x�@�H��p�����킩��
	for (map<float, int>::iterator it = rotationHistogram.begin(); it != rotationHistogram.end(); it++) {
		if (max_i < it->second) {
			max_i = it->second;
			max_angle = it->first;
		}
	}

	return max_i;
}

#endif // _Errata_HPP_