#ifndef _BasicCalculation_HPP_
#define _BasicCalculation_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

inline float dot_product3_cal(pcl::PointXYZ p, pcl::PointXYZ q) {
	return (p.x * q.x) + (p.y * q.y) + (p.z * q.z);
}

inline pcl::PointXYZ cross_product3_cal(pcl::PointXYZ p, pcl::PointXYZ q) {
	return pcl::PointXYZ((p.y * q.z) - (p.z * q.y), (p.z * q.x) - (p.x * q.z), (p.x * q.y) - (p.y * q.x));
}

inline float euclidean_distance(pcl::PointXYZ p, pcl::PointXYZ q) {
	return ((p.x - q.x)*(p.x - q.x) + (p.y - q.y)*(p.y - q.y) + (p.z - q.z)*(p.z - q.z));
}

inline double cal_angle(pcl::PointXYZ p, pcl::PointXYZ q) {
	float pq_dot = dot_product3_cal(p, q);
	float p_norm = sqrt(dot_product3_cal(p, p));
	float q_norm = sqrt(dot_product3_cal(q, q));
	double angle_cos = pq_dot / (p_norm*q_norm);
	if (angle_cos > 1)
		angle_cos = 1;
	if (angle_cos < -1)
		angle_cos = -1;
	double radian = acos(angle_cos);
	return radian * 180 / M_PI;
}

inline pcl::PointXYZ cal_cloud_average(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::PointXYZ average(0.0f, 0.0f, 0.0f);
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		average.x += i->x;
		average.y += i->y;
		average.z += i->z;
	}
	average.x /= cloud->size();
	average.y /= cloud->size();
	average.z /= cloud->size();
	return average;
}


// �l�̌ܓ��i10��n��̈ʂ������j
inline double roundN(double src, int n) {
	double dst;
	dst = src * pow(10, -n - 1);          /*�������s������10-1 �̈ʂɂ���*/
	dst = (double)(int)(dst + 0.5);
	return    dst * pow(10, n + 1);       /*�������s�����������ɖ߂�*/
}

#endif