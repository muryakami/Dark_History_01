#ifndef _PPF_HPP_
#define _PPF_HPP_

#include "BasicCalculation.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <map>

using namespace std;

int round10(float num) {
	num /= 10;
	num = round(num);
	num *= 10;
	return (int)num;
}

float round01(float num) {
	num *= 10;
	num = round(num);
	num /= 10;
	return num;
}


typedef struct _myPPF {			// PPF(point pair feature)
	double distance;				// ���� int
	double angle_between;			// m1��m2���Ȃ��p int
	double angle_n1;				// m1���Ȃ��p int
	double angle_n2;				// m2���Ȃ��p int
	bool operator < (const _myPPF& rhs) const {
		// distance��r
		if (distance < rhs.distance) return true;
		if (distance > rhs.distance) return false;
		// angle_between��r�idistance==rhs.distance�̏ꍇ�j
		if (angle_between < rhs.angle_between) return true;
		if (angle_between > rhs.angle_between) return false;
		// angle_n1��r�iangle_between==rhs.angle_between�̏ꍇ�j
		if (angle_n1 < rhs.angle_n1) return true;
		if (angle_n1 > rhs.angle_n1) return false;
		// angle_n2��r�iangle_n1==rhs.angle_n1�̏ꍇ�j
		if (angle_n2 < rhs.angle_n2) return true;
		if (angle_n2 > rhs.angle_n2) return false;
		return false;
	}
} myPPF;


// �ʏ�PPF
vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> make_PPFs(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	float distance_resolution = 3.0f; // ����\(�ʎq���̋�؂�)
	float angle_resolution = 10.0f; // ����\(�ʎq���̋�؂�)

	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			if (i == j) continue;
			pcl::PointXYZ m2(j->x, j->y, j->z);

			myPPF m1m2;

			// ����
			double point_distance = sqrt(euclidean_distance(m1, m2));
			point_distance /= distance_resolution; // �ʎq��
			point_distance = (int)point_distance;
			point_distance *= distance_resolution;
			m1m2.distance = point_distance;

			// �p�x
			pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
			pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
			double angle0 = cal_angle(m1_normal, m2_normal);
			angle0 /= angle_resolution; // �ʎq��
			angle0 = (int)angle0;
			angle0 *= angle_resolution;
			m1m2.angle_between = angle0;

			// �p�x
			pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
			double angle1 = cal_angle(m1_m2, m1_normal);
			double angle10 = angle1 / angle_resolution; // �ʎq��
			angle10 = (int)angle10;
			angle10 *= angle_resolution;

			// �p�x
			pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
			double angle2 = cal_angle(m2_m1, m2_normal);
			double angle20 = angle2 / angle_resolution; // �ʎq��
			angle20 = (int)angle20;
			angle20 *= angle_resolution;

			pair<pcl::PointXYZ, pcl::PointXYZ> couple;
			if (angle1 <= angle2) {
				m1m2.angle_n1 = angle10;
				m1m2.angle_n2 = angle20;
				couple = make_pair(m1, m2);
			}
			else {
				m1m2.angle_n1 = angle20;
				m1m2.angle_n2 = angle10;
				couple = make_pair(m2, m1);
			}

			PPFs.push_back(make_pair(m1m2, couple));

		}
	}
	return PPFs;
}


// ��d�o�^�Ȃ�
vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> make_PPFs1(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	float threshold = 10.0f; //1.0f
	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			pcl::PointXYZ m2(j->x, j->y, j->z);
			double point_distance = euclidean_distance(m1, m2);
			//if (point_distance > threshold) {
			if (point_distance < threshold) {
				myPPF m1m2;
				//point_distance = round10(point_distance); // �ʎq��
				point_distance /= 100;
				point_distance = round(point_distance);
				point_distance *= 100;
				m1m2.distance = (int)point_distance;

				pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
				pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
				double angle0 = cal_angle(m1_normal, m2_normal);
				angle0 = round10(angle0); // �ʎq��
				m1m2.angle_between = angle0;

				pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
				double angle1 = cal_angle(m2_m1, m1_normal);
				double angle10 = round10(angle1); // �ʎq��

				pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
				double angle2 = cal_angle(m1_m2, m2_normal);
				double angle20 = round10(angle2); // �ʎq��	

				pair<pcl::PointXYZ, pcl::PointXYZ> couple;
				if (angle1 <= angle2) {
					m1m2.angle_n1 = angle10;
					m1m2.angle_n2 = angle20;
					couple = make_pair(m1, m2);
				}
				else {
					m1m2.angle_n1 = angle20;
					m1m2.angle_n2 = angle10;
					couple = make_pair(m2, m1);
				}
				PPFs.push_back(make_pair(m1m2, couple));
			}
		}
	}
	return PPFs;
}

// ��d�o�^����
vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> make_PPFs2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	float threshold = 10.0f; //1.0f
	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			pcl::PointXYZ m2(j->x, j->y, j->z);
			double point_distance = euclidean_distance(m1, m2);
			if (point_distance > threshold) {
				myPPF m1m2;
				//point_distance = round10(point_distance); // �ʎq��
				point_distance /= 100;
				point_distance = round(point_distance);
				point_distance *= 100;
				m1m2.distance = (int)point_distance;

				pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
				pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
				double angle0 = cal_angle(m1_normal, m2_normal);
				angle0 = round10(angle0); // �ʎq��
				m1m2.angle_between = angle0;

				pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
				double angle1 = cal_angle(m2_m1, m1_normal);
				double angle10 = round10(angle1); // �ʎq��

				pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
				double angle2 = cal_angle(m1_m2, m2_normal);
				double angle20 = round10(angle2); // �ʎq��	

				pair<pcl::PointXYZ, pcl::PointXYZ> couple;
				if (angle10 == angle20) {
					m1m2.angle_n1 = angle10;
					m1m2.angle_n2 = angle20;
					couple = make_pair(m1, m2);
					PPFs.push_back(make_pair(m1m2, couple));
					m1m2.angle_n1 = angle20;
					m1m2.angle_n2 = angle10;
					couple = make_pair(m2, m1);
					PPFs.push_back(make_pair(m1m2, couple));
				}
				else if (angle10 < angle20) {
					m1m2.angle_n1 = angle10;
					m1m2.angle_n2 = angle20;
					couple = make_pair(m1, m2);
					PPFs.push_back(make_pair(m1m2, couple));
				}
				else {
					m1m2.angle_n1 = angle20;
					m1m2.angle_n2 = angle10;
					couple = make_pair(m2, m1);
					PPFs.push_back(make_pair(m1m2, couple));
				}
			}
		}
	}
	return PPFs;
}


pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr myPPF_matching(vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF, vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF) {

	/*
	// ����f�[�^�̓����_��PPF���}���`�}�b�v�Ɋi�[
	std::multimap<myPPF, std::pair<pcl::PointXYZ, pcl::PointXYZ>> keypoint_map;
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = keypoint_PPF.begin(); i != keypoint_PPF.end(); i++) {
	keypoint_map.insert(*i);
	}
	// STL�f�[�^�̓����_��PPF���}���`�}�b�v���猟��(�d���v�Z����)�����Ƃŏd������菜���K�v���� ��Α��̑Ή�(���Ή��C��Ή��̏����Ŏ�菜����)
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr result(new pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>());
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
	auto p = keypoint_map.equal_range(i->first);
	for (auto it = p.first; it != p.second; ++it) {
	result->push_back(make_pair(i->second, it->second));
	}
	}
	*/

	// ����f�[�^�̓����_��PPF���}�b�v�Ɋi�[
	std::map<myPPF, std::pair<pcl::PointXYZ, pcl::PointXYZ>> keypoint_map;
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = keypoint_PPF.begin(); i != keypoint_PPF.end(); i++) {
		keypoint_map.insert(*i);
	}
	// STL�f�[�^�̓����_��PPF���}�b�v���猟��(�d���v�Z�Ȃ�)�����m�łȂ� ��Έ�̑Ή�
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr result(new pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>());
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
		auto p = keypoint_map.equal_range(i->first);
		for (auto it = p.first; it != p.second; ++it) {
			result->push_back(make_pair(i->second, it->second));
		}
	}

	return result;
}

// �ʏ�PPF �ȈՔ�
vector<myPPF> make_lightPPFs(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<myPPF> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	//float distance_resolution = 1.0f; // ����\(�ʎq���̋�؂�) 3.0f 2.0f
	//float angle_resolution = 1.0f; // ����\(�ʎq���̋�؂�) 30.0f

	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			if (i == j) continue;
			pcl::PointXYZ m2(j->x, j->y, j->z);
			pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
			myPPF m1m2;

			// ����
			double point_distance = sqrt(euclidean_distance(m1, m2));
			//point_distance /= distance_resolution; // �ʎq��
			//point_distance = (int)point_distance;
			//point_distance *= distance_resolution;
			m1m2.distance = point_distance;

			// �p�x
			double angle0 = cal_angle(m1_normal, m2_normal);
			//angle0 /= angle_resolution; // �ʎq��
			//angle0 = (int)angle0;
			//angle0 *= angle_resolution;
			m1m2.angle_between = angle0;

			// �p�x
			pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
			double angle1 = cal_angle(m1_m2, m1_normal);
			//double angle10 = angle1 / angle_resolution; // �ʎq��
			//angle10 = (int)angle10;
			//angle10 *= angle_resolution;

			// �p�x
			pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
			double angle2 = cal_angle(m2_m1, m2_normal);
			//double angle20 = angle2 / angle_resolution; // �ʎq��
			//angle20 = (int)angle20;
			//angle20 *= angle_resolution;

			if (angle1 <= angle2) {
				m1m2.angle_n1 = angle1; // angle10
				m1m2.angle_n2 = angle2; // angle20
			}
			else {
				m1m2.angle_n1 = angle2; // angle20
				m1m2.angle_n2 = angle1; // angle10
			}

			PPFs.push_back(m1m2);

		}
	}
	return PPFs;
}

// �ʏ�PPF �ȈՔŁ@�̃}�b�`���O
int myLightPPF_matching(vector<myPPF> keypoint_PPF, vector<myPPF> stl_PPF) {

	/*
	std::set<myPPF> PPFset;
	outputfile777 << "stl_PPF" << endl;
	for (vector<myPPF>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
	PPFset.insert(*i);
	outputfile777 << i->distance << "\t" << i->angle_between << "\t" << i->angle_n1 << "\t" << i->angle_n2 << endl;
	}
	outputfile777 << endl;
	outputfile777 << "keypoint_PPF" << endl;
	int count = 0;
	for (vector<myPPF>::iterator i = keypoint_PPF.begin(); i != keypoint_PPF.end(); i++) {
	outputfile777 << i->distance << "\t" << i->angle_between << "\t" << i->angle_n1 << "\t" << i->angle_n2 << endl;
	auto itr = PPFset.find(*i);
	if (itr != PPFset.end()) count++;
	}
	outputfile777 << endl;
	*/


	double threshold_distance = 1.0f; // 3.0f
	double threshold_angle = 15.0f; // 30.0f
	int count = 0;
	for (vector<myPPF>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
		for (vector<myPPF>::iterator j = keypoint_PPF.begin(); j != keypoint_PPF.end(); j++) {
			bool a = (fabs(i->distance - j->distance) <= threshold_distance);
			bool b = (fabs(i->angle_between - j->angle_between) <= threshold_angle);
			bool c = (fabs(i->angle_n1 - j->angle_n1) <= threshold_angle);
			bool d = (fabs(i->angle_n1 - j->angle_n1) <= threshold_angle);
			//accuracy_file << a << " " << b << " " << c << " " << d << endl;
			if (a&&b&&c&&d) {
				//accuracy_file << (a&&b&&c&&d) << endl;
				count++;
				break;
			}
		}
		//accuracy_file << (a&&b&&c&&d) << endl;
	}


	return count;
}

#endif // _PPF_HPP_