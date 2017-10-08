#include "main.h"


//float dot_product3_cal(pcl::PointXYZ p, pcl::PointXYZ q);
pcl::PointCloud<pcl::PointNormal>::Ptr make_occlusion(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f);//���_�̌���

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		pcl::PointXYZ n(it->normal_x, it->normal_y, it->normal_z);
		pcl::PointXYZ vppi(view_point.x - it->x, view_point.y - it->y, view_point.z - it->z);
		if (dot_product3_cal(n, vppi) < 0) continue; // ������
		//if (dot_product3_cal(n, vppi) > 0) continue; // ���
		cloud_normals->push_back(*it);
	}

	return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>()); //�ߖT�_�z��

	float r = 7.0f; //���ar 5.0f
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) { //�S�̂̓_�Q��1�_�Â��o
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		float judge = euclidean_distance(tmp, attention_point); //norm���v�Z
		if (judge < r*r) { //���ar�����߂�������ߖT�_�z��ɒǉ�
			neighborhood->push_back(tmp);
			continue;
		}
	}
	return neighborhood;
}






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

inline float bhattacharyya(array<float, 18> p, array<float, 18> q) {
	float result = 0;
	for (int i = 0; i < 18; i++) {
		result += p[i] * q[i];
	}
	return sqrt(result);
}

inline float uniqueness(array<float, 18> n, vector<array<float, 18>> NDH) {
	float result = 0;
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result += (1 - bhattacharyya(n, *it));
	}
	result /= NDH.size();
	return result;
}

inline vector<float> all_uniqueness(vector<array<float, 18>> NDH) {
	vector<float> result;
	result.reserve(NDH.size());
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result.push_back(uniqueness(*it, NDH));
	}
	return result;
}


inline float bhattacharyya36(array<float, 36> p, array<float, 36> q) {
	float result = 0;
	for (int i = 0; i < 36; i++) {
		result += p[i] * q[i];
	}
	return result;
	//return sqrt(result);
}

inline float uniqueness36(array<float, 36> n, vector<array<float, 36>> NDH) {
	float result = 0;
	for (vector<array<float, 36>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result += bhattacharyya36(n, *it);
	}
	result /= NDH.size();
	return result;
}

inline vector<float> all_uniqueness36(vector<array<float, 36>> NDH) {
	vector<float> result;
	result.reserve(NDH.size());
	for (vector<array<float, 36>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result.push_back(1 - uniqueness36(*it, NDH));
	}
	return result;
}



void data_matching(string path, vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF) {
	// �H��f�[�^�ǂݍ���
	/*pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = stl_points(path);*/
	outputfile777 << path << endl;

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud2 = stl_points(path);
	kdtree.setInputCloud(stl_cloud2);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = stl_cloud2->begin(); it != stl_cloud2->end(); it++) {
		if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 10) {
			stl_cloud->push_back(*it);
		}
	}


	// �H��f�[�^��PPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF = make_PPFs(stl_cloud);
	outputfile777 << "stl_cloud->size(): " << stl_cloud->size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;

	// �f�[�^�}�b�`���O
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr match_cloud = myPPF_matching(keypoint_PPF, stl_PPF);
	outputfile777 << "match_cloud_size: " << match_cloud->size() << endl;
	outputfile777 << "accuracy: " << (float)match_cloud->size() / stl_PPF.size() * 100 << endl;
	//int errata2_size = errata2(match_cloud);
	//outputfile777 << "errata2_size: " << errata2_size << endl;
	//outputfile777 << "accuracy2: " << (float)errata2_size / stl_PPF.size() * 100 << endl;
	//int errata3_size = errata3(match_cloud);
	//outputfile777 << "errata3_size: " << errata3_size << endl;
	//outputfile777 << "accuracy3: " << (float)errata3_size / stl_PPF.size() * 100 << endl;
	//outputfile777 << endl;
}

void data_matching2(string path, vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF) {
	// �H��f�[�^�ǂݍ���
	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = stl_points(path);
	outputfile777 << path << endl;

	STLDATA stl_object(path);
	pcl::PointCloud<pcl::PointXYZ> merged_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr triangle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	merged_cloud = *triangle_cloud;
	for (int i = 0; i<stl_object.getDatanum(); i++) {
		TRIANGLE tri;
		stl_object.getData(i, &tri);
		merged_cloud += *interpolation_triangle(tri, false);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsCurvature(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(merged_cloud.makeShared());
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*keypointsCurvature);
	// �����_����
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	detector.setRadius(5.0);//3.0 5.0 7.0
	detector.setInputCloud(keypointsCurvature);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		keypoints3D->push_back(tmp);
	}
	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypoints3D->size(); i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypoints3D->at(i));
		pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
		stl_cloud->push_back(normal);
	}




	// �H��f�[�^��PPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF = make_PPFs(stl_cloud);
	outputfile777 << "stl_cloud->size(): " << stl_cloud->size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;

	// �f�[�^�}�b�`���O
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr match_cloud = myPPF_matching(keypoint_PPF, stl_PPF);
	outputfile777 << "match_cloud_size: " << match_cloud->size() << endl;
	outputfile777 << "accuracy: " << (float)match_cloud->size() / stl_PPF.size() * 100 << endl;
	//int errata2_size = errata2(match_cloud);
	//outputfile777 << "errata2_size: " << errata2_size << endl;
	//outputfile777 << "accuracy2: " << (float)errata2_size / stl_PPF.size() * 100 << endl;
	//int errata3_size = errata3(match_cloud);
	//outputfile777 << "errata3_size: " << errata3_size << endl;
	//outputfile777 << "accuracy3: " << (float)errata3_size / stl_PPF.size() * 100 << endl;
	//outputfile777 << endl;
}

pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtraction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> make_stl_PPFs(string path) {
	// �H��f�[�^�ǂݍ���
	outputfile777 << path << endl;

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(path, false);

	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud_lattice);

	// �����_�̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(surfaceN);

	// �H��f�[�^��PPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF = make_PPFs(attention_point);

	return stl_PPF;
}




#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool TF)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);		// �O��l����������_�Q�����
	sor.setMeanK(50);				// ��������ߖT�_�̐���ݒ�
	sor.setStddevMulThresh(1.0);	// �����̕W���΍��̒l��ݒ�
	sor.setNegative(TF);			// �O��l���o�͂���ꍇ��true�ɂ���
	sor.filter(*cloud_filtered);	// �o��
	return cloud_filtered;
}

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr Detect_wall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool TF)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);	// ���o���郂�f���̃^�C�v���w��
	seg.setMethodType(pcl::SAC_RANSAC);		// ���o�Ɏg�p������@���w��
	seg.setDistanceThreshold(0.10);			// 臒l 0.01, 0.05
											//seg.setDistanceThreshold(0.10);			// 臒l 0.01, 0.05

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(TF);//True�̏ꍇ�o�͂͌��o���ꂽ�ǈȊO�̃f�[�^ false�̏ꍇ�͕ǂ̃f�[�^
	extract.filter(*cloud_output);

	return cloud_output;//�o��
}

#include <pcl/filters/passthrough.h>
//pcl::PointCloud<pcl::PointXYZ>::Ptr Detect_Cylinder(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
pcl::PointCloud<pcl::PointNormal>::Ptr Detect_Cylinder(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	pcl::copyPointCloud(*cloud, *cloud_filtered);
	pcl::copyPointCloud(*cloud, *cloud_normals);
	for (int i = 0; i < 100; i++) {
		cout << "PointXYZ: " << cloud_filtered->at(i).x << " " << cloud_filtered->at(i).y << " " << cloud_filtered->at(i).z << endl;
		cout << "Normal: " << cloud_normals->at(i).normal_x << " " << cloud_normals->at(i).normal_y << " " << cloud_normals->at(i).normal_z << endl;
	}

	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals <pcl::PointXYZ, pcl::Normal> seg;

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER); // SACMODEL_CYLINDER SACMODEL_CIRCLE3D
	seg.setMethodType(pcl::SAC_RANSAC); // SAC_RANSAC SAC_RRANSAC SAC_RMSAC SAC_MLESAC
	seg.setNormalDistanceWeight(0.087); //0.01  0.087=sin5�x 0.174=sin10�x
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(1.00); //0.05	0.10
	seg.setRadiusLimits(0, 30.0); // 0,0.1		0, 20.0
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	//seg.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f)); // �C�ӎ��H(�����Őݒ�) �����I�ɒǉ�


	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud_filtered);
	pcl::ExtractIndices<pcl::PointNormal> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>());
	extract.filter(*cloud_cylinder);

	return cloud_cylinder;//�o��
}


class PointYCompareMin {
public:
	bool operator()(const pcl::PointXYZ &left, const pcl::PointXYZ &right) const {
		if (left.y > right.y) return true;
		if (left.y < right.y) return false;
		return false;
	}
};


class CurvatureCompareDescending {
public:
	bool operator()(const pcl::PointNormal &left, const pcl::PointNormal &right) const {
		if (left.curvature > right.curvature) return true;
		if (left.curvature < right.curvature) return false;
		return false;
	}
};

class CurvatureCompareAscending {
public:
	bool operator()(const pcl::PointNormal &left, const pcl::PointNormal &right) const {
		if (left.curvature < right.curvature) return true;
		if (left.curvature > right.curvature) return false;
		return false;
	}
};

class UniquenessCompareDescending {
public:
	bool operator()(const pair<float, int> &left, const pair<float, int> &right) const {
		if (left.first > right.first) return true;
		if (left.first < right.first) return false;
		return false;
	}
};

class UniquenessCompareAscending {
public:
	bool operator()(const pair<float, int> &left, const pair<float, int> &right) const {
		if (left.first < right.first) return true;
		if (left.first > right.first) return false;
		return false;
	}
};



// �Ɍ`���q�X�g�O�����̌v�Z
std::vector<array<float, 36>> myPolarHistograms36(pcl::PointCloud<pcl::PointNormal>::Ptr searchPoints, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	std::vector<array<float, 36>> PHs;
	PHs.reserve(cloud->size());

	float radius = 3.0f;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = searchPoints->begin(); it != searchPoints->end(); it++) {
		pcl::PointNormal searchPoint = *it;
		vector<pair<int, float>> neighbors_inf = radius_search_inf(searchPoint, radius, cloud);
		pcl::PointCloud<pcl::PointNormal>::Ptr neighbors(new pcl::PointCloud<pcl::PointNormal>);

		// �ߖT�_��񂩂�ߖT�_�̔z����쐬
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			neighbors->push_back(cloud->points[itr->first]);
		}

		// ���ˉe�̂��߂̏���(���ړ_�̖@������]�Cz���ɍ��킹��}�g���N�X)
		pcl::PointXYZ searchN(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z);
		pcl::PointXYZ projectN(0.0f, 0.0f, 1.0f);
		pcl::PointXYZ axisR = cross_product3_cal(searchN, projectN);
		float cosR = dot_product3_cal(searchN, projectN);
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



		// ���ˉe(�ߖT�_�̖@������])
		/*vector<pair<pcl::PointXYZ, float>> neighborsR_inf;
		neighborsR_inf.reserve(neighbors_inf.size());
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
		pcl::PointNormal pi = cloud->points[itr->first];
		pcl::PointXYZ rotatePoint;
		rotatePoint.x = rotate_matrix[0][0] * pi.normal_x + rotate_matrix[0][1] * pi.normal_y + rotate_matrix[0][2] * pi.normal_z;
		rotatePoint.y = rotate_matrix[1][0] * pi.normal_x + rotate_matrix[1][1] * pi.normal_y + rotate_matrix[1][2] * pi.normal_z;
		rotatePoint.z = rotate_matrix[2][0] * pi.normal_x + rotate_matrix[2][1] * pi.normal_y + rotate_matrix[2][2] * pi.normal_z;
		float squared_distance = itr->second;
		neighborsR_inf.push_back(make_pair(rotatePoint, squared_distance));
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr neighborsR(new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<pair<pcl::PointXYZ, float>>::iterator itr = neighborsR_inf.begin(); itr != neighborsR_inf.end(); itr++) {
		neighborsR->push_back(itr->first);
		}*/

		// ���ˉe(�ߖT�_�̖@������])
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighborsR(new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			pcl::PointNormal pi = cloud->points[itr->first];
			pcl::PointXYZ rotatePoint;
			rotatePoint.x = rotate_matrix[0][0] * pi.normal_x + rotate_matrix[0][1] * pi.normal_y + rotate_matrix[0][2] * pi.normal_z;
			rotatePoint.y = rotate_matrix[1][0] * pi.normal_x + rotate_matrix[1][1] * pi.normal_y + rotate_matrix[1][2] * pi.normal_z;
			rotatePoint.z = rotate_matrix[2][0] * pi.normal_x + rotate_matrix[2][1] * pi.normal_y + rotate_matrix[2][2] * pi.normal_z;
			neighborsR->push_back(rotatePoint);
		}

		array<array<double, 3>, 3> neighborsR_PCA = cal_PCA(neighborsR);
		//cout << "x: " << neighborsR_PCA[0][0] << "\ty: " << neighborsR_PCA[0][1] << "\tz: " << neighborsR_PCA[0][2] << endl;
		//cout << "x: " << neighborsR_PCA[1][0] << "\ty: " << neighborsR_PCA[1][1] << "\tz: " << neighborsR_PCA[1][2] << endl;
		//cout << "x: " << neighborsR_PCA[2][0] << "\ty: " << neighborsR_PCA[2][1] << "\tz: " << neighborsR_PCA[2][2] << endl;
		//cout << endl;



		// �e�_�ɂ�����ŋߖT�_�܂ł̋����̑��a(�d�݌W��w1�̌v�Z�Ŏg�p)
		int K = 2;
		float nearestSum = 0;
		for (pcl::PointCloud<pcl::PointNormal>::iterator itr = neighbors->begin(); itr != neighbors->end(); itr++) {
			vector<pair<int, float>> nearestP = nearest_search1(*itr, K, neighbors);
			for (vector<pair<int, float>>::iterator iter = nearestP.begin(); iter != nearestP.end(); iter++) {
				nearestSum += iter->second;
			}
		}


		// �Ɍ`���q�X�g�O�����̏�����
		array<float, 36> angleH;
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] = 0;

		// �q�X�g�O�����̐��K���p
		float sum_total = 0;
		// �Ɍ`���q�X�g�O�����̍쐬
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			pcl::PointNormal pi = cloud->points[itr->first];
			float squared_distance = itr->second;
			float rotateX = rotate_matrix[0][0] * pi.normal_x + rotate_matrix[0][1] * pi.normal_y + rotate_matrix[0][2] * pi.normal_z;
			float rotateY = rotate_matrix[1][0] * pi.normal_x + rotate_matrix[1][1] * pi.normal_y + rotate_matrix[1][2] * pi.normal_z;
			float rotateZ = rotate_matrix[2][0] * pi.normal_x + rotate_matrix[2][1] * pi.normal_y + rotate_matrix[2][2] * pi.normal_z;
			/*if (rotateX > 1) rotateX = 1;
			if (rotateX < -1) rotateX = -1;
			double radian = acos(rotateX);
			double angle = radian * 180 / M_PI;
			if (rotateY < 0) angle = 360 - angle;*/

			//double cosR = neighborsR_PCA[2][0] * rotateX + neighborsR_PCA[2][1] * rotateY;
			//double sinR = neighborsR_PCA[2][0] * rotateY - neighborsR_PCA[2][1] * rotateX;
			//double cosR = neighborsR_PCA[1][0] * rotateX + neighborsR_PCA[1][1] * rotateY;
			//double sinR = neighborsR_PCA[1][0] * rotateY - neighborsR_PCA[1][1] * rotateX;
			double cosR = neighborsR_PCA[0][0] * rotateX + neighborsR_PCA[0][1] * rotateY;
			double sinR = neighborsR_PCA[0][0] * rotateY - neighborsR_PCA[0][1] * rotateX;
			if (cosR > 1) cosR = 1;
			if (cosR < -1) cosR = -1;
			double radian = acos(cosR);
			double angle = radian * 180 / M_PI;
			if (sinR < 0) angle = 360 - angle;

			angle /= 10;
			int indexH = (int)angle;
			vector<pair<int, float>> nearPi = nearest_search1(pi, K, neighbors);
			float nearestPi = 0;
			for (vector<pair<int, float>>::iterator iter = nearPi.begin(); iter != nearPi.end(); iter++) {
				nearestPi += iter->second;
			}
			float w1 = sqrt(nearestPi / nearestSum);
			float w2 = radius - sqrt(squared_distance);
			float w3 = 1 - (searchPoint.normal_x*rotateX + searchPoint.normal_y*rotateY + searchPoint.normal_z*rotateZ);
			angleH[indexH] += w1*w2*w3;
			sum_total += w1*w2*w3;
		}

		// �Ɍ`���q�X�g�O�����̐��K��
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] /= sum_total;

		// �Ɍ`���q�X�g�O�����ɂ�����s�[�N�̊i�[�ʒu�𒲐�(�ŏ��̃r���ɓ���)
		std::array<float, 36>::iterator maxIt = std::max_element(angleH.begin(), angleH.end());
		int maxIndex = std::distance(angleH.begin(), maxIt);
		array<float, 36> tempH;
		for (int j = 0; j < tempH.size(); j++)
			tempH[j] = angleH[j];
		for (int j = 0; j < angleH.size(); j++) {
			int tempIndex = (maxIndex - j + 36) % 36;
			angleH[j] = tempH[tempIndex];
		}

		// �Ɍ`���q�X�g�O�����̔z��Ɋi�[
		PHs.push_back(angleH);
	}

	for (vector<array<float, 36>>::iterator it = PHs.begin(); it != PHs.end(); it++) {
		for (int i = 0; i < it->size(); i++) {
			outputfile << "PHs[" << i << "]: " << it->at(i) << endl;
		}
		outputfile << endl;
	}

	return PHs;
}


// �Ǐ��`��̓Ǝ����ɒ��ڂ������̔F���ɗL����3-D�����_�̎������o
pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtraction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\06\\outputfile\\result004.txt");

	// �_���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// �ȗ��������_�̒��o
	//pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints = cloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		if (it->curvature < 0.03) continue;
		highCurvaturePoints->push_back(*it);
	}

	//return highCurvaturePoints; // ���ȗ��_�Q�̊m�F

	// �Ɍ`���q�X�g�O�����̎Z�o
	std::vector<array<float, 36>> PHs = myPolarHistograms36(highCurvaturePoints, cloud);

	// �Ǝ����w�W�̎Z�o
	vector<float> Sn = all_uniqueness36(PHs);

	// �ȗ��̈�ɓƎ����w�W�̒l����
	for (int i = 0; i < highCurvaturePoints->size(); i++) {
		highCurvaturePoints->at(i).curvature = Sn[i];
	}

	// �Ǝ����w�W�̒l�Ō���
	/*
	// �Ǝ����w�W�������_�̒��o
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
	if (it->curvature < 0.9) continue; //0.85
	stl_cloud->push_back(*it);
	}

	// �����_�̌���(���̈�T��)
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending> cQueue;
	while (!stl_cloud->empty()) {
	cQueue = priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending>();
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = stl_cloud->begin(); it != stl_cloud->end(); it++) {
	cQueue.push(*it);
	}
	kdtree.setInputCloud(stl_cloud); //
	pcl::PointNormal searchPoint = cQueue.top();
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	stl_cloud->erase(stl_cloud->begin() + pointIdxRadiusSearch[i]);
	}
	cloud_ptr->push_back(searchPoint);
	}
	*/


	// �_�̐��Ō���

	// �����_�̌���(���̈�T��)
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending> cQueue;
	while (!highCurvaturePoints->empty()) {
		cQueue = priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending>();
		for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
			cQueue.push(*it);
		}
		kdtree.setInputCloud(highCurvaturePoints); //
		pcl::PointNormal searchPoint = cQueue.top();
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
			for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
				highCurvaturePoints->erase(highCurvaturePoints->begin() + pointIdxRadiusSearch[i]);
		}
		cloud_ptr->push_back(searchPoint);
		if (cloud_ptr->size() >= 20) break; //25
	}

	return cloud_ptr;
}

/*
pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessIndex(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

}
*/

void data_matchingL(string filename, vector<myPPF> target_PPF) {
	//vector<myPPF> data_matchingL(string filename, vector<myPPF> target_PPF) {

	// �H��f�[�^���
	outputfile777 << filename << endl;

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// �����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals);
	// �H��f�[�^��PPF
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);


	// �f�[�^�}�b�`���O (�ȈՔ�)
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "match_cloud->size: " << match_cloud << endl;
	outputfile777 << "accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;

	outputfile777 << endl;

	//return source_PPF; //
}





/*
// �@������
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// �_���
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// �����_�̖@������
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // �S�̂̏d�S���v�Z
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//���_�̌���
ne.setInputCloud(cloud);//�@���̌v�Z���s�������_�Q���w�肷��
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREE�����
ne.setSearchMethod(tree);//�������@��KDTREE���w�肷��
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);//�@����������ϐ�
ne.setRadiusSearch(3.0);//�������锼�a���w�肷�� 3.0, 1.0
ne.compute(*cloud_normals);//�@�����̏o�͐���w�肷��

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).x = cloud->at(i).x;
cloud_normals->at(i).y = cloud->at(i).y;
cloud_normals->at(i).z = cloud->at(i).z;
}

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).normal_x *= (-1);
cloud_normals->at(i).normal_y *= (-1);
cloud_normals->at(i).normal_z *= (-1);
}

return cloud_normals;


}

pcl::PointNormal flipNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point, pcl::PointXYZ average_point) {
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

// �@������
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// �_���
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// �����_�̖@������
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // �S�̂̏d�S���v�Z
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;


//�@����������ϐ�
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREE�����
pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setSearchMethod(tree);// �������@��KDTREE���w�肷��
ne.setInputCloud(keypointsXYZ); // �@���̌v�Z���s�������_�Q���w�肷��
ne.setSearchSurface(cloud); // �@���v�Z�Ɏg���_�Q���w�肷��
ne.setRadiusSearch(radius);//�������锼�a���w�肷�� 3.0, 1.0
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//���_�̌���
ne.compute(*cloud_normals);//�@�����̏o�͐���w�肷��


// ne.setInputCloud(keypointsXYZ);	�Ŗ@�����v�Z�����ꍇ��
// ne.setSearchSurface(cloud);		��ǉ����Ė@�����v�Z�����ꍇ�̈Ⴂ���m�F����(visual��)
// ���̌�Ckdtree�ɉ��������Ă��邩�m�F(cloud�������Ă���Ƃ悢�C�����炭�Ⴄ����)
// kdtree��keypointsXYZ�������Ă����ꍇ�͐V���ɍ�蒼��(cloud��������kdtree)

for (pcl::PointNormal it : *cloud_normals) {
if (kdtree.radiusSearch(it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroidN; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroidN);
pcl::PointXYZ cloud_average(xyz_centroidN[0], xyz_centroidN[1], xyz_centroidN[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}


for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).x = cloud->at(i).x;
cloud_normals->at(i).y = cloud->at(i).y;
cloud_normals->at(i).z = cloud->at(i).z;
}

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).normal_x *= (-1);
cloud_normals->at(i).normal_y *= (-1);
cloud_normals->at(i).normal_z *= (-1);
}

return cloud_normals;


}
*/


void save_MyLightPPF(string filename) {

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �@���̐���
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // 08
	// �����_���o�C�����ʌv�Z
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals); // 08

	// �@���̐����C�����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction_HarrisN(cloud_lattice); // Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = ISS3Ddetector(cloud_lattice); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = SUSANdetector(cloud_lattice); // SUSAN

	// �����_���o�C�����ʌv�Z
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // ����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr attention_point = myFeaturePointExtraction_HarrisN2(cloud_normals); // ����
	//pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(attention_point); // ���� ���ړ_�̏d�S(�ߖT�_�̏d�S�ł���Ȃ�)

	// �H��f�[�^��PPF
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);

	filename.erase(--filename.end());
	filename.erase(--filename.end());
	filename.erase(--filename.end());
	filename += "txt";
	ofstream savefile(filename);
	////////
	//savefile << "cloud_in->size(): " << cloud_in->size() << endl;
	//savefile << "cloud_lattice->size(): " << cloud_lattice->size() << endl;
	//savefile << "attention_point->size(): " << attention_point->size() << endl;
	//savefile << "source_PPF.size(): " << source_PPF.size() << endl;
	////////
	for (vector<myPPF>::iterator it = source_PPF.begin(); it != source_PPF.end(); it++) {
		savefile << it->distance << " " << it->angle_between << " " << it->angle_n1 << " " << it->angle_n2 << endl;
	}
	savefile.close();
}

vector<myPPF> load_MyLightPPF(string filename) {

	vector<myPPF> source_PPF;
	source_PPF.reserve(300);

	ifstream loadfile(filename);
	string line;
	while (getline(loadfile, line)) {
		std::string word;
		std::istringstream stream(line);
		myPPF sample;
		getline(stream, word, ' ');
		sample.distance = stod(word);
		getline(stream, word, ' ');
		sample.angle_between = stod(word);
		getline(stream, word, ' ');
		sample.angle_n1 = stod(word);
		getline(stream, word, ' ');
		sample.angle_n2 = stod(word);
		source_PPF.push_back(sample);
	}
	loadfile.close();

	return source_PPF;
}

void save_accuracy(string filename, vector<myPPF> target_PPF) {

	vector<myPPF> source_PPF = load_MyLightPPF(filename);

	// �H��f�[�^���
	accuracy_file << filename << endl;

	// �f�[�^�}�b�`���O (�ȈՔ�)
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	accuracy_file << "target_PPF->size: " << target_PPF.size() << endl; //
	accuracy_file << "source_PPF->size: " << source_PPF.size() << endl; //
	accuracy_file << "match_cloud->size: " << match_cloud << endl;
	accuracy_file << "target_accuracy: " << (float)match_cloud / target_PPF.size() * 100 << endl;
	accuracy_file << "source_accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	accuracy_file << endl;
}















//#include "MT.h"
//#include "randUnitVector.h"



#include <pcl/features/principal_curvatures.h>
// --------------
// -----Main-----
// --------------
int main(int argc, char* argv[]) {
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\06\\outputfile\\result004.txt");
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�ł����킹\\20\\outputfile\\20_5\\test\\endmill\\output000.txt");
	//std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	//double elapsed;


	// ------------------------------------------
	// �H��z���_�[�ʂƍH��˂��o���ʂ����߂鏈��
	// ------------------------------------------


	// �H��p���i�Œ�j
	/*pcl::Normal tool_normal(0.0f, 0.0f, 1.0f);
	float toolN_norm = sqrt((tool_normal.normal_x * tool_normal.normal_x) + (tool_normal.normal_y * tool_normal.normal_y) + (tool_normal.normal_z * tool_normal.normal_z));
	double theta_cos = 0.9962; // 5�x*/


	/*
	// ����f�[�^�ǂݍ��݁i�S�āj
	const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\dr1_whitesprayed2mod2.stl";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\ex1_whitesprayed2mod2.stl";
	STLDATA stl_object(filename);

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\endmill\\OCTACUT322S32RB.STL";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\drill_20170119\\merged_pointcloud_�m�C�Y�����L_ascii.stl";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\2017_03_03\\5.0_-5.0.STL";
	//STLDATA stl_object(filename);



	// ����f�[�^�ǂݍ��݁i�H���[���j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	/////// �H���[�̂� ///////
	// �h����
	//if (point.y > -6) continue;	// �O��l�̏���������΂����OK
	if (point.y > -7) continue;		// �O��l�̏����Ȃ�
	//if (point.y > -20) continue;
	//if (point.y < -21) continue;
	// �o�C�g
	//if (point.y > -35) continue;	// ���s���̂�
	//if (point.y > -27) continue;
	//if (point.z < 65) continue;
	////////////////////////////
	cloud_in->points.push_back(point);
	}
	}
	//cout << "cloud_in->size(): " << cloud_in->size() << endl;


	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypointsCurvature->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypointsCurvature->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypointsCurvature->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(keypoint_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(keypointsCurvature, false);
	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	cout << "pointN: " << keypointsCurvature->size() << endl;


	// �����_����
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	//detector.setNonMaxSupression(false);
	detector.setRadius(5.0);//3.0 5.0 7.0
	detector.setInputCloud(keypointsCurvature);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
	tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
	keypoints3D->push_back(tmp);
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D = Remove_outliers2(keypoints3D_0, false);


	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypoints3D->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}
	*/









	/*
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\dr1_whitesprayed2mod2.stl";
	const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\ex1_whitesprayed2mod2.stl";

	// ����f�[�^�ǂݍ��݁i�H���[���j
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in0 = stl_points0(filename);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in1 = stl_nAveraging(cloud_in0);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in2 = stl_decimating0(cloud_in1);

	// ����f�[�^�ǂݍ��݁i�H���[���j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = stl_pointsXYZ(filename);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);

	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	// �����_�̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(surfaceN);
	*/

	/*
	// ����f�[�^��PPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);
	*/



	/*
	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);
	*/













	// �H��f�[�^�ǂݍ���
	//const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\������.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\������.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\�~��.STL";



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// �����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals);
	// �H��f�[�^��PPF
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> source_PPF = make_PPFs(attention_point);
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);
	*/



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	// �_�Q�f�[�^
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�O�a�o�C�g_PVJNR2525M.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�O�a�o�C�g_STGER2020K16.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����_MDW0300GS4.txt";
	const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����_MWE0300MA.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_in2);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // �_�Q�f�[�^�̂Ƃ�
	// RANSAC ���ʏ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	//cout << "cloud_in2->size(): " << cloud_in2->size() << endl;
	//cout << "cloud->size(): " << cloud->size() << endl;
	//cout << "cloud_lattice->size(): " << cloud_lattice->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // true�Ńm�C�Y true
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f); // 0.5f, 0.5f, 0.5f		10.0f, 10.0f, 10.0f
	sor2.filter(*cloud_lattice2);*/



	// �@���̐���
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// �����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals2);
	//�I�N���[�W�����L�� (�e�X�g)
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals3);*/


	// Harris �̓����_���o
	// �I�N���[�W�����Ȃ�
	/*pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(cloud_lattice2); //Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(cloud_lattice2); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN*/
	// �I�N���[�W��������
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_normals3, *surface0);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(surface0); // Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(surface0); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN


	// �H��f�[�^��PPF
	/*vector<myPPF> target_PPF = make_lightPPFs(attention_point2);*/



	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);*/

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	/*
	// �f�[�^�}�b�`���O (�ȈՔ�)
	//pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr match_cloud = myPPF_matching(target_PPF, source_PPF);
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "source_PPF.size(): " << source_PPF.size() << endl;
	outputfile777 << "target_PPF.size(): " << target_PPF.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud << endl;
	outputfile777 << "source_accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	outputfile777 << "target_accuracy: " << (float)match_cloud / target_PPF.size() * 100 << endl;

	outputfile777 << endl;
	*/












	/*
	ifstream outputfile456;
	string path;
	vector<myPPF> source_PPF;

	path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database\\06\\Alexandre\\NikonTurningTool.txt";
	string line;
	int count = 0;
	outputfile456.open(path, ios::in);

	getline(outputfile456, line); //

	while (getline(outputfile456, line)) {
	std::string word;
	std::istringstream stream(line);
	//while (getline(stream, word,  ' ')) {
	//double test = stod(word);
	//}
	myPPF sample;
	getline(stream, word, ' ');
	sample.distance = stod(word);
	getline(stream, word, ' ');
	sample.angle_between = stod(word);
	getline(stream, word, ' ');
	sample.angle_n1 = stod(word);
	getline(stream, word, ' ');
	sample.angle_n2 = stod(word);
	source_PPF.push_back(sample);
	}
	outputfile456.close();

	for (vector<myPPF>::iterator it = source_PPF.begin(); it != source_PPF.end(); it++) {
	cout << it->distance << " " << it->angle_between << " " << it->angle_n1 << " " << it->angle_n2 << endl;
	}
	*/









	/*
	string path;
	vector<myPPF> source_PPF;
	int match_cloud;

	path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database\\07\\Alexandre\\NikonTurningTool.txt";
	source_PPF = load_MyLightPPF(path);
	// �H��f�[�^���
	outputfile777 << path << endl;
	// �f�[�^�}�b�`���O (�ȈՔ�)
	match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "match_cloud->size: " << match_cloud << endl;
	outputfile777 << "accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	outputfile777 << endl;
	*/












	/*
	//std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	//double elapsed;
	// �_�Q�f�[�^
	//start = std::chrono::system_clock::now(); // �v���J�n����
	const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud);

	outputfile777 << "cloud_in->size(): " << cloud_in->size() << endl;
	outputfile777 << endl;
	end = std::chrono::system_clock::now();  // �v���I������
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/

















	/*
	// �f�[�^�}�b�`���O
	string path;
	string database_path;
	string feature_value = "";
	string type_company;
	string extension = ".STL";


	// 11��
	database_path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files";
	type_company = "";
	path = database_path + feature_value + type_company + "\\NikonTurningTool" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative0" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative2" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative3" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative4" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative5" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative6" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative7" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative8" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative9" + extension;
	save_MyLightPPF(path);

	// 15��
	database_path = "C:\\Users\\yuki\\Desktop\\yuki\\database";
	type_company = "\\mitsubishi\\bite";
	path = database_path + feature_value + type_company + "\\DCLNR3225P12" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DDJNR1616H11" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DDJNR2020K15" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DVJNR2020K16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MTJNR2525M16N" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MTJNR2525M22N" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PVJNR2020K16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PWLNR2525M06" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SDNCR0808K07-SM" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SRGCR1616H08" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SSSCR1212F09" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SVPPR1616M11-SM" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SVVCN1616H16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SXZCR2020K15" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TLHR2020K5" + extension;
	save_MyLightPPF(path);

	// 47��
	type_company = "\\sumitomo";
	path = database_path + feature_value + type_company + "\\DTR55CL2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CL2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CR2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CR2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QL2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QL2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QR2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QR2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-600S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-600W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-800S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-800W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-600S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-600W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-800S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-800W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SGWR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SGWR1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0707-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0808-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0909-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR0808" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR0808-60" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1010-60" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1212-60" + extension;
	save_MyLightPPF(path);

	// 12��
	type_company = "\\mitsubishi\\drill";
	path = database_path + feature_value + type_company + "\\BRA2000S25" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRA2500S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRA3000S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS2000S25" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS2500S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS3000S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX2000X2F25 - MVX2000X2F25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX2500X2F25 - MVX2500X2F25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX3000X2F32 - MVX3000X2F32-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN2000S25 - TAWSN2000S25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN2500S32 - TAWSN2500S32-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN3000S32 - TAWSN3000S32-1" + extension;
	save_MyLightPPF(path);
	*/











	/*
	// �f�[�^�}�b�`���O
	string path;
	string database_path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database";
	string feature_value = "\\Harris_20";
	string type_company;
	string extension = ".txt";


	// 11��
	type_company = "\\Alexandre";
	path = database_path + feature_value + type_company + "\\NikonTurningTool" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative0" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative2" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative3" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative4" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative5" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative6" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative7" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative8" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative9" + extension;
	save_accuracy(path, target_PPF);

	// 15��
	type_company = "\\mitsubishi\\bite";
	path = database_path + feature_value + type_company + "\\DCLNR3225P12" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DDJNR1616H11" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DDJNR2020K15" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DVJNR2020K16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MTJNR2525M16N" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MTJNR2525M22N" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PVJNR2020K16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PWLNR2525M06" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SDNCR0808K07-SM" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SRGCR1616H08" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SSSCR1212F09" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SVPPR1616M11-SM" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SVVCN1616H16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SXZCR2020K15" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TLHR2020K5" + extension;
	save_accuracy(path, target_PPF);

	// 47��
	type_company = "\\sumitomo";
	path = database_path + feature_value + type_company + "\\DTR55CL2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CL2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CR2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CR2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QL2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QL2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QR2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QR2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-600S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-600W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-800S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-800W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-600S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-600W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-800S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-800W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SGWR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SGWR1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0707-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0808-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0909-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR0808" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR0808-60" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1010-60" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1212-60" + extension;
	save_accuracy(path, target_PPF);

	// 12��
	type_company = "\\mitsubishi\\drill";
	path = database_path + feature_value + type_company + "\\BRA2000S25" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRA2500S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRA3000S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS2000S25" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS2500S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS3000S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX2000X2F25 - MVX2000X2F25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX2500X2F25 - MVX2500X2F25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX3000X2F32 - MVX3000X2F32-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN2000S25 - TAWSN2000S25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN2500S32 - TAWSN2500S32-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN3000S32 - TAWSN3000S32-1" + extension;
	save_accuracy(path, target_PPF);
	*/










	/*
	// �H��f�[�^�ǂݍ��݁i�f�[�^�}�b�`���O�j
	// 11��
	string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative0.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative2.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative3.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative4.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative5.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative6.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative7.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative8.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative9.STL";
	data_matchingL(path, target_PPF);

	// 15��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DCLNR3225P12.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR1616H11.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR2020K15.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DVJNR2020K16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M16N.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M22N.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PWLNR2525M06.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SDNCR0808K07-SM.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SRGCR1616H08.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SSSCR1212F09.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVPPR1616M11-SM.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVVCN1616H16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SXZCR2020K15.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\TLHR2020K5.STL";
	data_matchingL(path, target_PPF);

	// 47��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0707-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0808-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0909-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808-60.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010-60.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212-60.STL";
	data_matchingL(path, target_PPF);

	// 12��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2000S25.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2500S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA3000S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS3000S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2500X2F25 - MVX2500X2F25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX3000X2F32 - MVX3000X2F32-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2500S32 - TAWSN2500S32-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN3000S32 - TAWSN3000S32-1.STL";
	data_matchingL(path, target_PPF);
	*/




































	// �H��f�[�^�ǂݍ���
	const string filename = ".\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";



	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// �����_���o
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	//pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal = myFeaturePointExtraction(cloud_normals);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*source_keypointsNormal, *source_keypointsXYZ);
	// �����ʌv�Z
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = myFeatureDescription_FPFH2(cloud_normals, source_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr source_features = myFeatureDescription_SHOT352(cloud_normals, source_keypointsXYZ);



	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // true�Ńm�C�Y true
																					   // �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_lattice2);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// �����_���o
	std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	double elapsed;

	start = std::chrono::system_clock::now(); // �v���J�n����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal = myFeaturePointExtraction(cloud_normals2);
	end = std::chrono::system_clock::now();  // �v���I������

	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	accuracy_file << "input_cloud: " << elapsed << "[ms]" << endl;
	accuracy_file << endl;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*target_keypointsNormal, *target_keypointsXYZ);
	// �����ʌv�Z
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = myFeatureDescription_FPFH2(cloud_normals2, target_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr target_features = myFeatureDescription_SHOT352(cloud_normals2, target_keypointsXYZ);




	// �Ή��_�T��
	//std::vector<int> correspondences = myFindMatching2(source_features, target_features);
	//std::vector<int> correspondences = myFindMatching_SHOT352(source_features, target_features);
	pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF = make_lightPPFs(attention_point);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF = make_lightPPFs(attention_point2);
	int match_cloud = myLightPPF_matching(keypoint_PPF, stl_PPF);

	outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud << endl;
	outputfile777 << "stl_accuracy: " << (float)match_cloud / stl_PPF.size() * 100 << endl;
	outputfile777 << "keypoint_accuracy: " << (float)match_cloud / keypoint_PPF.size() * 100 << endl;

	outputfile777 << endl;

	// ��Ή�����
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	// R��T�̐���
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);

	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point3 = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF2 = make_lightPPFs(attention_point3);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point4 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF2 = make_lightPPFs(attention_point4);
	int match_cloud2 = myLightPPF_matching(keypoint_PPF2, stl_PPF2);

	outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF2.size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF2.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud2 << endl;
	outputfile777 << "stl_accuracy: " << (float)match_cloud2 / stl_PPF2.size() * 100 << endl;
	outputfile777 << "keypoint_accuracy: " << (float)match_cloud2 / keypoint_PPF2.size() * 100 << endl;

	outputfile777 << endl;



	/*for (int i = 0; i < correspondences.size(); i++) {
	outputfile777 << "[" << i << "]: " << correspondences[i] << endl;
	}
	outputfile777 << endl;

	for (int i = 0; i < pCorrespondences->size(); i++) {
	outputfile777 << "[" << i << "]: " << pCorrespondences->at(i) << endl;
	}
	outputfile777 << endl;*/


	for (pcl::Correspondence i : *correspondences) {
		outputfile777 << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile777 << endl;

	for (pcl::Correspondence i : *pCorrespondences) {
		outputfile777 << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile777 << endl;






	//CorrespondenceGrouping2();








	/*
	cout << endl << endl;
	init_genrand((unsigned)time(NULL));
	//for (int i = 0; i<100; i++) { printf("%ld\n", genrand_int32()); }
	for (int i = 0; i<100; i++) { printf("%lf\n", genrand_real1()); }


	cout << endl << endl;
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));
	for (int i = 0; i < 100; i++) {
		randUnitVector(sampleTest, sampleTime);
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
	}


	cout << endl << endl;
	for (int i = 0; i < 100; i++) {
		randUnitVector(sampleTest, sampleTime);
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
		cout << "length: " << sampleTest->x*sampleTest->x + sampleTest->y*sampleTest->y + sampleTest->z*sampleTest->z << endl;;
		float error = sqrt(0.04);
		//error *= genrand_real1();
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
		cout << "length: " << sampleTest->x*sampleTest->x + sampleTest->y*sampleTest->y + sampleTest->z*sampleTest->z << endl;;
	}
	*/







































	/*
	for (pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud_lattice2->begin(); iter != cloud_lattice2->end(); iter++) {
	iter->z = 0.0f;
	}


	testPCA(3, cloud_lattice2);
	cout << endl;

	array<array<double, 3>, 3> holder_PCA = cal_PCA(cloud_lattice2);
	pcl::Normal holder_normal0(holder_PCA[0][0], holder_PCA[0][1], holder_PCA[0][2]);
	cout << "x: " << holder_normal0.normal_x << "\ty: " << holder_normal0.normal_y << "\tz: " << holder_normal0.normal_z << endl;
	pcl::Normal holder_normal1(holder_PCA[1][0], holder_PCA[1][1], holder_PCA[1][2]);
	cout << "x: " << holder_normal1.normal_x << "\ty: " << holder_normal1.normal_y << "\tz: " << holder_normal1.normal_z << endl;
	pcl::Normal holder_normal2(holder_PCA[2][0], holder_PCA[2][1], holder_PCA[2][2]);
	cout << "x: " << holder_normal2.normal_x << "\ty: " << holder_normal2.normal_y << "\tz: " << holder_normal2.normal_z << endl;


	cout << endl;
	cout << "�e�X�g�F " << "test" << endl;
	pm();
	cout << endl;
	cout << endl;


	vector<array<float, 2>> testV;
	testV.reserve(cloud_lattice2->size());
	for (int i = 0; i < cloud_lattice2->size(); i++) {
	array<float, 2> xy;
	xy[0] = cloud_lattice2->at(i).x;
	xy[1] = cloud_lattice2->at(i).y;
	testV.push_back(xy);
	}

	array<array<double, 2>, 2> holder_PCA2 = cal_PCA2(testV);
	cout << "x: " << holder_PCA2[0][0] << "\ty: " << holder_PCA2[0][1] << endl;
	cout << "x: " << holder_PCA2[1][0] << "\ty: " << holder_PCA2[1][1] << endl;
	*/






















	// ���K��
	/*for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result003.txt");
	//for (int i = 0; i < Sn.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn[i] << endl;
	//}
	//outputfileNDH.close();

	vector<pair<float, int>> Sn_high;
	Sn_high.reserve((int)(Sn.size() / 10));
	for (int i = 0; i < Sn.size(); i++) {
	if (Sn[i] > 0.4) {
	Sn_high.push_back(make_pair(Sn[i], i));
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result005.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessPoints(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < Sn_high.size(); i++) {
	uniquenessPoints->push_back(surfaceN->at(Sn_high[i].second));
	}
	*/





	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointNormal>);
	pcl::VoxelGrid<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_ptr);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �O��l�̏���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_lattice);		// �O��l����������_�Q�����
	sor.setMeanK(50);				// ��������ߖT�_�̐���ݒ�
	sor.setStddevMulThresh(1.0);	// �����̕W���΍��̒l��ݒ�
	sor.setNegative(false);			// �O��l���o�͂���ꍇ��true�ɂ���
	sor.filter(*cloud_filtered);	// �o��

	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_lattice;*/

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud = cloud_ptr;

	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_ptr;



	/*
	// STL�̖@�������𗘗p���������_�̎������o
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = cloud_ptr;
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>);

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 1.0f;

	kdtree.setInputCloud(surfaceN);

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	pcl::PointNormal searchPoint = *it;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	double angle_max = 0;
	pcl::PointXYZ pN;
	pcl::PointXYZ qN;
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	for (int j = i; j < pointIdxRadiusSearch.size(); j++) {
	pcl::PointXYZ p(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	pcl::PointXYZ q(surfaceN->points[pointIdxRadiusSearch[j]].normal_x, surfaceN->points[pointIdxRadiusSearch[j]].normal_y, surfaceN->points[pointIdxRadiusSearch[j]].normal_z);
	double angle = cal_angle(p, q);
	if (angle < angle_max) continue;
	angle_max = angle;
	pN = p;
	qN = q;
	}
	}
	if (angle_max < 60) continue;
	//pcl::PointXYZ crossN = cross_product3_cal(pN, qN);
	//double angle_min = 180;
	//for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	//	pcl::PointXYZ p(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	//	double angle = cal_angle(p, crossN);
	//	angle = min(angle, 180 - angle); //
	//	if (angle > angle_min) continue;
	//	angle_min = angle;
	//}
	//if (angle_min > 60) continue;
	keypoints->push_back(searchPoint);
	}
	}
	*/


	/*double RV_x = 1.0;
	double RV_z = 0.0;

	array<float, 36> angleH;
	for (int j = 0; j < angleH.size(); j++)
	angleH[j] = 0;

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_lattice->begin(); it != cloud_lattice->end(); it++) {
	double angle_cos = (RV_x*it->normal_x) + (RV_z*it->normal_z);
	double angle_sin = (RV_x*it->normal_z) - (RV_z*it->normal_x);
	double radian = acos(angle_cos);
	double angle = radian * 180 / M_PI;
	//if (angle_sin < 0) angle += 180;
	if (angle_sin < 0) angle = 360 - angle;
	angle /= 10;
	int indexH = (int)angle;
	angleH[indexH]++;
	}

	for (int j = 0; j < angleH.size(); j++)
	cout << "angleH[" << j << "]: " << angleH[j] << endl;
	*/



	/*
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);



	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	*/


	// ����f�[�^�ǂݍ��݁i�z���_�[�ʁj�i�@���x�N�g�����H��p���Ɠ����ɂȂ�O�p�`�|���S���̒��o�j
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);

	if (angle_cos < theta_cos) { // �p�x���傫�����cos�Ƃ͏������Ȃ�
	continue;
	}

	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	//////// �z���_�[�� ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;*/


	// ����f�[�^�ǂݍ��݁i�z���_�[�ʁj�i�@���x�N�g�����H��p���Ɠ����ɂȂ�O�p�`�|���S���̒��o�j
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);
	if (angle_cos < theta_cos) { // �p�x���傫�����cos�Ƃ͏������Ȃ�
	continue;
	}
	pcl::PointXYZ point;
	point.x = tri.vertex[0].x;
	point.y = tri.vertex[0].y;
	point.z = tri.vertex[0].z;
	//////// �z���_�[�� ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;



	// �z���_�[�ʂ̌��o�iRANSAC�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_wall(holder_face, false);*/


	// �z���_�[�ʂ̌��o�i�ʎq���ɂ��Y�l��r�j
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y�l�̗ʎq��
	int offset_i = 27*10;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = holder_face->begin(); it != holder_face->end(); it++) {
	float y_i = it->y;
	y_i *= 10;
	y_i = round(y_i);
	int index = y_i + offset_i;
	sample_vec0[index].push_back(*it);
	}
	int max = -1;
	int max_i = 0;
	for (int i = 0; i < 21; i++) {
	int size = sample_vec0[i].size();
	//cout << "sample_vec[" << i << "].size(): " << size << endl;
	if (size > max) {
	max = size;
	max_i = i;
	}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = sample_vec0[max_i].makeShared();
	//cout << "max_i: " << max_i << endl;*/


	// �z���_�[�ʂ̌��o�iY�l�ɂ��O���[�s���O�j �� ������
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y�l�̗ʎq��
	int offset_i = 27 * 10;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = holder_face->begin(); it != holder_face->end(); it++) {
	float y_i = it->y;
	y_i *= 10;
	y_i = round(y_i);
	int index = y_i + offset_i;
	sample_vec0[index].push_back(*it);
	}
	int max = -1;
	int max_i = 0;
	for (int i = 0; i < 21; i++) {
	int size = sample_vec0[i].size();
	if (size > max) {
	max = size;
	max_i = i;
	}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = sample_vec0[max_i].makeShared();
	//cout << "max_i: " << max_i << endl;*/



	// �z���_�[�ʂ̌���i�ŏ����@�j
	/*array<array<double, 3>, 3> holder_PCA = cal_PCA(sample_vec);
	pcl::Normal holder_normal(holder_PCA[0][0], holder_PCA[0][1], holder_PCA[0][2]);
	//cout << "x: " << holder_normal.normal_x << "\ty: " << holder_normal.normal_y << "\tz: " << holder_normal.normal_z << endl;



	// �H���[�_�̒��o
	priority_queue <pcl::PointXYZ, vector<pcl::PointXYZ>, PointYCompareMin> minpq;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
	minpq.push(*it);
	}
	//cout << "minpq" << endl;
	pcl::PointXYZ min_value = minpq.top();
	//cout << "x: " << min_value.x << "\ty: " << min_value.y << "\tz: " << min_value.z << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tip_point(new pcl::PointCloud<pcl::PointXYZ>);
	tip_point->push_back(min_value);



	// �_�Ɩʂ̋����Z�o
	pcl::PointXYZ A = sample_vec->at(0);
	float coef_d = -(holder_normal.normal_x*A.x + holder_normal.normal_y*A.y + holder_normal.normal_z*A.z);
	float error_d = -(holder_normal.normal_x*min_value.x + holder_normal.normal_y*min_value.y + holder_normal.normal_z*min_value.z);
	float distance = fabsf(coef_d - error_d);
	cout << "distance: " << distance << endl;*/


	/*
	// �ȗ�
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr keypointsCurvature(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	if (it->curvature < 0.01) continue; //0.05
	//cout << "it->curvature: " <<it->curvature << endl;
	keypointsCurvature->push_back(*it);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	cout << "keypointsCurvature->points.size(): " << keypointsCurvature->points.size() << endl;


	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);

	priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending> cQueue;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = keypointsCurvature->begin(); it != keypointsCurvature->end(); it++) {
	cQueue.push(*it);
	}

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	kdtree.setInputCloud(keypointsCurvature);
	pcl::PointNormal searchPoint = cQueue.top();
	cout << "searchPoint.curvature: " << searchPoint.curvature << endl;

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	keypointsCurvature->erase(keypointsCurvature->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	attention_point->push_back(searchPoint);

	while (!keypointsCurvature->empty()) {
	cQueue = priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending>();
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = keypointsCurvature->begin(); it != keypointsCurvature->end(); it++) {
	cQueue.push(*it);
	}
	kdtree.setInputCloud(keypointsCurvature);
	searchPoint = cQueue.top();
	cout << "searchPoint.curvature: " << searchPoint.curvature << endl;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	keypointsCurvature->erase(keypointsCurvature->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	attention_point->push_back(searchPoint);
	}


	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(cloud, false);

	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	*/



	/*
	// RANSAC�ɂ��`�󕪗�
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN2 = surface_normals(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN(new pcl::PointCloud<pcl::PointNormal>());
	surfaceN->reserve(surfaceN2->size());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN2->begin(); it != surfaceN2->end(); it++) {
	if (isnan(it->normal_x)) continue;
	surfaceN->push_back(*it);
	}
	*/

	/*ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result001.txt");
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	outputfileNDH << "it->x: " << it->x << endl;
	outputfileNDH << "it->y: " << it->y << endl;
	outputfileNDH << "it->z: " << it->z << endl;
	outputfileNDH << "it->normal_x: " << it->normal_x << endl;
	outputfileNDH << "it->normal_y: " << it->normal_y << endl;
	outputfileNDH << "it->normal_z: " << it->normal_z << endl;
	}
	outputfileNDH.close();*/




	/*
	// �Ǐ��`��̓Ǝ����ɒ��ڂ������̔F���ɗL����3-D�����_�̎������o
	//pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	std::vector<array<float, 18>> NDH;
	NDH.reserve(surfaceN->size());

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	kdtree.setInputCloud(surfaceN);

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	pcl::PointNormal searchPoint = *it;
	array<float, 18> angleH;
	for (int j = 0; j < angleH.size(); j++)
	angleH[j] = 0;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	pcl::PointXYZ p(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z);
	pcl::PointXYZ q(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	double angle = cal_angle(p, q);
	//if (isnan(angle)) continue; //
	angle /= 10;
	int indexH = (int)angle;
	//cout << "[" << i << "]:\t" << "angle: " << angle << "\tindexH: " << indexH << endl;
	angleH[indexH]++;
	}
	}
	NDH.push_back(angleH);
	}

	// ���K��
	for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result003.txt");
	//for (int i = 0; i < Sn.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn[i] << endl;
	//}
	//outputfileNDH.close();

	vector<pair<float, int>> Sn_high;
	Sn_high.reserve((int)(Sn.size() / 10));
	for (int i = 0; i < Sn.size(); i++) {
	if (Sn[i] > 0.4) {
	Sn_high.push_back(make_pair(Sn[i], i));
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result005.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessPoints(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < Sn_high.size(); i++) {
	uniquenessPoints->push_back(surfaceN->at(Sn_high[i].second));
	}





	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(cloud, false);

	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	cout << "surfaceN: " << surfaceN->size() << endl;
	cout << "cloud: " << cloud->size() << endl;
	*/






	// �����_����
	/*pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	//detector.setNonMaxSupression(false);
	detector.setRadius(3.0);//3.0 5.0 7.0
	detector.setInputCloud(cloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);

	//std::cout << "keypoints detected: " << keypoints->size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
	tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
	//if ((*i).intensity>max) {
	//std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
	//max = (*i).intensity;
	//}
	//if ((*i).intensity<min) {
	//min = (*i).intensity;
	//}
	keypoints3D->push_back(tmp);
	}
	//std::cout << "maximal responce: " << max << " min responce:  " << min << std::endl;



	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(cloud);
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/
	// average �����_�ߖT�̏d�S
	/*pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointXYZ cloud_average = cal_cloud_average(neighborhood_cloud);
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/



	// ����f�[�^��PPF
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(keypoint_normals);
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs2(keypoint_normals);
	//outputfile777 << "keypoint_cloud->size(): " << keypoint_cloud->size() << endl;
	//outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;*/
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);



	// �H��f�[�^�ǂݍ��݁i�f�[�^�}�b�`���O�j
	string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative0.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative2.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative3.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative4.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative5.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative6.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative7.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative8.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative9.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DCLNR3225P12.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR1616H11.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR2020K15.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DVJNR2020K16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M16N.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M22N.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PWLNR2525M06.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SDNCR0808K07-SM.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SRGCR1616H08.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SSSCR1212F09.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVPPR1616M11-SM.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVVCN1616H16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SXZCR2020K15.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\TLHR2020K5.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0707-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0808-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0909-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808-60.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010-60.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212-60.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2000S25.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2500S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA3000S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS3000S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2500X2F25 - MVX2500X2F25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX3000X2F32 - MVX3000X2F32-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2500S32 - TAWSN2500S32-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN3000S32 - TAWSN3000S32-1.STL";
	data_matching(path, keypoint_PPF);*/



	//string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	//string path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = stl_points(path);


	//string path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	/*
	// �_���Ԉ�������
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud2 = stl_points(path);
	// Neighbors within radius search
	while (!stl_cloud2->empty()) {
	kdtree.setInputCloud(stl_cloud2);
	searchPoint = stl_cloud2->at(0);
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	stl_cloud2->erase(stl_cloud2->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "stl_cloud2->size(): " << stl_cloud2->size() << endl;
	stl_cloud->push_back(searchPoint);
	}
	*/

	/*
	// ���ʍ������
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud2 = stl_points(path);
	// Neighbors within radius search
	kdtree.setInputCloud(stl_cloud2);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = stl_cloud2->begin(); it != stl_cloud2->end(); it++) {
	searchPoint = *it;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 10) {
	stl_cloud->push_back(searchPoint);
	}
	}
	*/



	// �_�Q�f�[�^
	/*start = std::chrono::system_clock::now(); // �v���J�n����
	const string filename = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = voxel_grid(cloud_in);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = myifstream_test(filename);
	outputfile777 << "cloud_in->size(): " << cloud_in->size() << endl;
	outputfile777 << endl;
	end = std::chrono::system_clock::now();  // �v���I������
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/


	////////////////////////////////////////////

	//outputfile.close();
	outputfile777.close();


	// ���͓_�Q�Ɩ@����\��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(stl_cloud, 0, 255, 0);
	//viewer->addPointCloud(stl_cloud, stl_color, "stl_cloud");
	//viewer->addPointCloudNormals<pcl::PointNormal>(stl_cloud, 1, 3.0, "stl_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "stl_cloud");

	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(surfaceN, 0, 255, 0);
	//viewer->addPointCloud(surfaceN, stl_color, "stl_cloud");
	//viewer->addPointCloudNormals<pcl::PointNormal>(surfaceN, 1, 3.0, "stl_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "stl_cloud");


	// point cloud
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(cloud, 255, 255, 255);
	//viewer->addPointCloud(cloud, pccolor, "point_cloud");
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(keypoints3D, 255, 0, 0);
	viewer->addPointCloud(keypoints3D, kpcolor, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	viewer->addPointCloudNormals<pcl::PointNormal>(keypoint_normals, 1, 7.0, "keypoint_normals");*/


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_holder(holder_face, 255, 255, 255);
	//viewer->addPointCloud(holder_face, pccolor_in, "holder_face");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_in(cloud_in, 255, 255, 255);
	//viewer->addPointCloud(cloud_in, pccolor_in, "cloud_in");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC(sample_vec, 0, 0, 255);
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pccolor_SAC(sample_vec, 0, 0, 255);
	viewer->addPointCloud(sample_vec, pccolor_SAC, "sample_vec");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC2(sample_vec2, 0, 255, 0);
	viewer->addPointCloud(sample_vec2, pccolor_SAC2, "sample_vec2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec2");*/

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tip_color(tip_point, 255, 0, 0);
	//viewer->addPointCloud(tip_point, tip_color, "tip_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "tip_point");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> attention_color(attention_point, 255, 0, 0);
	//viewer->addPointCloud(attention_point, attention_color, "attention_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PrincipalCurvatures> pccolor_C(principal_curvatures, 255, 255, 255);
	//viewer->addPointCloud(principal_curvatures, pccolor_C, "principal_curvatures");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(keypointsCurvature, 255, 0, 0);
	//viewer->addPointCloud(keypointsCurvature, kccolor, "keypointsCurvature");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(keypointsCurvature, 255, 0, 0);
	//viewer->addPointCloud(keypointsCurvature, kccolor, "keypointsCurvature");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypointsCurvature");


	/*int vp[2];
	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(attention_point2);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp[1]);
	viewer->addSphere(attention_point2->points[10], 3.0, 0.0, 0.0, 0.0, "sphere");*/

	/*int vp;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp);
	viewer->addSphere(attention_point2->points[10], 3.0, 0.0, 0.5, 0.5, "sphere", vp);*/




	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lattice_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, lattice_color, "cloud_lattice2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice2");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloudN_color(cloud_normals3, 255, 255, 255);
	//viewer->addPointCloud(cloud_normals3, cloudN_color, "cloud_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_normals");



	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(surfaceN, 255, 255, 255);
	//viewer->addPointCloud(surfaceN, stl_color, "stl_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stl_cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color(attention_point, 255, 0, 0);
	//viewer->addPointCloud(attention_point, attention_color, "attention_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");
	//viewer->addPointCloudNormals<pcl::PointNormal>(attention_point, 1, 7.0, "attention_pointn");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color2(surfaceN2, 255, 255, 255);
	//viewer->addPointCloud(surfaceN2, stl_color2, "stl_cloud2");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stl_cloud2");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color2(attention_point2, 0, 255, 0);
	viewer->addPointCloud(attention_point2, attention_color2, "attention_point2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point2");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_point2n");













	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(cloud, 0, 255, 0);
	//viewer->addPointCloud(cloud, pccolor, "point_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "point_cloud");


	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(cloud, 255, 255, 255);
	viewer->addPointCloud(cloud, kccolor, "cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pccolor_SAC(sample_vec, 0, 0, 255);
	viewer->addPointCloud(sample_vec, pccolor_SAC, "sample_vec");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC2(sample_vec2, 255, 0, 0);
	viewer->addPointCloud(sample_vec2, pccolor_SAC2, "sample_vec2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec2");*/


	/*pcl::PointCloud<pcl::PointXYZ>::Ptr surface0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr surfaceN0(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*surfaceN, *surface0);
	pcl::copyPointCloud(*surfaceN, *surfaceN0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(surface0, 255, 255, 255);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(surface0, surfaceN0, 2, 0.5, "surface0");*/

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(surfaceN, 255, 255, 255);
	viewer->addPointCloudNormals<pcl::PointNormal>(surfaceN, 1, 0.1, "surfaceN");*/



	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(surfaceN, 255, 255, 255);
	//viewer->addPointCloud(surfaceN, kccolor, "surfaceN");

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> uni_color(uniquenessPoints, 255, 0, 0);
	viewer->addPointCloud(uniquenessPoints, uni_color, "uniquenessPoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "uniquenessPoints");*/



	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kp_color(keypoints, 255, 0, 0);
	//viewer->addPointCloud(keypoints, kp_color, "keypoints");
	//viewer->addPointCloudNormals<pcl::PointNormal>(keypoints, 1, 3.0, "kp_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");




	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}