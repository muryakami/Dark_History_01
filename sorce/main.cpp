#include "main.h"


float dot_product3_cal(pcl::PointXYZ p, pcl::PointXYZ q);
pcl::PointCloud<pcl::PointNormal>::Ptr make_occlusion(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f);//視点の決定

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		pcl::PointXYZ n(it->normal_x, it->normal_y, it->normal_z);
		pcl::PointXYZ vppi(view_point.x - it->x, view_point.y - it->y, view_point.z - it->z);
		if (dot_product3_cal(n, vppi) < 0) continue; // 正しい
		//if (dot_product3_cal(n, vppi) > 0) continue; // 誤り
		cloud_normals->push_back(*it);
	}

	return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>()); //近傍点配列

	float r = 7.0f; //半径r 5.0f
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) { //全体の点群を1点づつ抽出
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		float judge = euclidean_distance(tmp, attention_point); //normを計算
		if (judge < r*r) { //半径rよりも近かったら近傍点配列に追加
			neighborhood->push_back(tmp);
			continue;
		}
	}
	return neighborhood;
}

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

inline float error() {
	return (rand() % 21 - 10)*0.01 * 3; // 3
	//return (rand() % 21 - 10)*0.01 * 0;
}

// 三角形の要素内補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_triangle(TRIANGLE tri, bool TF) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// 点情報
	pcl::PointXYZ A;
	A.x = tri.vertex[0].x;
	A.y = tri.vertex[0].y;
	A.z = tri.vertex[0].z;
	pcl::PointXYZ B;
	B.x = tri.vertex[1].x;
	B.y = tri.vertex[1].y;
	B.z = tri.vertex[1].z;
	pcl::PointXYZ C;
	C.x = tri.vertex[2].x;
	C.y = tri.vertex[2].y;
	C.z = tri.vertex[2].z;

	// 基底ベクトル
	pcl::PointXYZ eu;
	eu.x = B.x - A.x;
	eu.y = B.y - A.y;
	eu.z = B.z - A.z;
	float eu_norm = sqrt(dot_product3_cal(eu, eu));

	pcl::PointXYZ ev;
	ev.x = C.x - A.x;
	ev.y = C.y - A.y;
	ev.z = C.z - A.z;
	float ev_norm = sqrt(dot_product3_cal(ev, ev));

	/*for (float up = 0; up <= 1; up += 0.05) {
	for (float vp = 0; vp <= (1 - up); vp += 0.05) {
	pcl::PointXYZ p;
	p.x = A.x + up*eu.x + vp*ev.x;
	p.y = A.y + up*eu.y + vp*ev.y;
	p.z = A.z + up*eu.z + vp*ev.z;
	cloud_ptr->push_back(p);
	}
	}*/

	for (float up = 0; up <= 1 * eu_norm; up += 1) {
		for (float vp = 0; vp <= (1 - up / eu_norm)*ev_norm; vp += 1) {
			pcl::PointXYZ p;
			p.x = A.x + up*(eu.x / eu_norm) + vp*(ev.x / ev_norm);
			p.y = A.y + up*(eu.y / eu_norm) + vp*(ev.y / ev_norm);
			p.z = A.z + up*(eu.z / eu_norm) + vp*(ev.z / ev_norm);
			if (TF) {
				p.x += error();
				p.y += error();
				p.z += error();
			}

			//if (p.z < 100) continue; //
			//if (p.z > 67) continue; //

			cloud_ptr->push_back(p);
		}
	}

	return cloud_ptr;
}

// STLデータの内挿補間
pcl::PointCloud<pcl::PointXYZ>::Ptr interpolation_stl(string filename, bool TF) {

	// 点群情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	// データ読み込み
	STLDATA stl_object(filename);

	// 内挿補間
	pcl::PointCloud<pcl::PointXYZ> merged_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr triangle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	merged_cloud = *triangle_cloud;
	for (int i = 0; i<stl_object.getDatanum(); i++) {
		TRIANGLE tri;
		stl_object.getData(i, &tri);
		merged_cloud += *interpolation_triangle(tri, TF);
	}

	// ダウンサンプリング
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(merged_cloud.makeShared());
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_ptr);

	return cloud_ptr;
}

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
	double distance;				// 距離 int
	double angle_between;			// m1とm2がなす角 int
	double angle_n1;				// m1がなす角 int
	double angle_n2;				// m2がなす角 int
	bool operator < (const _myPPF& rhs) const {
		// distance比較
		if (distance < rhs.distance) return true;
		if (distance > rhs.distance) return false;
		// angle_between比較（distance==rhs.distanceの場合）
		if (angle_between < rhs.angle_between) return true;
		if (angle_between > rhs.angle_between) return false;
		// angle_n1比較（angle_between==rhs.angle_betweenの場合）
		if (angle_n1 < rhs.angle_n1) return true;
		if (angle_n1 > rhs.angle_n1) return false;
		// angle_n2比較（angle_n1==rhs.angle_n1の場合）
		if (angle_n2 < rhs.angle_n2) return true;
		if (angle_n2 > rhs.angle_n2) return false;
		return false;
	}
} myPPF;


// 通常PPF
vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> make_PPFs(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	float distance_resolution = 3.0f; // 分解能(量子化の区切り)
	float angle_resolution = 10.0f; // 分解能(量子化の区切り)

	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			if (i == j) continue;
			pcl::PointXYZ m2(j->x, j->y, j->z);

			myPPF m1m2;

			// 距離
			double point_distance = sqrt(euclidean_distance(m1, m2));
			point_distance /= distance_resolution; // 量子化
			point_distance = (int)point_distance;
			point_distance *= distance_resolution;
			m1m2.distance = point_distance;

			// 角度
			pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
			pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
			double angle0 = cal_angle(m1_normal, m2_normal);
			angle0 /= angle_resolution; // 量子化
			angle0 = (int)angle0;
			angle0 *= angle_resolution;
			m1m2.angle_between = angle0;

			// 角度
			pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
			double angle1 = cal_angle(m1_m2, m1_normal);
			double angle10 = angle1 / angle_resolution; // 量子化
			angle10 = (int)angle10;
			angle10 *= angle_resolution;

			// 角度
			pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
			double angle2 = cal_angle(m2_m1, m2_normal);
			double angle20 = angle2 / angle_resolution; // 量子化
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


// 二重登録なし
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
				//point_distance = round10(point_distance); // 量子化
				point_distance /= 100;
				point_distance = round(point_distance);
				point_distance *= 100;
				m1m2.distance = (int)point_distance;

				pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
				pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
				double angle0 = cal_angle(m1_normal, m2_normal);
				angle0 = round10(angle0); // 量子化
				m1m2.angle_between = angle0;

				pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
				double angle1 = cal_angle(m2_m1, m1_normal);
				double angle10 = round10(angle1); // 量子化

				pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
				double angle2 = cal_angle(m1_m2, m2_normal);
				double angle20 = round10(angle2); // 量子化	

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

// 二重登録あり
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
				//point_distance = round10(point_distance); // 量子化
				point_distance /= 100;
				point_distance = round(point_distance);
				point_distance *= 100;
				m1m2.distance = (int)point_distance;

				pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
				pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
				double angle0 = cal_angle(m1_normal, m2_normal);
				angle0 = round10(angle0); // 量子化
				m1m2.angle_between = angle0;

				pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
				double angle1 = cal_angle(m2_m1, m1_normal);
				double angle10 = round10(angle1); // 量子化

				pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
				double angle2 = cal_angle(m1_m2, m2_normal);
				double angle20 = round10(angle2); // 量子化	

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
	// 測定データの特徴点のPPFをマルチマップに格納
	std::multimap<myPPF, std::pair<pcl::PointXYZ, pcl::PointXYZ>> keypoint_map;
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = keypoint_PPF.begin(); i != keypoint_PPF.end(); i++) {
	keypoint_map.insert(*i);
	}
	// STLデータの特徴点のPPFをマルチマップから検索(重複計算あり)←あとで重複を取り除く必要あり 一対多の対応(正対応，誤対応の処理で取り除ける)
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr result(new pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>());
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
	auto p = keypoint_map.equal_range(i->first);
	for (auto it = p.first; it != p.second; ++it) {
	result->push_back(make_pair(i->second, it->second));
	}
	}
	*/

	// 測定データの特徴点のPPFをマップに格納
	std::map<myPPF, std::pair<pcl::PointXYZ, pcl::PointXYZ>> keypoint_map;
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = keypoint_PPF.begin(); i != keypoint_PPF.end(); i++) {
		keypoint_map.insert(*i);
	}
	// STLデータの特徴点のPPFをマップから検索(重複計算なし)←正確でない 一対一の対応
	pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr result(new pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>());
	for (vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>>::iterator i = stl_PPF.begin(); i != stl_PPF.end(); i++) {
		auto p = keypoint_map.equal_range(i->first);
		for (auto it = p.first; it != p.second; ++it) {
			result->push_back(make_pair(i->second, it->second));
		}
	}

	return result;
}

// 通常PPF 簡易版
vector<myPPF> make_lightPPFs(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
	vector<myPPF> PPFs;
	const int cloud_size = cloud->size() * cloud->size();
	PPFs.reserve(cloud_size);

	//float distance_resolution = 1.0f; // 分解能(量子化の区切り) 3.0f 2.0f
	//float angle_resolution = 1.0f; // 分解能(量子化の区切り) 30.0f

	for (pcl::PointCloud<pcl::PointNormal>::iterator i = cloud->begin(); i != cloud->end(); i++) {
		pcl::PointXYZ m1(i->x, i->y, i->z);
		pcl::PointXYZ m1_normal(i->normal_x, i->normal_y, i->normal_z);
		for (pcl::PointCloud<pcl::PointNormal>::iterator j = i; j != cloud->end(); j++) {
			if (i == j) continue;
			pcl::PointXYZ m2(j->x, j->y, j->z);
			pcl::PointXYZ m2_normal(j->normal_x, j->normal_y, j->normal_z);
			myPPF m1m2;

			// 距離
			double point_distance = sqrt(euclidean_distance(m1, m2));
			//point_distance /= distance_resolution; // 量子化
			//point_distance = (int)point_distance;
			//point_distance *= distance_resolution;
			m1m2.distance = point_distance;

			// 角度
			double angle0 = cal_angle(m1_normal, m2_normal);
			//angle0 /= angle_resolution; // 量子化
			//angle0 = (int)angle0;
			//angle0 *= angle_resolution;
			m1m2.angle_between = angle0;

			// 角度
			pcl::PointXYZ m1_m2(m2.x - m1.x, m2.y - m1.y, m2.z - m1.z);
			double angle1 = cal_angle(m1_m2, m1_normal);
			//double angle10 = angle1 / angle_resolution; // 量子化
			//angle10 = (int)angle10;
			//angle10 *= angle_resolution;

			// 角度
			pcl::PointXYZ m2_m1(m1.x - m2.x, m1.y - m2.y, m1.z - m2.z);
			double angle2 = cal_angle(m2_m1, m2_normal);
			//double angle20 = angle2 / angle_resolution; // 量子化
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

// 通常PPF 簡易版　のマッチング
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



typedef struct _myQuaternion {
	float angle;
	pcl::PointXYZ axis;
	bool operator < (const _myQuaternion& rhs) const {
		// angle比較
		if (angle < rhs.angle) return true;
		if (angle > rhs.angle) return false;
		// axis_x比較
		if (axis.x < rhs.axis.x) return true;
		if (axis.x > rhs.axis.x) return false;
		// axis_y比較
		if (axis.y < rhs.axis.y) return true;
		if (axis.y > rhs.axis.y) return false;
		// axis_z比較
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
		quaternion.angle = quaternion.angle * 180 / M_PI;	// 度
		quaternion.angle = round10(quaternion.angle);		// 量子化

		pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
		float norm = sqrt(euclidean_distance(quaternion.axis, origin));
		quaternion.axis.x /= norm;	// 正規化
		quaternion.axis.y /= norm;
		quaternion.axis.z /= norm;
		quaternion.axis.x = round01(quaternion.axis.x);	// 量子化
		quaternion.axis.y = round01(quaternion.axis.y);
		quaternion.axis.z = round01(quaternion.axis.z);

		auto itr = rotationHistogram.find(quaternion);	// quaternion が設定されているか？
		if (itr != rotationHistogram.end()) {
			//設定されている場合の処理
			itr->second++;
		}
		else {
			//設定されていない場合の処理
			rotationHistogram.emplace(quaternion, 1);
		}
	}

	int max_i = -1;
	myQuaternion max_quaternion; // 回転軸と回転角度　工具姿勢がわかる
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
		angle = angle * 180 / M_PI;	// 度
		angle = round10(angle);		// 量子化

		auto itr = rotationHistogram.find(angle);	// angle が設定されているか？
		if (itr != rotationHistogram.end()) {
			//設定されている場合の処理
			itr->second++;
		}
		else {
			//設定されていない場合の処理
			rotationHistogram.emplace(angle, 1);
		}
	}

	int max_i = -1;
	float max_angle; // 回転軸と回転角度　工具姿勢がわかる
	for (map<float, int>::iterator it = rotationHistogram.begin(); it != rotationHistogram.end(); it++) {
		if (max_i < it->second) {
			max_i = it->second;
			max_angle = it->first;
		}
	}

	return max_i;
}

float bhattacharyya(array<float, 18> p, array<float, 18> q) {
	float result = 0;
	for (int i = 0; i < 18; i++) {
		result += p[i] * q[i];
	}
	return sqrt(result);
}

float uniqueness(array<float, 18> n, vector<array<float, 18>> NDH) {
	float result = 0;
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result += (1 - bhattacharyya(n, *it));
	}
	result /= NDH.size();
	return result;
}

vector<float> all_uniqueness(vector<array<float, 18>> NDH) {
	vector<float> result;
	result.reserve(NDH.size());
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result.push_back(uniqueness(*it, NDH));
	}
	return result;
}


float bhattacharyya36(array<float, 36> p, array<float, 36> q) {
	float result = 0;
	for (int i = 0; i < 36; i++) {
		result += p[i] * q[i];
	}
	return result;
	//return sqrt(result);
}

float uniqueness36(array<float, 36> n, vector<array<float, 36>> NDH) {
	float result = 0;
	for (vector<array<float, 36>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result += bhattacharyya36(n, *it);
	}
	result /= NDH.size();
	return result;
}

vector<float> all_uniqueness36(vector<array<float, 36>> NDH) {
	vector<float> result;
	result.reserve(NDH.size());
	for (vector<array<float, 36>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result.push_back(1 - uniqueness36(*it, NDH));
	}
	return result;
}



void data_matching(string path, vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF) {
	// 工具データ読み込み
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


	// 工具データのPPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF = make_PPFs(stl_cloud);
	outputfile777 << "stl_cloud->size(): " << stl_cloud->size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;

	// データマッチング
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
	// 工具データ読み込み
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
	// 特徴点生成
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
	// 特徴点の法線生成
	// average 工具全体の重心
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypoints3D->size(); i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypoints3D->at(i));
		pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
		stl_cloud->push_back(normal);
	}




	// 工具データのPPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> stl_PPF = make_PPFs(stl_cloud);
	outputfile777 << "stl_cloud->size(): " << stl_cloud->size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;

	// データマッチング
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
	// 工具データ読み込み
	outputfile777 << path << endl;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(path, false);

	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud_lattice);

	// 特徴点の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(surfaceN);

	// 工具データのPPF
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
	sor.setInputCloud(cloud);		// 外れ値を除去する点群を入力
	sor.setMeanK(50);				// 検索する近傍点の数を設定
	sor.setStddevMulThresh(1.0);	// 距離の標準偏差の値を設定
	sor.setNegative(TF);			// 外れ値を出力する場合はtrueにする
	sor.filter(*cloud_filtered);	// 出力
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
	seg.setModelType(pcl::SACMODEL_PLANE);	// 検出するモデルのタイプを指定
	seg.setMethodType(pcl::SAC_RANSAC);		// 検出に使用する方法を指定
	seg.setDistanceThreshold(0.10);			// 閾値 0.01, 0.05
											//seg.setDistanceThreshold(0.10);			// 閾値 0.01, 0.05

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(TF);//Trueの場合出力は検出された壁以外のデータ falseの場合は壁のデータ
	extract.filter(*cloud_output);

	return cloud_output;//出力
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
	seg.setNormalDistanceWeight(0.087); //0.01  0.087=sin5度 0.174=sin10度
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(1.00); //0.05	0.10
	seg.setRadiusLimits(0, 30.0); // 0,0.1		0, 20.0
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	//seg.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f)); // 任意軸？(自分で設定) 実験的に追加


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

	return cloud_cylinder;//出力
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



ofstream outputfile("C:\\Users\\yuki\\Documents\\進捗報告\\02\\outputfile\\test001.txt");

// 極形式ヒストグラムの計算
std::vector<array<float, 36>> myPolarHistograms36(pcl::PointCloud<pcl::PointNormal>::Ptr searchPoints, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	std::vector<array<float, 36>> PHs;
	PHs.reserve(cloud->size());

	float radius = 3.0f;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = searchPoints->begin(); it != searchPoints->end(); it++) {
		pcl::PointNormal searchPoint = *it;
		vector<pair<int, float>> neighbors_inf = radius_search_inf(searchPoint, radius, cloud);
		pcl::PointCloud<pcl::PointNormal>::Ptr neighbors(new pcl::PointCloud<pcl::PointNormal>);

		// 近傍点情報から近傍点の配列を作成
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			neighbors->push_back(cloud->points[itr->first]);
		}

		// 正射影のための準備(注目点の法線を回転，z軸に合わせるマトリクス)
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



		// 正射影(近傍点の法線を回転)
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

		// 正射影(近傍点の法線を回転)
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



		// 各点における最近傍点までの距離の総和(重み係数w1の計算で使用)
		int K = 2;
		float nearestSum = 0;
		for (pcl::PointCloud<pcl::PointNormal>::iterator itr = neighbors->begin(); itr != neighbors->end(); itr++) {
			vector<pair<int, float>> nearestP = nearest_search1(*itr, K, neighbors);
			for (vector<pair<int, float>>::iterator iter = nearestP.begin(); iter != nearestP.end(); iter++) {
				nearestSum += iter->second;
			}
		}


		// 極形式ヒストグラムの初期化
		array<float, 36> angleH;
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] = 0;

		// ヒストグラムの正規化用
		float sum_total = 0;
		// 極形式ヒストグラムの作成
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

		// 極形式ヒストグラムの正規化
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] /= sum_total;

		// 極形式ヒストグラムにおけるピークの格納位置を調整(最初のビンに統一)
		std::array<float, 36>::iterator maxIt = std::max_element(angleH.begin(), angleH.end());
		int maxIndex = std::distance(angleH.begin(), maxIt);
		array<float, 36> tempH;
		for (int j = 0; j < tempH.size(); j++)
			tempH[j] = angleH[j];
		for (int j = 0; j < angleH.size(); j++) {
			int tempIndex = (maxIndex - j + 36) % 36;
			angleH[j] = tempH[tempIndex];
		}

		// 極形式ヒストグラムの配列に格納
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


// 局所形状の独自性に着目した物体認識に有効な3-D特徴点の自動抽出
pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtraction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\06\\outputfile\\result004.txt");

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 曲率が高い点の抽出
	//pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints = cloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		if (it->curvature < 0.03) continue;
		highCurvaturePoints->push_back(*it);
	}

	//return highCurvaturePoints; // 高曲率点群の確認

	// 極形式ヒストグラムの算出
	std::vector<array<float, 36>> PHs = myPolarHistograms36(highCurvaturePoints, cloud);

	// 独自性指標の算出
	vector<float> Sn = all_uniqueness36(PHs);

	// 曲率領域に独自性指標の値を代入
	for (int i = 0; i < highCurvaturePoints->size(); i++) {
		highCurvaturePoints->at(i).curvature = Sn[i];
	}

	// 独自性指標の値で決定
	/*
	// 独自性指標が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
	if (it->curvature < 0.9) continue; //0.85
	stl_cloud->push_back(*it);
	}

	// 特徴点の決定(球領域探索)
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


	// 点の数で決定

	// 特徴点の決定(球領域探索)
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

	// 工具データ情報
	outputfile777 << filename << endl;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// 特徴点検出，特徴量計算
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals);
	// 工具データのPPF
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);


	// データマッチング (簡易版)
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "match_cloud->size: " << match_cloud << endl;
	outputfile777 << "accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;

	outputfile777 << endl;

	//return source_PPF; //
}



pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtraction_Harris(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud);

	// 曲率が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_normals->begin(); it != cloud_normals->end(); it++) {
		if (it->curvature < 0.03) continue;
		highCurvaturePoints->push_back(*it);
	}

	// 曲率が高い点の座標情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*highCurvaturePoints, *cloud_points);
	for (int i = 0; i < 100; i++) {
		cout << "PointXYZ: " << cloud_points->at(i).x << " " << cloud_points->at(i).y << " " << cloud_points->at(i).z << endl;
	}


	/////////////
	cloud_points = cloud;
	/////////////


	// 特徴点生成
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	detector.setRadius(3.0);//3.0 5.0 7.0
	detector.setThreshold(0.005f); // 旋削工具
								   //detector.setThreshold(1e-6); // 回転工具
	detector.setInputCloud(cloud_points);
	//detector.setSearchSurface(ダウンサンプリングされる前の点群);
	detector.setNonMaxSupression(true);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		keypoints3D->push_back(tmp);
	}

	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypoints3D->begin(); it != keypoints3D->end(); it++) {
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

/*class PointYCompareIntensity { //昇順
public:
bool operator()(const pcl::PointXYZI &left, const pcl::PointXYZI &right) const {
if (left.intensity < right.intensity) return true;
if (left.intensity > right.intensity) return false;
return false;
}
};*/
bool compare_intensity(const pcl::PointXYZI &left, const pcl::PointXYZI &right) { //降順
	return left.intensity > right.intensity;
}
pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtraction_HarrisN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());

	// パラメータ
	float radius = 0.3f; // 3.0f 5.0f 7.0f		3.0f 0.3f
	float threshhold = 1e-6; // 1e-6 0.005f			1e-12 1e-6

	/*
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud);

	// 特徴点検出
	std::cout << "detection" << std::endl;
	pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::Ptr detector(new pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>());
	detector->setNonMaxSupression(true);
	detector->setRadius(radius);
	//detector->setRadiusSearch(radius);
	detector->setThreshold(threshhold);
	detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::HARRIS); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE

	detector->setInputCloud(cloud_normals);
	detector->setSearchSurface(cloud_normals);
	detector->compute(*keypoints);
	cout << "number of source keypoints found: " << keypoints->points.size() << endl;

	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);
	*/


							 // 特徴点抽出
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
	detector.setRadius(radius);
	detector.setThreshold(threshhold);
	detector.setNonMaxSupression(true);

	detector.setInputCloud(cloud);
	//detector.setSearchSurface(ダウンサンプリングされる前の点群);
	detector.compute(*keypoints);

	//keypointsXYZ->points.resize(keypoints->points.size());
	//pcl::copyPointCloud(*keypoints, *keypointsXYZ);




	/////////////////////////
	/*cout << "before sorting" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}*/

	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート

																		/*cout << "after sorting (descending order)" << endl;
																		for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
																		cout << "it->intensity: " << it->intensity << endl;
																		}
																		cout << "point extraction" << endl;*/

	int numThreshold = 20; //15 20			20
	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		if (count >= numThreshold) break; //
		keypointsXYZ->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
		count++;
		//cout << "it->intensity: " << it->intensity << endl;
	}
	cout << "keypoints->size(): " << keypoints->size() << endl;
	/////////////////////////




	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	radius = 0.5f; // 3.0f 5.0f			5.0f 0.5f
	kdtree.setInputCloud(cloud);

	Eigen::Vector4f xyz_centroid; // 全体の重心を計算
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			//Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
			//pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}

// 特徴点検出
pcl::PointCloud<pcl::PointXYZ>::Ptr myFeaturePointExtraction_HarrisN2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals) {

	float radius = 3.0f; // 3.0f 5.0f 7.0f

						 // 特徴点検出
	std::cout << "detection" << std::endl;
	pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::Ptr detector(new pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>());
	detector->setNonMaxSupression(true);
	detector->setRadius(radius);
	detector->setRadiusSearch(radius);
	detector->setThreshold(0.005f); // 旋削工具
									//detector->setThreshold(1e-6); // 回転工具
									//detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::TOMASI); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
	detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::HARRIS); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
																						  //detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointNormal, pcl::PointXYZI>::CURVATURE); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE

	detector->setInputCloud(cloud_normals);
	detector->setSearchSurface(cloud_normals);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector->compute(*keypoints);
	cout << "number of source keypoints found: " << keypoints->points.size() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);

	return keypointsXYZ;
}

// 特徴点検出
pcl::PointCloud<pcl::PointXYZ>::Ptr myFeaturePointExtraction_HarrisN3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// パラメータ
	float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f
	float threshold = 0.005f; // 0.005f(旋削工具) 1e-6(回転工具)

							  // 特徴点検出
	std::cout << "detection" << std::endl;
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::Ptr detector(new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>());
	detector->setNonMaxSupression(true);
	detector->setRadius(radius);
	detector->setRadiusSearch(radius); // 最近傍の決定に使用
	detector->setThreshold(threshold);
	//detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::TOMASI); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
	detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE
																					   //detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::CURVATURE); // HARRIS, NOBLE, LOWE, TOMASI, CURVATURE

	detector->setInputCloud(cloud);
	//detector->setSearchSurface(cloud); // 法線推定にダウンサンプリング前の点群を使用できる
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector->compute(*keypoints);
	cout << "number of source keypoints found: " << keypoints->points.size() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);

	return keypointsXYZ;
}

/*
// 法線推定
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// 点情報
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// パラメータ
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// 特徴点の法線生成
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // 全体の重心を計算
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//視点の決定
ne.setInputCloud(cloud);//法線の計算を行いたい点群を指定する
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREEを作る
ne.setSearchMethod(tree);//検索方法にKDTREEを指定する
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);//法線情報を入れる変数
ne.setRadiusSearch(3.0);//検索する半径を指定する 3.0, 1.0
ne.compute(*cloud_normals);//法線情報の出力先を指定する

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

// 法線推定
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// 点情報
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// パラメータ
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// 特徴点の法線生成
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // 全体の重心を計算
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;


//法線情報を入れる変数
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);

// パラメータ
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREEを作る
pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setSearchMethod(tree);// 検索方法にKDTREEを指定する
ne.setInputCloud(keypointsXYZ); // 法線の計算を行いたい点群を指定する
ne.setSearchSurface(cloud); // 法線計算に使う点群を指定する
ne.setRadiusSearch(radius);//検索する半径を指定する 3.0, 1.0
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//視点の決定
ne.compute(*cloud_normals);//法線情報の出力先を指定する


// ne.setInputCloud(keypointsXYZ);	で法線を計算した場合と
// ne.setSearchSurface(cloud);		を追加して法線を計算した場合の違いを確認する(visual化)
// その後，kdtreeに何が入っているか確認(cloudが入っているとよい，おそらく違うけど)
// kdtreeにkeypointsXYZが入っていた場合は新たに作り直す(cloudが入ったkdtree)

for (pcl::PointNormal it : *cloud_normals) {
if (kdtree.radiusSearch(it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroidN; // 近傍の重心を計算
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


// 特徴量計算
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// pcl::FPFHSignature33 バージョン
pcl::PointCloud<pcl::FPFHSignature33>::Ptr myFeatureDescription_FPFH2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());

	float radius = 0.3f;// 3.0f
						// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::FPFHSignature33>::Ptr descriptor(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius * 10);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	return features;
}

// 対応点探索
std::vector<int> myFindMatching2(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features) {

	std::vector<int> correspondences;
	////
	/*std::vector<pair<int, int>> correspondences2;*/
	////

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	correspondences.resize(source_features->size());

	pcl::KdTreeFLANN<pcl::FPFHSignature33> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); ++i) {
		correspondences[i] = -1; // -1 means no correspondence

		if (isnan(source_features->points[i].histogram[0])) continue;

		//元々
		search_tree.nearestKSearch(*source_features, i, 1, index, L2_distance);
		correspondences[i] = index[0];

		////自分
		/*int K = 5;
		//if (search_tree.nearestKSearch(source_features->at(i).histogram, K, index, L2_distance) > 0) {
		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
		for (int j = 0; j < index.size(); j++) {
		correspondences2.push_back(make_pair(i, index[j]));
		}
		}*/
		////
	}

	////
	/*for (std::vector<pair<int, int>>::iterator it = correspondences2.begin(); it != correspondences2.end(); it++) {
	accuracy_file << "[" << it->first << "]: " << it->second << endl;
	}*/

	/*outputfile << "source_features" << endl;
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = source_features->begin(); it != source_features->end(); it++) {
	for (int i = 0; i < it->descriptorSize(); i++) {
	outputfile << "[" << i << "]: " << it->histogram[i] << " ";
	}
	outputfile << endl;
	}
	outputfile << endl;
	outputfile << "target_features" << endl;
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
	for (int i = 0; i < it->descriptorSize(); i++) {
	outputfile << "[" << i << "]: " << it->histogram[i] << " ";
	}
	outputfile << endl;
	}*/

	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = source_features->begin(); it != source_features->end(); it++) {
		double hissum = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			hissum += it->histogram[i];
		}
		for (int i = 0; i < it->descriptorSize(); i++) {
			it->histogram[i] /= hissum;
		}
	}
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
		double hissum = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			hissum += it->histogram[i];
		}
		for (int i = 0; i < it->descriptorSize(); i++) {
			it->histogram[i] /= hissum;
		}
	}
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
		double result = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			result += it->histogram[i] * source_features->points[0].histogram[i];
		}
		outputfile << "result: " << result << endl;
	}

	////

	return correspondences;
}

// 特徴量計算
// pcl::SHOT352 バージョン
pcl::PointCloud<pcl::SHOT352>::Ptr myFeatureDescription_SHOT352(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<pcl::SHOT352>::Ptr features(new pcl::PointCloud<pcl::SHOT352>());

	float radius = 0.3f;// 3.0f 0.3f
						// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::SHOT352>::Ptr descriptor(new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius * 10);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	return features;
}

// 対応点探索
std::vector<int> myFindMatching_SHOT352(pcl::PointCloud<pcl::SHOT352>::Ptr source_features, pcl::PointCloud<pcl::SHOT352>::Ptr target_features) {

	std::vector<int> correspondences;

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	correspondences.resize(source_features->size());

	pcl::KdTreeFLANN<pcl::SHOT352> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); ++i) {
		correspondences[i] = -1; // -1 means no correspondence

		if (isnan(source_features->points[i].descriptor[0])) continue;

		search_tree.nearestKSearch(*source_features, i, 1, index, L2_distance);
		correspondences[i] = index[0];
	}

	return correspondences;
}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching2(std::vector<int> correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	//cout << "correspondences.size(): " << correspondences.size() << endl;

	int nCorrespondence = 0;
	for (int i = 0; i < correspondences.size(); i++) {
		//cout << "correspondences[i]: " << correspondences[i] << endl;
		if (correspondences[i] >= 0) nCorrespondence++; // do not count "-1" in correspondences
	}

	pCorrespondences->resize(nCorrespondence);
	for (int i = 0, j = 0; i < correspondences.size(); i++)
	{
		if (correspondences[i] > 0)
		{
			//cout << "(*pCorrespondences)[" << j << "].distance_B: " << (*pCorrespondences)[j].distance << endl;
			(*pCorrespondences)[j].index_query = i;
			(*pCorrespondences)[j].index_match = correspondences[i];
			j++;
			//cout << "(*pCorrespondences)[" << j << "].distance_B: " << (*pCorrespondences)[j].distance << endl;
		}
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);

	return pCorrespondences;
}

// 対応点探索
pcl::CorrespondencesPtr myFindMatching_SHOT352_2(pcl::PointCloud<pcl::SHOT352>::Ptr source_features, pcl::PointCloud<pcl::SHOT352>::Ptr target_features) {

	int K = 1; // 3 5
			   //std::vector<pair<int, int>> correspondences;
			   //correspondences.reserve(source_features->size()*K);
			   //std::vector<pair<int, int>> correspondences(source_features->size()*K, make_pair(-1, -1));  // -1 means no correspondence

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	correspondences->reserve(source_features->size()*K);

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	pcl::KdTreeFLANN<pcl::SHOT352> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); i++) {

		if (isnan(source_features->points[i].descriptor[0])) continue;

		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
			for (int j = 0; j < K; j++)
				correspondences->push_back(pcl::Correspondence(i, index[j], L2_distance[j]));
		}
	}

	/*for (int i = 0; i < correspondences->size();i++) {
	accuracy_file << "i: " << correspondences->at(i).index_query << "\tj: " << correspondences->at(i).index_match << "\tdistance: " << correspondences->at(i).distance << endl;
	}*/

	for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;

	return correspondences;
}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching2_2(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	/*int nCorrespondence = 0;
	for (int i = 0; i < correspondences.size(); i++) {
	if (correspondences[i].first >= 0) nCorrespondence++; // do not count "-1" in correspondences
	}

	pCorrespondences->resize(nCorrespondence);
	for (int i = 0, j = 0; i < correspondences.size(); i++) {
	if (correspondences[i].first >= 0) {
	(*pCorrespondences)[j].index_query = i;
	(*pCorrespondences)[j].index_match = correspondences[i];
	j++;
	}
	}*/

	pCorrespondences->reserve(correspondences->size());
	for (pcl::Correspondence it : *correspondences)
		pCorrespondences->push_back(it);

	/*pCorrespondences->reserve(correspondences->size());
	for (pcl::Correspondence it : *correspondences)
	pCorrespondences->push_back(pcl::Correspondence(it.index_query, it.index_match, FLT_MAX));*/


	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);

	return pCorrespondences;
}

// RとTの推定
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
Eigen::Matrix4f myEstimatingTransformation2(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features) {

	Eigen::Matrix4f transformation;

	// Estimating rigid transformation
	std::cout << "Estimating transformation" << std::endl;

	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;

	std::vector<int> source_index(source_features->size());
	for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;

	return transformation;

	/*
	est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	std::cout << transformation << std::endl;

	pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	*/

}


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


#include <pcl/keypoints/iss_3d.h>
pcl::PointCloud<pcl::PointNormal>::Ptr ISS3Ddetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
#include <pcl/keypoints/susan.h>
pcl::PointCloud<pcl::PointNormal>::Ptr SUSANdetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void save_MyLightPPF(string filename) {

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 法線の生成
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // 08
	// 特徴点検出，特徴量計算
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals); // 08

	// 法線の生成，特徴点検出，特徴量計算
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction_HarrisN(cloud_lattice); // Harris
																											  //pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = ISS3Ddetector(cloud_lattice); // ISS
																											  //pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = SUSANdetector(cloud_lattice); // SUSAN

																											  // 特徴点検出，特徴量計算
																											  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // 未定
																											  //pcl::PointCloud<pcl::PointXYZ>::Ptr attention_point = myFeaturePointExtraction_HarrisN2(cloud_normals); // 未定
																											  //pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(attention_point); // 未定 注目点の重心(近傍点の重心ですらない)

																											  // 工具データのPPF
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

	// 工具データ情報
	accuracy_file << filename << endl;

	// データマッチング (簡易版)
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	accuracy_file << "target_PPF->size: " << target_PPF.size() << endl; //
	accuracy_file << "source_PPF->size: " << source_PPF.size() << endl; //
	accuracy_file << "match_cloud->size: " << match_cloud << endl;
	accuracy_file << "target_accuracy: " << (float)match_cloud / target_PPF.size() * 100 << endl;
	accuracy_file << "source_accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	accuracy_file << endl;
}


/*
#include <pcl/point_cloud.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/narf.h>
void CRP(pcl::PointXYZ point_cloud) {
float angular_resolution = 0.5f;
int rotation_invariant = 0;
float support_size = 0.3f;
int descriptor_size = 36;
// -----------------------------------------------
// -----Create RangeImage from the PointCloud-----
// -----------------------------------------------
float noise_level = 0.0;
float min_range = 0.0f;
int border_size = 1;
boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
pcl::RangeImage& range_image = *range_image_ptr;
range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
range_image.integrateFarRanges(far_ranges);
if (setUnseenToMaxRange)
range_image.setUnseenToMaxRange();

// Extract NARF features:
std::cout << "Now extracting NARFs in every image point.\n";
std::vector<std::vector<pcl::Narf*> > narfs;
narfs.resize(range_image.points.size());
int last_percentage = -1;
for (unsigned int y = 0; y<range_image.height; ++y)
{
for (unsigned int x = 0; x<range_image.width; ++x)
{
int index = y*range_image.width + x;
int percentage = (int)((100 * index) / range_image.points.size());
if (percentage > last_percentage)
{
std::cout << percentage << "% " << std::flush;
last_percentage = percentage;
}
pcl::Narf::extractFromRangeImageAndAddToList(range_image, x, y, descriptor_size,
support_size, rotation_invariant != 0, narfs[index]);
//std::cout << "Extracted "<<narfs[index].size ()<<" features for pixel "<<x<<","<<y<<".\n";
}
}
std::cout << "100%\n";
std::cout << "Done.\n\n Now you can click on points in the image to visualize how the descriptor is "
<< "extracted and see the descriptor distances to every other point..\n";
}
*/


/*void ISS3Ddetector_sample() {
//
//  ISS3D parameters
//
double iss_salient_radius_;
double iss_non_max_radius_;
double iss_gamma_21_(0.975);
double iss_gamma_32_(0.975);
double iss_min_neighbors_(5);
int iss_threads_(4);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

// Fill in the model cloud
double model_resolution;

// Compute model_resolution
iss_salient_radius_ = 6 * model_resolution;
iss_non_max_radius_ = 4 * model_resolution;

//
// Compute keypoints
//
pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;


iss_detector.setSearchMethod(tree);
iss_detector.setSalientRadius(iss_salient_radius_);
iss_detector.setNonMaxRadius(iss_non_max_radius_);
iss_detector.setThreshold21(iss_gamma_21_);
iss_detector.setThreshold32(iss_gamma_32_);
iss_detector.setMinNeighbors(iss_min_neighbors_);
iss_detector.setNumberOfThreads(iss_threads_);
iss_detector.setInputCloud(model);
iss_detector.compute(*model_keypoints);
}*/

pcl::PointCloud<pcl::PointNormal>::Ptr ISS3Ddetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	//  ISS3D parameters
	double iss_salient_radius_ = 5.0f;
	double iss_non_max_radius_ = 3.0f;
	double iss_gamma_21_(0.975); //0.975	0.50
	double iss_gamma_32_(0.975); //0.975	0.50
	double iss_min_neighbors_(5); // 5	20
								  //int iss_threads_(4);

								  //pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	// Compute keypoints
	//pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;

	detector.setSearchMethod(tree);
	detector.setSalientRadius(iss_salient_radius_);
	detector.setNonMaxRadius(iss_non_max_radius_);
	detector.setThreshold21(iss_gamma_21_);
	detector.setThreshold32(iss_gamma_32_);
	detector.setMinNeighbors(iss_min_neighbors_);
	//detector.setNumberOfThreads(iss_threads_);
	detector.setInputCloud(cloud);
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);


	/////////////////////////
	/*cout << "before sorting" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート
	cout << "after sorting (descending order)" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}*/
	/*cout << "point extraction" << endl;
	int numThreshold = 20; //15
	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	if (count >= numThreshold) break;
	keypointsXYZ->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
	count++;
	//cout << "it->intensity: " << it->intensity << endl;
	}*/
	cout << "keypoints->size(): " << keypoints->size() << endl;
	/////////////////////////


	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);

	Eigen::Vector4f xyz_centroid; // 全体の重心を計算
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			//Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
			//pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}


pcl::PointCloud<pcl::PointNormal>::Ptr SUSANdetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 法線の生成
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud);

	//  parameters
	float radius = 3.0f; // 0.01f
	float distanceThreshold = 0.01f; // 0.001f
	float angularThreshold = 0.01f; // 0.0001f
	float intensityThreshold = 1.0f; // 7.0f 1.5f 1.0f
	bool nonmax = true; // true;
	bool validate = true;
	//int threads = 4;

	/*float radius = 1.0f; // 0.01f
	float distanceThreshold = 0.01f; // 0.001f
	float angularThreshold = 0.001f; // 0.0001f
	float intensityThreshold = 1.0f; // 7.0f 1.5f
	bool nonmax = true; // true;*/


	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());

	// Compute keypoints
	pcl::SUSANKeypoint<pcl::PointXYZ, pcl::PointXYZI> detector;
	//pcl::SUSANKeypoint<pcl::PointNormal, pcl::PointXYZI> detector;

	detector.setSearchMethod(tree);

	detector.setRadius(radius);
	detector.setDistanceThreshold(distanceThreshold);
	detector.setAngularThreshold(angularThreshold);
	detector.setIntensityThreshold(intensityThreshold);
	detector.setNonMaxSupression(nonmax);
	//detector.setGeometricValidation(validate);
	//detector.setNumberOfThreads(threads_);

	detector.setInputCloud(cloud);
	//detector.setInputCloud(cloud_normals);
	//detector.setSearchSurface(cloud_normals);
	//detector.setRadiusSearch(radius);
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);


	/////////////////////////
	/*cout << "before sorting" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート
	cout << "after sorting (descending order)" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	cout << "point extraction" << endl;
	int numThreshold = 20; //15
	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	if (count >= numThreshold) break;
	keypointsXYZ->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
	count++;
	cout << "it->intensity: " << it->intensity << endl;
	}*/
	cout << "keypoints->size(): " << keypoints->size() << endl;
	/////////////////////////


	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float kd_radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);

	Eigen::Vector4f xyz_centroid; // 全体の重心を計算
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, kd_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			//Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
			//pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}



//#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
//float model_ss_(0.01f);
//float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f); //0.02f
float cg_size_(0.01f);
float cg_thresh_(5.0f);

float cloud_ss_(0.03f);

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i) {
		if (!pcl_isfinite((*cloud)[i].x)) continue;

		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2) {
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}

	if (n_points != 0) res /= n_points;

	return res;
}

//
//  Load clouds
//
pcl::PointCloud<pcl::PointXYZ>::Ptr loadClouds(string filename) {

	// 点群情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	return cloud_ptr;
}

//
//  Set up resolution invariance
//
void setUpResolutionInvariance(pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud) {

	if (use_cloud_resolution_) {
		float resolution = static_cast<float> (computeCloudResolution(model_cloud));
		if (resolution != 0.0f)
		{
			//model_ss_ *= resolution;
			//scene_ss_ *= resolution;
			rf_rad_ *= resolution;
			descr_rad_ *= resolution;
			cg_size_ *= resolution;

			cloud_ss_ *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		//std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		//std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;

		std::cout << "Cloud sampling size:    " << cloud_ss_ << std::endl;
	}

	return;
}

//
//  Compute Normals
//
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//法線情報を入れる変数
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud);
	norm_est.compute(*normals);

	return normals;
}

//
//  Downsample Clouds to Extract keypoints
//
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudsToExtractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//特徴点情報を入れる変数
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(cloud_ss_);
	uniform_sampling.filter(*keypoints);
	std::cout << "Cloud total points: " << cloud->size() << "; Selected Keypoints: " << keypoints->size() << std::endl;

	return keypoints;
}

class CLOUDDATA //点群, 法線, キーポイント
{
private: // メンバ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;
public: // コンストラクタ
	CLOUDDATA() {};
	CLOUDDATA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
		this->cloud = cloud;
		this->normals = normals;
		this->keypoints = keypoints;
	};
public: // メソッド
	void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		this->cloud = cloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() {
		return this->cloud;
	}
	void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals) {
		this->normals = normals;
	}
	pcl::PointCloud<pcl::Normal>::Ptr getNormals() {
		return this->normals;
	}
	void setKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
		this->keypoints = keypoints;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoints() {
		return this->keypoints;
	}
};


//
//  Compute Descriptor for keypoints
//
//pcl::PointCloud<pcl::SHOT352>::Ptr computeDescriptorForKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
pcl::PointCloud<pcl::SHOT352>::Ptr computeDescriptorForKeypoints(CLOUDDATA data) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = data.getCloud();
	pcl::PointCloud<pcl::Normal>::Ptr normals = data.getNormals();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = data.getKeypoints();

	//特徴量情報を入れる変数
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(keypoints);
	descr_est.setInputNormals(normals);
	descr_est.setSearchSurface(cloud);
	descr_est.compute(*descriptors);

	return descriptors;
}

//
//  Find Model-Scene Correspondences with KdTree
//
pcl::CorrespondencesPtr findModelSceneCorrespondencesWithKdTree(pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors, pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors) {

	//対応点情報を入れる変数
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<pcl::SHOT352> match_search;
	match_search.setInputCloud(model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	return model_scene_corrs;
}

//
//  Compute (Keypoints) Reference Frames only for Hough
//
//pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeReferenceFramesOnlyForHough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeReferenceFramesOnlyForHough(CLOUDDATA data) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = data.getCloud();
	pcl::PointCloud<pcl::Normal>::Ptr normals = data.getNormals();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = data.getKeypoints();

	pcl::PointCloud<pcl::ReferenceFrame>::Ptr cloud_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(rf_rad_);

	rf_est.setInputCloud(keypoints);
	rf_est.setInputNormals(normals);
	rf_est.setSearchSurface(cloud);
	rf_est.compute(*cloud_rf);

	return cloud_rf;
}

class ActualClustering
{
private: // メンバ
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
public: // コンストラクタ
	ActualClustering() {};
	ActualClustering(vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations, vector<pcl::Correspondences> clustered_corrs) {
		this->rototranslations = rototranslations;
		this->clustered_corrs = clustered_corrs;
	};
public: // メソッド
	void setRototranslations(vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations) {
		this->rototranslations = rototranslations;
	}
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > getRototranslations() {
		return this->rototranslations;
	}
	void setClustered_corrs(vector<pcl::Correspondences> clustered_corrs) {
		this->clustered_corrs = clustered_corrs;
	}
	vector<pcl::Correspondences> getClustered_corrs() {
		return this->clustered_corrs;
	}
};

//
//  Actual Clustering
//
ActualClustering actualClustering(CLOUDDATA model_data, CLOUDDATA scene_data, pcl::CorrespondencesPtr model_scene_corrs) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = model_data.getKeypoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = scene_data.getKeypoints();

	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_)
	{
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		/*pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

		pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);

		rf_est.setInputCloud(model_keypoints);	//	分割すればなんとかなるんじゃね？
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model_cloud);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene_cloud);
		rf_est.compute(*scene_rf);*/

		pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf = computeReferenceFramesOnlyForHough(model_data);
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf = computeReferenceFramesOnlyForHough(scene_data);



		//  Clustering
		pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize(rototranslations, clustered_corrs);
	}
	else // Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
	}

	return ActualClustering(rototranslations, clustered_corrs);
}

class RotationAndTranslation
{
private: // メンバ
	Eigen::Matrix3f rotation;
	Eigen::Vector3f translation;
public: // コンストラクタ
	RotationAndTranslation() {};
	RotationAndTranslation(Eigen::Matrix3f rotation, Eigen::Vector3f translation) {
		this->rotation = rotation;
		this->translation = translation;
	};
public: // メソッド
	void setRotation(Eigen::Matrix3f rotation) {
		this->rotation = rotation;
	}
	Eigen::Matrix3f getRotation() {
		return this->rotation;
	}
	void setTranslation(Eigen::Vector3f translation) {
		this->translation = translation;
	}
	Eigen::Vector3f getTranslation() {
		return this->translation;
	}
};

//
//  Output results
//
RotationAndTranslation outputResults(ActualClustering ac) {

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = ac.getRototranslations();
	std::vector<pcl::Correspondences> clustered_corrs = ac.getClustered_corrs();

	RotationAndTranslation rt;

	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		rt.setRotation(rotation);
		rt.setTranslation(translation);
	}

	return rt;
}


void CorrespondenceGrouping() {

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = myFeatureDescription_FPFH2(cloud_normals, source_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr source_features = myFeatureDescription_SHOT352(cloud_normals, source_keypointsXYZ);

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // trueでノイズ true
																					   // 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_lattice2);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = myFeatureDescription_FPFH2(cloud_normals2, target_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr target_features = myFeatureDescription_SHOT352(cloud_normals2, target_keypointsXYZ);




	// 対応点探索
	//std::vector<int> correspondences = myFindMatching2(source_features, target_features);
	//std::vector<int> correspondences = myFindMatching_SHOT352(source_features, target_features);
	pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	// 特徴点の法線生成
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

	// 誤対応除去
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	// RとTの推定
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);

	// 特徴点の法線生成
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
}


void CorrespondenceGrouping2() {

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";


	// CLOUDDATA model_data2 = CLOUDDATA();
	// STLデータの内挿補間 (モデルデータ) false(ノイズなし)
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud = interpolation_stl(filename, false);
	// 法線の生成
	pcl::PointCloud<pcl::Normal>::Ptr model_normals = computeNormals(model_cloud);
	// 特徴点抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = downsampleCloudsToExtractKeypoints(model_cloud);
	// 特徴量計算
	CLOUDDATA model_data(model_cloud, model_normals, model_keypoints);
	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors = computeDescriptorForKeypoints(model_data);


	// CLOUDDATA scene_data2 = CLOUDDATA();
	// STLデータの内挿補間 (シーンデータ) true(ノイズあり)
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud = interpolation_stl(filename, true);
	// 法線の生成
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals = computeNormals(scene_cloud);
	// 特徴点抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = downsampleCloudsToExtractKeypoints(scene_cloud);
	// 特徴量計算
	CLOUDDATA scene_data(scene_cloud, scene_normals, scene_keypoints);
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors = computeDescriptorForKeypoints(scene_data);


	// 対応点探索
	pcl::CorrespondencesPtr model_scene_corrs = findModelSceneCorrespondencesWithKdTree(model_descriptors, scene_descriptors);
	// 実際のクラスタリング
	ActualClustering ac = actualClustering(model_data, scene_data, model_scene_corrs);
	// 回転行列と平行移動の計算
	RotationAndTranslation rt = outputResults(ac);


	return;
}







#include "MT.h"
#include "randUnitVector.h"



#include <pcl/features/principal_curvatures.h>
// --------------
// -----Main-----
// --------------
int main(int argc, char* argv[]) {
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\06\\outputfile\\result004.txt");
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\打ち合わせ\\20\\outputfile\\20_5\\test\\endmill\\output000.txt");
	//std::chrono::system_clock::time_point  start, end; // 型は auto で可
	//double elapsed;


	// ------------------------------------------
	// 工具ホルダ端面と工具突き出し量を求める処理
	// ------------------------------------------


	// 工具姿勢（固定）
	/*pcl::Normal tool_normal(0.0f, 0.0f, 1.0f);
	float toolN_norm = sqrt((tool_normal.normal_x * tool_normal.normal_x) + (tool_normal.normal_y * tool_normal.normal_y) + (tool_normal.normal_z * tool_normal.normal_z));
	double theta_cos = 0.9962; // 5度*/


	/*
	// 測定データ読み込み（全て）
	const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\dr1_whitesprayed2mod2.stl";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\ex1_whitesprayed2mod2.stl";
	STLDATA stl_object(filename);

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\endmill\\OCTACUT322S32RB.STL";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\drill_20170119\\merged_pointcloud_ノイズ除去有_ascii.stl";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\2017_03_03\\5.0_-5.0.STL";
	//STLDATA stl_object(filename);



	// 測定データ読み込み（工具先端部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	/////// 工具先端のみ ///////
	// ドリル
	//if (point.y > -6) continue;	// 外れ値の除去をすればこれでOK
	if (point.y > -7) continue;		// 外れ値の除去なし
	//if (point.y > -20) continue;
	//if (point.y < -21) continue;
	// バイト
	//if (point.y > -35) continue;	// 奥行きのみ
	//if (point.y > -27) continue;
	//if (point.z < 65) continue;
	////////////////////////////
	cloud_in->points.push_back(point);
	}
	}
	//cout << "cloud_in->size(): " << cloud_in->size() << endl;


	// 特徴点の法線生成
	// average 工具全体の重心
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


	// 特徴点生成
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


	// 特徴点の法線生成
	// average 工具全体の重心
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

	// 測定データ読み込み（工具先端部）
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in0 = stl_points0(filename);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in1 = stl_nAveraging(cloud_in0);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in2 = stl_decimating0(cloud_in1);

	// 測定データ読み込み（工具先端部）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = stl_pointsXYZ(filename);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);

	// 外れ値の除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	// 特徴点の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(surfaceN);
	*/

	/*
	// 測定データのPPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);
	*/



	/*
	// 外れ値の除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);
	*/













	// 工具データ読み込み
	//const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\立方体.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\直方体.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\円柱.STL";



	/*
	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// 特徴点検出，特徴量計算
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals);
	// 工具データのPPF
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> source_PPF = make_PPFs(attention_point);
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);
	*/



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	// 点群データ
	//const string filename = "C:\\Users\\yuki\\Desktop\\村上測定データ\\外径バイト_PVJNR2525M.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\村上測定データ\\外径バイト_STGER2020K16.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\村上測定データ\\ドリル_MDW0300GS4.txt";
	const string filename = "C:\\Users\\yuki\\Desktop\\村上測定データ\\ドリル_MWE0300MA.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_in2);
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // 点群データのとき
	// RANSAC 平面除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // 点群データのとき

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	//cout << "cloud_in2->size(): " << cloud_in2->size() << endl;
	//cout << "cloud->size(): " << cloud->size() << endl;
	//cout << "cloud_lattice->size(): " << cloud_lattice->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // trueでノイズ true
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f); // 0.5f, 0.5f, 0.5f		10.0f, 10.0f, 10.0f
	sor2.filter(*cloud_lattice2);*/



	// 法線の生成
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// 特徴点検出，特徴量計算
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals2);
	//オクルージョン有り (テスト)
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals3);*/


	// Harris の特徴点抽出
	// オクルージョンなし
	/*pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(cloud_lattice2); //Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(cloud_lattice2); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN*/
	// オクルージョンあり
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_normals3, *surface0);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(surface0); // Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(surface0); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN


	// 工具データのPPF
	/*vector<myPPF> target_PPF = make_lightPPFs(attention_point2);*/



	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);*/

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	/*
	// データマッチング (簡易版)
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

	path = "C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\PPF_database\\06\\Alexandre\\NikonTurningTool.txt";
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

	path = "C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\PPF_database\\07\\Alexandre\\NikonTurningTool.txt";
	source_PPF = load_MyLightPPF(path);
	// 工具データ情報
	outputfile777 << path << endl;
	// データマッチング (簡易版)
	match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "match_cloud->size: " << match_cloud << endl;
	outputfile777 << "accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	outputfile777 << endl;
	*/












	/*
	//std::chrono::system_clock::time_point  start, end; // 型は auto で可
	//double elapsed;
	// 点群データ
	//start = std::chrono::system_clock::now(); // 計測開始時間
	const string filename = "C:\\Users\\yuki\\Desktop\\村上測定データ\\ドリル.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud);

	outputfile777 << "cloud_in->size(): " << cloud_in->size() << endl;
	outputfile777 << endl;
	end = std::chrono::system_clock::now();  // 計測終了時間
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/

















	/*
	// データマッチング
	string path;
	string database_path;
	string feature_value = "";
	string type_company;
	string extension = ".STL";


	// 11個
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

	// 15個
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

	// 47個
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

	// 12個
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
	// データマッチング
	string path;
	string database_path = "C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\PPF_database";
	string feature_value = "\\Harris_20";
	string type_company;
	string extension = ".txt";


	// 11個
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

	// 15個
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

	// 47個
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

	// 12個
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
	// 工具データ読み込み（データマッチング）
	// 11個
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

	// 15個
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

	// 47個
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

	// 12個
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




































	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";



	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	//pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal = myFeaturePointExtraction(cloud_normals);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*source_keypointsNormal, *source_keypointsXYZ);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = myFeatureDescription_FPFH2(cloud_normals, source_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr source_features = myFeatureDescription_SHOT352(cloud_normals, source_keypointsXYZ);



	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // trueでノイズ true
																					   // 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_lattice2);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// 特徴点検出
	std::chrono::system_clock::time_point  start, end; // 型は auto で可
	double elapsed;

	start = std::chrono::system_clock::now(); // 計測開始時間
											  //pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal = myFeaturePointExtraction(cloud_normals2);
	end = std::chrono::system_clock::now();  // 計測終了時間

	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	accuracy_file << "input_cloud: " << elapsed << "[ms]" << endl;
	accuracy_file << endl;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*target_keypointsNormal, *target_keypointsXYZ);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = myFeatureDescription_FPFH2(cloud_normals2, target_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr target_features = myFeatureDescription_SHOT352(cloud_normals2, target_keypointsXYZ);




	// 対応点探索
	//std::vector<int> correspondences = myFindMatching2(source_features, target_features);
	//std::vector<int> correspondences = myFindMatching_SHOT352(source_features, target_features);
	pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	// 特徴点の法線生成
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

	// 誤対応除去
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	// RとTの推定
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);

	// 特徴点の法線生成
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
	cout << "テスト： " << "test" << endl;
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






















	// 正規化
	/*for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\MVX2000X2F25_result003.txt");
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

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result005.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessPoints(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < Sn_high.size(); i++) {
	uniquenessPoints->push_back(surfaceN->at(Sn_high[i].second));
	}
	*/





	// 点群の格子化（ダウンサンプリング）
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointNormal>);
	pcl::VoxelGrid<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_ptr);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 外れ値の除去
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_lattice);		// 外れ値を除去する点群を入力
	sor.setMeanK(50);				// 検索する近傍点の数を設定
	sor.setStddevMulThresh(1.0);	// 距離の標準偏差の値を設定
	sor.setNegative(false);			// 外れ値を出力する場合はtrueにする
	sor.filter(*cloud_filtered);	// 出力

	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_lattice;*/

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud = cloud_ptr;

	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_ptr;



	/*
	// STLの法線方向を利用した特徴点の自動抽出
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
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);



	// 外れ値の除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	*/


	// 測定データ読み込み（ホルダ端面）（法線ベクトルが工具姿勢と同じになる三角形ポリゴンの抽出）
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);

	if (angle_cos < theta_cos) { // 角度が大きければcosθは小さくなる
	continue;
	}

	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	//////// ホルダ端面 ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;*/


	// 測定データ読み込み（ホルダ端面）（法線ベクトルが工具姿勢と同じになる三角形ポリゴンの抽出）
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);
	if (angle_cos < theta_cos) { // 角度が大きければcosθは小さくなる
	continue;
	}
	pcl::PointXYZ point;
	point.x = tri.vertex[0].x;
	point.y = tri.vertex[0].y;
	point.z = tri.vertex[0].z;
	//////// ホルダ端面 ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;



	// ホルダ端面の検出（RANSAC）
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_wall(holder_face, false);*/


	// ホルダ端面の検出（量子化によるY値比較）
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y値の量子化
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


	// ホルダ端面の検出（Y値によるグルーピング） ← 未完成
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y値の量子化
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



	// ホルダ端面の決定（最小二乗法）
	/*array<array<double, 3>, 3> holder_PCA = cal_PCA(sample_vec);
	pcl::Normal holder_normal(holder_PCA[0][0], holder_PCA[0][1], holder_PCA[0][2]);
	//cout << "x: " << holder_normal.normal_x << "\ty: " << holder_normal.normal_y << "\tz: " << holder_normal.normal_z << endl;



	// 工具先端点の抽出
	priority_queue <pcl::PointXYZ, vector<pcl::PointXYZ>, PointYCompareMin> minpq;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
	minpq.push(*it);
	}
	//cout << "minpq" << endl;
	pcl::PointXYZ min_value = minpq.top();
	//cout << "x: " << min_value.x << "\ty: " << min_value.y << "\tz: " << min_value.z << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tip_point(new pcl::PointCloud<pcl::PointXYZ>);
	tip_point->push_back(min_value);



	// 点と面の距離算出
	pcl::PointXYZ A = sample_vec->at(0);
	float coef_d = -(holder_normal.normal_x*A.x + holder_normal.normal_y*A.y + holder_normal.normal_z*A.z);
	float error_d = -(holder_normal.normal_x*min_value.x + holder_normal.normal_y*min_value.y + holder_normal.normal_z*min_value.z);
	float distance = fabsf(coef_d - error_d);
	cout << "distance: " << distance << endl;*/


	/*
	// 曲率
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
	// RANSACによる形状分類
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN2 = surface_normals(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN(new pcl::PointCloud<pcl::PointNormal>());
	surfaceN->reserve(surfaceN2->size());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN2->begin(); it != surfaceN2->end(); it++) {
	if (isnan(it->normal_x)) continue;
	surfaceN->push_back(*it);
	}
	*/

	/*ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\MVX2000X2F25_result001.txt");
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
	// 局所形状の独自性に着目した物体認識に有効な3-D特徴点の自動抽出
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

	// 正規化
	for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\MVX2000X2F25_result003.txt");
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

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\研究室関連\\輪講・打ち合わせ\\M2\\打ち合わせ\\02\\outputfile\\NDH_result005.txt");
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






	// 特徴点生成
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



	// 特徴点の法線生成
	// average 工具全体の重心
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(cloud);
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/
	// average 特徴点近傍の重心
	/*pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointXYZ cloud_average = cal_cloud_average(neighborhood_cloud);
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/



	// 測定データのPPF
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(keypoint_normals);
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs2(keypoint_normals);
	//outputfile777 << "keypoint_cloud->size(): " << keypoint_cloud->size() << endl;
	//outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;*/
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);



	// 工具データ読み込み（データマッチング）
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
	// 点を間引く処理
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
	// 情報量高いやつ
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



	// 点群データ
	/*start = std::chrono::system_clock::now(); // 計測開始時間
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
	end = std::chrono::system_clock::now();  // 計測終了時間
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/


	////////////////////////////////////////////

	//outputfile.close();
	outputfile777.close();


	// 入力点群と法線を表示
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