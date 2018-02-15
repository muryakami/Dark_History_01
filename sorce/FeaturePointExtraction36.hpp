#ifndef _FPE36_HPP_
#define _FPE36_HPP_

#include "Output.h"
#include "BasicCalculation.hpp"
#include "PointSearch.hpp"
#include "CalPCA.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;


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

inline float bhattacharyya18(array<float, 18> p, array<float, 18> q) {
	float result = 0;
	for (int i = 0; i < 18; i++) {
		result += p[i] * q[i];
	}
	return result;
}

inline float uniqueness18(array<float, 18> n, vector<array<float, 18>> NDH) {
	float result = 0;
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result += bhattacharyya18(n, *it);
	}
	result /= NDH.size();
	return result;
}

inline vector<float> all_uniqueness18(vector<array<float, 18>> NDH) {
	vector<float> result;
	result.reserve(NDH.size());
	for (vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
		result.push_back(1 - uniqueness18(*it, NDH));
	}
	return result;
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
	
	// 独自性指標が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
	if (it->curvature < 0.85) continue; //0.85 0.90
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
	


	// 点の数で決定
	/*
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
	*/
	return cloud_ptr;
}



//////////////////////////
//		リメイク		//
//////////////////////////

// 極形式ヒストグラムの計算
std::vector<array<float, 18>> myPolarHistograms18Re(pcl::PointCloud<pcl::PointNormal>::Ptr searchPoints, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	std::vector<array<float, 18>> PHs;
	PHs.reserve(cloud->size());

	float radius = 3.0f;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = searchPoints->begin(); it != searchPoints->end(); it++) {
		pcl::PointNormal searchPoint = *it;
		vector<pair<int, float>> neighbors_inf = radius_search_inf(searchPoint, radius, cloud);

		// もし近傍点が存在しなかったらループを抜ける（-nan(ind) ゼロディバイド対策）
		//if (neighbors_inf.size() == 0) continue;
		if (neighbors_inf.size() == 0) {
			processing_time << "neighbors_inf.size(): " << neighbors_inf.size() << endl;
			continue;
		}

		pcl::PointCloud<pcl::PointNormal>::Ptr neighbors(new pcl::PointCloud<pcl::PointNormal>);

		// 近傍点情報から近傍点の配列を作成
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			neighbors->push_back(cloud->points[itr->first]);
		}

		/*
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighborsR(new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			pcl::PointNormal pi = cloud->points[itr->first];
			pcl::PointXYZ rotatePoint;
			rotatePoint.x = rotate_matrix[0][0] * pi.normal_x + rotate_matrix[0][1] * pi.normal_y + rotate_matrix[0][2] * pi.normal_z;
			rotatePoint.y = rotate_matrix[1][0] * pi.normal_x + rotate_matrix[1][1] * pi.normal_y + rotate_matrix[1][2] * pi.normal_z;
			rotatePoint.z = rotate_matrix[2][0] * pi.normal_x + rotate_matrix[2][1] * pi.normal_y + rotate_matrix[2][2] * pi.normal_z;
			neighborsR->push_back(rotatePoint);
		}

		// xy平面で主成分分析すべき？
		array<array<double, 3>, 3> neighborsR_PCA = cal_PCA(neighborsR);


		// 正射影(近傍点の法線を回転)
		vector<array<float, 2>> neighborsR2;
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			pcl::PointNormal pi = cloud->points[itr->first];
			array<float, 2> rotatePoint;
			rotatePoint[0] = rotate_matrix[0][0] * pi.normal_x + rotate_matrix[0][1] * pi.normal_y + rotate_matrix[0][2] * pi.normal_z;
			rotatePoint[1] = rotate_matrix[1][0] * pi.normal_x + rotate_matrix[1][1] * pi.normal_y + rotate_matrix[1][2] * pi.normal_z;
			neighborsR2.push_back(rotatePoint);
		}

		// xy平面での主成分分析のはず
		array<array<double, 2>, 2> neighborsR2_PCA = cal_PCA2(neighborsR2);
		*/



		// 正射影(近傍点の法線をxy平面に射影)
		pcl::PointNormal plane;
		plane.x = 0.0f;	plane.y = 0.0f;	plane.z = 0.0f;
		plane.normal_x = 0.0f;	plane.normal_y = 0.0f;	plane.normal_z = 1.0f;
		vector<array<float, 2>> neighborsR3;
		for (vector<pair<int, float>>::iterator itr = neighbors_inf.begin(); itr != neighbors_inf.end(); itr++) {
			pcl::PointNormal pi = cloud->points[itr->first];

			// 射影したい点から平面上の点への移動ベクトル
			pcl::PointXYZ moveVector(plane.x - pi.normal_x, plane.y - pi.normal_y, plane.z - pi.normal_x);

			// 移動ベクトルの法線方向成分を計算(点から平面までの距離)
			pcl::PointXYZ planeNormal(plane.normal_x, plane.normal_y, plane.normal_z);
			float d = dot_product3_cal(moveVector, planeNormal);
			pcl::PointXYZ distanceVector(d*planeNormal.x, d*planeNormal.y, d*planeNormal.z);

			// 射影したい点を平面に移動
			//pcl::PointXYZ projectionPoint(pi.normal_x + distanceVector.x, pi.normal_y + distanceVector.y, pi.normal_z + distanceVector.z);
			array<float, 2> projectionPoint;
			projectionPoint[0] = pi.normal_x + distanceVector.x;
			projectionPoint[1] = pi.normal_y + distanceVector.y;

			neighborsR3.push_back(projectionPoint);
		}

		// xy平面での主成分分析
		array<array<double, 2>, 2> neighborsR3_PCA = cal_PCA2(neighborsR3);


		/*rpc << "固有ベクトル：" << neighborsR_PCA[0][0] << "\t" << neighborsR_PCA[0][1] << "\t" << neighborsR_PCA[0][2] << endl;
		rpc << "固有ベクトル：" << neighborsR_PCA[1][0] << "\t" << neighborsR_PCA[1][1] << "\t" << neighborsR_PCA[1][2] << endl;
		rpc << "固有ベクトル：" << neighborsR_PCA[2][0] << "\t" << neighborsR_PCA[2][1] << "\t" << neighborsR_PCA[2][2] << endl;
		rpc << endl;

		rpc << "固有ベクトル：" << neighborsR2_PCA[0][0] << "\t" << neighborsR2_PCA[0][1] << endl;
		rpc << "固有ベクトル：" << neighborsR2_PCA[1][0] << "\t" << neighborsR2_PCA[1][1] << endl;
		rpc << endl;

		rpc << "固有ベクトル：" << neighborsR3_PCA[0][0] << "\t" << neighborsR3_PCA[0][1] << endl;
		rpc << "固有ベクトル：" << neighborsR3_PCA[1][0] << "\t" << neighborsR3_PCA[1][1] << endl;
		rpc << endl;*/



		// 各点における最近傍点までの距離の総和(重み係数w1の計算で使用)
		/*int K = 2;
		float nearestSum = 0;
		for (pcl::PointCloud<pcl::PointNormal>::iterator itr = neighbors->begin(); itr != neighbors->end(); itr++) {
			vector<pair<int, float>> nearestP = nearest_search1(*itr, K, neighbors);
			for (vector<pair<int, float>>::iterator iter = nearestP.begin(); iter != nearestP.end(); iter++) {
				nearestSum += iter->second;
			}
		}*/


		// 極形式ヒストグラムの初期化
		array<float, 18> angleH;
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] = 0;

		// ヒストグラムの正規化用
		float sum_total = 0;

		// 極形式ヒストグラムの作成
		for (int i = 0; i < neighborsR3.size();i++) {
		//for (array<float, 2> it : neighborsR3) {
			float squared_distance = neighbors_inf[i].second;
			double cosR = neighborsR3_PCA[1][0] * neighborsR3[i][0] + neighborsR3_PCA[1][1] * neighborsR3[i][1];
			double sinR = neighborsR3_PCA[1][0] * neighborsR3[i][1] - neighborsR3_PCA[1][1] * neighborsR3[i][0];
			if (cosR > 1) cosR = 1;
			if (cosR < -1) cosR = -1;
			double radian = acos(cosR);
			double angle = radian * 180 / M_PI;

			// 注目点と近傍点の距離に関する係数
			float w2 = radius - sqrt(squared_distance);
			// θの安定性に関する係数
			float w3 = 1 - (searchPoint.normal_x*neighborsR3[i][0] + searchPoint.normal_y*neighborsR3[i][1]);

			angle /= 10;
			int indexH = (int)angle;
			angleH[indexH] += w2*w3;
			sum_total += w2*w3;
		}

		// 極形式ヒストグラムの正規化
		for (int j = 0; j < angleH.size(); j++)
			angleH[j] /= sum_total;

		// 極形式ヒストグラムにおけるピークの格納位置を調整(最初のビンに統一)
		/*std::array<float, 18>::iterator maxIt = std::max_element(angleH.begin(), angleH.end());
		int maxIndex = std::distance(angleH.begin(), maxIt);
		array<float, 18> tempH;
		for (int j = 0; j < tempH.size(); j++)
			tempH[j] = angleH[j];
		for (int j = 0; j < angleH.size(); j++) {
			int tempIndex = (maxIndex - j + 18) % 18;
			angleH[j] = tempH[tempIndex];
		}*/

		// デバッグのための出力
		//processing_time << "inf: " << neighbors_inf.size() << "\tneighbors: " << neighbors->size() << "\tneighborsR3: " << neighborsR3.size() << "\tsum: " << sum_total << endl;

		// 極形式ヒストグラムの配列に格納
		PHs.push_back(angleH);

	}

	for (vector<array<float, 18>>::iterator it = PHs.begin(); it != PHs.end(); it++) {
		for (int i = 0; i < it->size(); i++) {
			outputfile << "PHs[" << i << "]: " << it->at(i) << endl;
		}
		outputfile << endl;
	}

	return PHs;
}


// 局所形状の独自性に着目した物体認識に有効な3-D特徴点の自動抽出
pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtractionRe(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

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
	std::vector<array<float, 18>> PHs = myPolarHistograms18Re(highCurvaturePoints, cloud);
	processing_time << "PHs.size: " << PHs.size() << endl;
	for (array<float, 18> PH : PHs) {
		processing_time << "独自性: " << endl;
		for (float it : PH) {
			processing_time << it << "\t";
		}
		processing_time << endl;
	}

	// 独自性指標の算出
	vector<float> Sn = all_uniqueness18(PHs);
	for (float it : Sn) {
		accuracy_file << "独自性: " << it << endl;
		accuracy_file << endl;
	}

	// 曲率領域に独自性指標の値を代入
	for (int i = 0; i < highCurvaturePoints->size(); i++) {
		highCurvaturePoints->at(i).curvature = Sn[i];
	}


	// 独自性指標の値で決定
	
	// 独自性指標が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
		if (it->curvature < 0.85f) continue; //0.85 0.9
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
	


	// 点の数で決定
	/*
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
	*/


	// モデルの重心を基にした法線
	//return cloud_ptr;



	// 局所領域の重心を基にした法線
	// 特徴点のXYZ座標
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(cloud_ptr->points.size());
	pcl::copyPointCloud(*cloud_ptr, *keypointsXYZ);

	// 点群のXYZ座標
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	cloudXYZ->points.resize(cloud->points.size());
	pcl::copyPointCloud(*cloud, *cloudXYZ);

	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	*attention_point = *myKeypoint_normals(cloudXYZ, keypointsXYZ);

	return attention_point;
}


//pcl::PointXYZINormal
// 局所形状の独自性に着目した物体認識に有効な3-D特徴点の自動抽出
pcl::PointCloud<pcl::PointNormal>::Ptr myFeaturePointExtractionRe2(pcl::PointCloud<pcl::PointXYZINormal>::Ptr harrisPoints, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 曲率が高い点の抽出
	//pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints = cloud;
	/*pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints(new pcl::PointCloud<pcl::PointNormal>);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++) {
	if (it->curvature < 0.03) continue;
	highCurvaturePoints->push_back(*it);
	}*/

	//return highCurvaturePoints; // 高曲率点群の確認

	pcl::PointCloud<pcl::PointNormal>::Ptr highCurvaturePoints(new pcl::PointCloud<pcl::PointNormal>());
	highCurvaturePoints->points.resize(harrisPoints->points.size());
	pcl::copyPointCloud(*harrisPoints, *highCurvaturePoints);
	/*rpc << "HarrisPoints.size(): " << harrisPoints->size() << endl;
	rpc << "highCurvaturePoints.size(): " << highCurvaturePoints->size() << endl;
	rpc << endl;
	for (pcl::PointNormal it : *highCurvaturePoints) {
		rpc << it.x << "\t" << it.y << "\t" << it.z << "\t" << it.normal_x << "\t" << it.normal_y << "\t" << it.normal_z << endl;
	}*/

	//return highCurvaturePoints; // Harris点群の確認


	// 極形式ヒストグラムの算出
	std::vector<array<float, 18>> PHs = myPolarHistograms18Re(highCurvaturePoints, cloud);
	processing_time << "PHs.size: " << PHs.size() << endl;
	for (array<float, 18> PH : PHs) {
		processing_time << "独自性: " << endl;
		for (float it : PH) {
			processing_time << it << "\t";
		}
		processing_time << endl;
	}

	// 独自性指標の算出
	vector<float> Sn = all_uniqueness18(PHs);
	for (float it : Sn) {
		accuracy_file << "独自性: " << it << endl;
		accuracy_file << endl;
	}

	// 曲率領域に独自性指標の値を代入
	/*for (int i = 0; i < highCurvaturePoints->size(); i++) {
	highCurvaturePoints->at(i).curvature = Sn[i];
	}
	// 独自性指標が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
		if (it->curvature < 0.85) continue; //1e-4 0.85 0.9 1e-12 0.001f
		stl_cloud->push_back(*it);
	}*/

	// 曲率領域に独自性指標の値を代入　←　代入じゃなくて掛け合わせることで重みにする
	for (int i = 0; i < highCurvaturePoints->size(); i++) {
		highCurvaturePoints->at(i).curvature = harrisPoints->at(i).intensity * (Sn[i] * Sn[i]);
		rpc << "Harris: " << harrisPoints->at(i).intensity << endl;
		rpc << "独自性: " << Sn[i] << endl;
		rpc << "2つの積: " << highCurvaturePoints->at(i).curvature << endl;
		rpc << endl;
	}
	// 独自性指標の値で決定
	// 独自性指標が高い点の抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = highCurvaturePoints->begin(); it != highCurvaturePoints->end(); it++) {
		if (it->curvature < 0.001f) continue; //1e-4 0.85 0.9 1e-12 0.001f
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



	// 点の数で決定
	/*
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
	*/

	return cloud_ptr;
}




#endif // _FPE36_HPP_