#ifndef _Harris_HPP_
#define _Harris_HPP_

#include "SurfaceNormals.hpp"
#include "NormalDirection.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/harris_3d.h>

using namespace std;

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
	float threshold = 1e-12; // 0.005f(旋削工具) 1e-6(回転工具)
	//float threshold = 0.005f; // 0.005f(旋削工具) 1e-6(回転工具)
	//float threshold = 1e-72; // 意味ないかも

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

	/*
	// 点の数(-nan出るかも)
	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート
	int numThreshold = 50; //15 20
	int count = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		if (count >= numThreshold) break; //
		cloud_ptr->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
		count++;
	}

	return cloud_ptr;
	*/
}


// 特徴点検出
pcl::PointCloud<pcl::PointXYZINormal>::Ptr myFeaturePointExtraction_HarrisN4(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// パラメータ
	float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f
	float threshold = 1e-12; // 0.005f(旋削工具) 1e-6(回転工具)
	//float threshold = 0.005f; // 0.005f(旋削工具) 1e-6(回転工具)
	//float threshold = 1e-72; // 意味ないかも

	// 特徴点検出
	std::cout << "detection" << std::endl;
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::Ptr detector(new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>());
	detector->setNonMaxSupression(true);
	//detector->setNonMaxSupression(false);
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

	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	//keypointsXYZ->points.resize(keypoints->points.size());
	//pcl::copyPointCloud(*keypoints, *keypointsXYZ);

	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointXYZINormal>);
	*attention_point = *myKeypoint_normals2(cloud, keypoints);

	return attention_point;

}

#endif // _Harris_HPP_