#ifndef _DataMatching_HPP_
#define _DataMatching_HPP_

#include "Output.h"
#include "PreprocessingSTL.hpp"
#include "PointSearch.hpp"
#include "PPF.hpp"
#include "NormalDirection.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>
#include <pcl/keypoints/harris_3d.h>
#include <vector>

using namespace std;

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

#endif // _DataMatching_HPP_