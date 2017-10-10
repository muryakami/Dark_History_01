#ifndef _ForFixPoint_HPP_
#define _ForFixPoint_HPP_

#include "Output.h"
#include "InterpolationSTL.hpp"
#include "SurfaceNormals.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include "PPF.hpp"
#include "KeypointNormals.hpp"
//#include <pcl/features/feature.h>
//#include <pcl/filters/voxel_grid.h>
#include <chrono>

vector<myPPF> forFixPoint(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// 特徴点検出
	start = std::chrono::system_clock::now(); // 計測開始時間
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	end = std::chrono::system_clock::now();  // 計測終了時間
	// 特徴点の法線生成
	*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);

	// 処理時間の出力
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;

	return make_lightPPFs(attention_point);
}

// 一致率の出力
void outputAccuracy(vector<myPPF>* source_PPF, vector<myPPF>* target_PPF) {
	int match_cloud = myLightPPF_matching(*source_PPF, *target_PPF);
	accuracy_file << "accuracy: " << (float)match_cloud / target_PPF->size() * 100 << " [%]" << endl;
	accuracy_file << endl;
}


#endif // _ForFixPoint_HPP_