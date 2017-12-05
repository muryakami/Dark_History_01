#ifndef _Simulation20171030_HPP_
#define _Simulation20171030_HPP_

#include "Output.h"
#include "InterpolationSTL.hpp"
#include "SurfaceNormals.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include "PPF.hpp"
#include "KeypointNormals.hpp"
#include "AddingNoise.hpp"
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>


// 【マッチング手法の性能評価実験】
// 抽出した特徴点にノイズを加え，一致率を計算する

vector<myPPF> procedure(string filename, boolean TFC, boolean TFA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {

	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);

	// trueでノイズ付与
	if(TFC) addingNoise2(cloud_in, maxNoiseC);

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

	// 特徴点へのノイズ付与
	if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

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


void simulation20171030() {
	// 工具データ読み込み
	const string filename = ".\\STL files\\NikonTurningTool.STL";

	// PPFの生成(STLデータ:A)	ソース
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> source_PPF = procedure(filename, false, false, cloud_lattice, attention_point);

	// PPFの生成(STLデータ:A)	ターゲット
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2(new pcl::PointCloud<pcl::PointNormal>);
	// 点群誤差
	vector<myPPF> target_PPF = procedure(filename, true, false, cloud_lattice2, attention_point2);
	// 特徴点誤差
	//vector<myPPF> target_PPF = procedure(filename, false, true, cloud_lattice2, attention_point2);

	// 誤差の出力
	nearest_search_test3(attention_point, attention_point2);

	// 一致率の出力
	outputAccuracy(&source_PPF, &target_PPF);	// AとA



	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lattice_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, lattice_color, "cloud_lattice2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice2");

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color2(attention_point2, 0, 255, 0);
	viewer->addPointCloud(attention_point2, attention_color2, "attention_point2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point2");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_point2n");

	// 表示時のパラメータ
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


#endif // _Simulation20171030_HPP_