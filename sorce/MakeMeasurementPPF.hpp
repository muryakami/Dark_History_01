#ifndef _MakeMeasurementPPF_HPP_
#define _MakeMeasurementPPF_HPP_

#include "Output.h"
#include "InterpolationSTL.hpp"
#include "SurfaceNormals.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include "PPF.hpp"
#include "KeypointNormals.hpp"
#include "AddingNoise.hpp"

vector<myPPF> makeMeasurementPPF(const string targetPath) {

	bool TFC = false;
	bool TFA = false;
	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;
	
	// 点群データ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(targetPath);
	//const string filename = "..\\DataBase\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);


	/*
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // 点群データのとき
	// RANSAC 平面除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // 点群データのとき

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(targetPath);

	// 点群へのノイズ付与(trueで付与)
	if (TFC) addingNoise2(cloud_in, maxNoiseC);
	*/
	

	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // 点群データのとき
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = cloud_lattice;

	/*
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// 特徴点検出
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(cloud_normals);
	

	// 特徴点へのノイズ付与
	//if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	*attention_point = *myKeypoint_normals(cloud_lattice2, keypointsXYZ);
	*/

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// 特徴点検出
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice2);

	// Harris特徴点の中で独自性の高いものを抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);



	// PPFの作成
	vector<myPPF> target_PPF = make_lightPPFs(attention_point2);

	return target_PPF;
}

void makeMeasurementPPF2(const string targetPath) {
//void makeMeasurementPPF() {

	bool TFC = false;
	bool TFA = false;
	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;

	// 点群データ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(targetPath);
	//const string filename = "..\\DataBase\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);


	/*
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // 点群データのとき
	// RANSAC 平面除去
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // 点群データのとき

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(targetPath);

	// 点群へのノイズ付与(trueで付与)
	if (TFC) addingNoise2(cloud_in, maxNoiseC);
	*/


	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 外れ値の除去
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // 点群データのとき
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = cloud_lattice;

	/*
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// 特徴点検出
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(cloud_normals);


	// 特徴点へのノイズ付与
	//if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	*attention_point = *myKeypoint_normals(cloud_lattice2, keypointsXYZ);
	*/

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// 特徴点検出
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice2);

	// Harris特徴点の中で独自性の高いものを抽出
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);



	// PPFの作成
	vector<myPPF> target_PPF = make_lightPPFs(attention_point2);



	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, cloud_color, "cloud_lattice");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// 円筒点の表示
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cylinder_color(cloud_cylinder, 0, 255, 0);
	//viewer->addPointCloud(cloud_cylinder, cylinder_color, "cloud_cylinder");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud_cylinder");
	//viewer->addPointCloudNormals<pcl::PointNormal>(cloud_cylinder, 1, 3.0, "cloud_cylinder_normal");

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_point_color(attention_point2, 0, 255, 0);
	viewer->addPointCloud(attention_point2, attention_point_color, "attention_point");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_pointn");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

#endif // _MakeMeasurementPPF_HPP_