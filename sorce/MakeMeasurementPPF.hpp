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
	
	// �_�Q�f�[�^
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(targetPath);
	//const string filename = "..\\DataBase\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);


	/*
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // �_�Q�f�[�^�̂Ƃ�
	// RANSAC ���ʏ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(targetPath);

	// �_�Q�ւ̃m�C�Y�t�^(true�ŕt�^)
	if (TFC) addingNoise2(cloud_in, maxNoiseC);
	*/
	

	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = cloud_lattice;

	/*
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// �����_���o
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(cloud_normals);
	

	// �����_�ւ̃m�C�Y�t�^
	//if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	*attention_point = *myKeypoint_normals(cloud_lattice2, keypointsXYZ);
	*/

	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// �����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice2);

	// Harris�����_�̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);



	// PPF�̍쐬
	vector<myPPF> target_PPF = make_lightPPFs(attention_point2);

	return target_PPF;
}

void makeMeasurementPPF2(const string targetPath) {
//void makeMeasurementPPF() {

	bool TFC = false;
	bool TFA = false;
	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;

	// �_�Q�f�[�^
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(targetPath);
	//const string filename = "..\\DataBase\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);


	/*
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // �_�Q�f�[�^�̂Ƃ�
	// RANSAC ���ʏ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(targetPath);

	// �_�Q�ւ̃m�C�Y�t�^(true�ŕt�^)
	if (TFC) addingNoise2(cloud_in, maxNoiseC);
	*/


	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = cloud_lattice;

	/*
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// �����_���o
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(cloud_normals);


	// �����_�ւ̃m�C�Y�t�^
	//if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	*attention_point = *myKeypoint_normals(cloud_lattice2, keypointsXYZ);
	*/

	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice2);

	// �����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice2);

	// Harris�����_�̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);



	// PPF�̍쐬
	vector<myPPF> target_PPF = make_lightPPFs(attention_point2);



	// ���͓_�Q�Ɩ@����\��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// ���͓_�Q�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, cloud_color, "cloud_lattice");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// �~���_�̕\��
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cylinder_color(cloud_cylinder, 0, 255, 0);
	//viewer->addPointCloud(cloud_cylinder, cylinder_color, "cloud_cylinder");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud_cylinder");
	//viewer->addPointCloudNormals<pcl::PointNormal>(cloud_cylinder, 1, 3.0, "cloud_cylinder_normal");

	// �����_�̕\��
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