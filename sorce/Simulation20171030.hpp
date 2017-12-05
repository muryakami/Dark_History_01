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


// �y�}�b�`���O��@�̐��\�]�������z
// ���o���������_�Ƀm�C�Y�������C��v�����v�Z����

vector<myPPF> procedure(string filename, boolean TFC, boolean TFA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {

	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;

	std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
	double elapsed;

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);

	// true�Ńm�C�Y�t�^
	if(TFC) addingNoise2(cloud_in, maxNoiseC);

	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �@���̐���
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// �����_���o
	start = std::chrono::system_clock::now(); // �v���J�n����
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	end = std::chrono::system_clock::now();  // �v���I������

	// �����_�ւ̃m�C�Y�t�^
	if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// �����_�̖@������
	*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);

	// �������Ԃ̏o��
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;

	return make_lightPPFs(attention_point);
}


// ��v���̏o��
void outputAccuracy(vector<myPPF>* source_PPF, vector<myPPF>* target_PPF) {
	int match_cloud = myLightPPF_matching(*source_PPF, *target_PPF);
	accuracy_file << "accuracy: " << (float)match_cloud / target_PPF->size() * 100 << " [%]" << endl;
	accuracy_file << endl;
}


void simulation20171030() {
	// �H��f�[�^�ǂݍ���
	const string filename = ".\\STL files\\NikonTurningTool.STL";

	// PPF�̐���(STL�f�[�^:A)	�\�[�X
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> source_PPF = procedure(filename, false, false, cloud_lattice, attention_point);

	// PPF�̐���(STL�f�[�^:A)	�^�[�Q�b�g
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2(new pcl::PointCloud<pcl::PointNormal>);
	// �_�Q�덷
	vector<myPPF> target_PPF = procedure(filename, true, false, cloud_lattice2, attention_point2);
	// �����_�덷
	//vector<myPPF> target_PPF = procedure(filename, false, true, cloud_lattice2, attention_point2);

	// �덷�̏o��
	nearest_search_test3(attention_point, attention_point2);

	// ��v���̏o��
	outputAccuracy(&source_PPF, &target_PPF);	// A��A



	// ���͓_�Q�Ɩ@����\��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// ���͓_�Q�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lattice_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, lattice_color, "cloud_lattice2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice2");

	// �����_�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color2(attention_point2, 0, 255, 0);
	viewer->addPointCloud(attention_point2, attention_color2, "attention_point2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point2");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_point2n");

	// �\�����̃p�����[�^
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


#endif // _Simulation20171030_HPP_