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

	std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
	double elapsed;

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
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


#endif // _ForFixPoint_HPP_