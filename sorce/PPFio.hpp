#ifndef _PPFio_HPP_
#define _PPFio_HPP_

#include "Output.h"
#include "PPF.hpp"
#include "InterpolationSTL.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>

void save_MyLightPPF(string filename) {

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �@���̐���
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // 08
	// �����_���o�C�����ʌv�Z
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals); // 08

	// �@���̐����C�����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction_HarrisN(cloud_lattice); // Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = ISS3Ddetector(cloud_lattice); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = SUSANdetector(cloud_lattice); // SUSAN

	// �����_���o�C�����ʌv�Z
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice); // ����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr attention_point = myFeaturePointExtraction_HarrisN2(cloud_normals); // ����
	//pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(attention_point); // ���� ���ړ_�̏d�S(�ߖT�_�̏d�S�ł���Ȃ�)

	// �H��f�[�^��PPF
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

	// �H��f�[�^���
	accuracy_file << filename << endl;

	// �f�[�^�}�b�`���O (�ȈՔ�)
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	accuracy_file << "target_PPF->size: " << target_PPF.size() << endl; //
	accuracy_file << "source_PPF->size: " << source_PPF.size() << endl; //
	accuracy_file << "match_cloud->size: " << match_cloud << endl;
	accuracy_file << "target_accuracy: " << (float)match_cloud / target_PPF.size() * 100 << endl;
	accuracy_file << "source_accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	accuracy_file << endl;
}

#endif // _PPFio_HPP_