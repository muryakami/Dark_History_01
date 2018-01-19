#ifndef _MakeAccuracyFile_HPP_
#define _MakeAccuracyFile_HPP_

#include "Output.h"
#include "PPF.hpp"
#include "PPFio.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ��v���̏o��
/*void outputAccuracy2(vector<myPPF>* source_PPF, vector<myPPF>* target_PPF) {
	int match_cloud = myLightPPF_matching(*source_PPF, *target_PPF);
	accuracy_file << "accuracy: " << (float)match_cloud / target_PPF->size() * 100 << " [%]" << endl;
	accuracy_file << endl;
}*/


void makeAccuracyFile(vector<myPPF> measurement_PPF) {

	// PPF�f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\PPF");

	// AccuracyFile���o�͂���p�X�ϐ�
	fs::path fileO("..\\DataBase\\Accuracy\\accuracy000.txt");
	fs::ofstream savefile(fileO);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			// ���݂̃p�X
			//filenameIO << filename << std::endl;

			// ���f��PPF�̃��[�h
			vector<myPPF> model_PPF = load_MyLightPPF(filename.string());

			// ��v���̏o��
			//outputAccuracy2(&model_PPF, &measurement_PPF);	// A��A
			int match_cloud = myLightPPF_matching(measurement_PPF, model_PPF);
			savefile << filename << endl;
			savefile << "measurement_PPF.size: " << measurement_PPF.size() << endl;
			savefile << "model_PPF.size: " << model_PPF.size() << endl;
			savefile << "match_cloud: " << match_cloud << endl;
			savefile << "accuracy: " << (float)match_cloud / model_PPF.size() * 100 << " [%]" << endl;
			savefile << endl;

		}
	}

}


#endif // _MakeAccuracyFile_HPP_