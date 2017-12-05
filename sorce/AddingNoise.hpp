#ifndef _AddingNoise_HPP_
#define _AddingNoise_HPP_

#include "MT.h"
#include "RandUnitVector.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// �y�����_���m�C�Y�̕t�^�z
// �_�Q�f�[�^���󂯎��C�e�_�Ƀ����_���m�C�Y��t�^����

pcl::PointCloud<pcl::PointXYZ>::Ptr addingNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float max_error) {

	// �m�C�Y��t�^�����_�Q���i�[����z��
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// �����̃V�[�h����
	//float max_error = sqrt(maxNoise);
	init_genrand((unsigned)time(NULL));
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));

	// �����̕t�^
	for (pcl::PointXYZ point : *input_cloud) {
		// �x�N�g���̑傫��������
		float error = max_error * genrand_real1();
		// �x�N�g���̌���������
		randUnitVector(sampleTest, sampleTime);
		// �����_���x�N�g���̐���
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// �m�C�Y�̕t�^
		point.x += sampleTest->x;
		point.y += sampleTest->y;
		point.z += sampleTest->z;
		output_cloud->push_back(point);
	}

	return output_cloud;
}

void addingNoise2(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float max_error) {

	// �����̃V�[�h����
	init_genrand((unsigned)time(NULL));
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));

	// �o���Ȃ�
	// �����̕t�^
	/*for (pcl::PointXYZ point : *input_cloud) {
		// �x�N�g���̑傫��������
		float error = max_error * genrand_real1();
		// �x�N�g���̌���������
		randUnitVector(sampleTest, sampleTime);
		// �����_���x�N�g���̐���
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// �m�C�Y�̕t�^
		point.x += sampleTest->x;
		point.y += sampleTest->y;
		point.z += sampleTest->z;
	}*/
	
	// �o����
	// �����̕t�^
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		// �x�N�g���̑傫��������
		float error = max_error * genrand_real1();
		//float error = max_error;
		//cout << "genrand_real1(): " << genrand_real1() << endl;
		// �x�N�g���̌���������
		randUnitVector(sampleTest, sampleTime);
		// �����_���x�N�g���̐���
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// �m�C�Y�̕t�^
		it->x += sampleTest->x;
		it->y += sampleTest->y;
		it->z += sampleTest->z;
	}

	// �o����
	// �����̕t�^
	/*for (int i = 0; i < input_cloud->size();i++) {
		// �x�N�g���̑傫��������
		float error = max_error * genrand_real1();
		// �x�N�g���̌���������
		randUnitVector(sampleTest, sampleTime);
		// �����_���x�N�g���̐���
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// �m�C�Y�̕t�^
		input_cloud->at(i).x += sampleTest->x;
		input_cloud->at(i).y += sampleTest->y;
		input_cloud->at(i).z += sampleTest->z;
	}*/
	

}

typedef struct _offset {
	float x;
	float y;
	float z;
} OFFSET;


// �I�t�Z�b�g�̃N���X����ēn���������悢
void addingOffset(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, OFFSET offset) {
	// �I�t�Z�b�g�̕t�^
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		it->x += offset.x;
		it->y += offset.y;
		it->z += offset.z;
	}
}

void addingOffsetN(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, OFFSET offset) {
	// �I�t�Z�b�g�̕t�^
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		it->x += offset.x;
		it->y += offset.y;
		it->z += offset.z;
	}
}

#endif // _AddingNoise_HPP_