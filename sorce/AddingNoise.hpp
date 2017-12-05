#ifndef _AddingNoise_HPP_
#define _AddingNoise_HPP_

#include "MT.h"
#include "RandUnitVector.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 【ランダムノイズの付与】
// 点群データを受け取り，各点にランダムノイズを付与する

pcl::PointCloud<pcl::PointXYZ>::Ptr addingNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float max_error) {

	// ノイズを付与した点群を格納する配列
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 乱数のシード生成
	//float max_error = sqrt(maxNoise);
	init_genrand((unsigned)time(NULL));
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));

	// 乱数の付与
	for (pcl::PointXYZ point : *input_cloud) {
		// ベクトルの大きさを決定
		float error = max_error * genrand_real1();
		// ベクトルの向きを決定
		randUnitVector(sampleTest, sampleTime);
		// ランダムベクトルの生成
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// ノイズの付与
		point.x += sampleTest->x;
		point.y += sampleTest->y;
		point.z += sampleTest->z;
		output_cloud->push_back(point);
	}

	return output_cloud;
}

void addingNoise2(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float max_error) {

	// 乱数のシード生成
	init_genrand((unsigned)time(NULL));
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));

	// 出来ない
	// 乱数の付与
	/*for (pcl::PointXYZ point : *input_cloud) {
		// ベクトルの大きさを決定
		float error = max_error * genrand_real1();
		// ベクトルの向きを決定
		randUnitVector(sampleTest, sampleTime);
		// ランダムベクトルの生成
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// ノイズの付与
		point.x += sampleTest->x;
		point.y += sampleTest->y;
		point.z += sampleTest->z;
	}*/
	
	// 出来る
	// 乱数の付与
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		// ベクトルの大きさを決定
		float error = max_error * genrand_real1();
		//float error = max_error;
		//cout << "genrand_real1(): " << genrand_real1() << endl;
		// ベクトルの向きを決定
		randUnitVector(sampleTest, sampleTime);
		// ランダムベクトルの生成
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// ノイズの付与
		it->x += sampleTest->x;
		it->y += sampleTest->y;
		it->z += sampleTest->z;
	}

	// 出来る
	// 乱数の付与
	/*for (int i = 0; i < input_cloud->size();i++) {
		// ベクトルの大きさを決定
		float error = max_error * genrand_real1();
		// ベクトルの向きを決定
		randUnitVector(sampleTest, sampleTime);
		// ランダムベクトルの生成
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		// ノイズの付与
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


// オフセットのクラス作って渡した方がよい
void addingOffset(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, OFFSET offset) {
	// オフセットの付与
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		it->x += offset.x;
		it->y += offset.y;
		it->z += offset.z;
	}
}

void addingOffsetN(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud, OFFSET offset) {
	// オフセットの付与
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = input_cloud->begin(); it != input_cloud->end(); it++) {
		it->x += offset.x;
		it->y += offset.y;
		it->z += offset.z;
	}
}

#endif // _AddingNoise_HPP_