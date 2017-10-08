#ifndef _Detectors_H_
#define _Detectors_H_

#include "NormalDirection.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>

using namespace std;

/*
#include <pcl/point_cloud.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/narf.h>
void CRP(pcl::PointXYZ point_cloud) {
float angular_resolution = 0.5f;
int rotation_invariant = 0;
float support_size = 0.3f;
int descriptor_size = 36;
// -----------------------------------------------
// -----Create RangeImage from the PointCloud-----
// -----------------------------------------------
float noise_level = 0.0;
float min_range = 0.0f;
int border_size = 1;
boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
pcl::RangeImage& range_image = *range_image_ptr;
range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
range_image.integrateFarRanges(far_ranges);
if (setUnseenToMaxRange)
range_image.setUnseenToMaxRange();

// Extract NARF features:
std::cout << "Now extracting NARFs in every image point.\n";
std::vector<std::vector<pcl::Narf*> > narfs;
narfs.resize(range_image.points.size());
int last_percentage = -1;
for (unsigned int y = 0; y<range_image.height; ++y)
{
for (unsigned int x = 0; x<range_image.width; ++x)
{
int index = y*range_image.width + x;
int percentage = (int)((100 * index) / range_image.points.size());
if (percentage > last_percentage)
{
std::cout << percentage << "% " << std::flush;
last_percentage = percentage;
}
pcl::Narf::extractFromRangeImageAndAddToList(range_image, x, y, descriptor_size,
support_size, rotation_invariant != 0, narfs[index]);
//std::cout << "Extracted "<<narfs[index].size ()<<" features for pixel "<<x<<","<<y<<".\n";
}
}
std::cout << "100%\n";
std::cout << "Done.\n\n Now you can click on points in the image to visualize how the descriptor is "
<< "extracted and see the descriptor distances to every other point..\n";
}
*/


/*void ISS3Ddetector_sample() {
//
//  ISS3D parameters
//
double iss_salient_radius_;
double iss_non_max_radius_;
double iss_gamma_21_(0.975);
double iss_gamma_32_(0.975);
double iss_min_neighbors_(5);
int iss_threads_(4);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

// Fill in the model cloud
double model_resolution;

// Compute model_resolution
iss_salient_radius_ = 6 * model_resolution;
iss_non_max_radius_ = 4 * model_resolution;

//
// Compute keypoints
//
pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;


iss_detector.setSearchMethod(tree);
iss_detector.setSalientRadius(iss_salient_radius_);
iss_detector.setNonMaxRadius(iss_non_max_radius_);
iss_detector.setThreshold21(iss_gamma_21_);
iss_detector.setThreshold32(iss_gamma_32_);
iss_detector.setMinNeighbors(iss_min_neighbors_);
iss_detector.setNumberOfThreads(iss_threads_);
iss_detector.setInputCloud(model);
iss_detector.compute(*model_keypoints);
}*/

pcl::PointCloud<pcl::PointNormal>::Ptr ISS3Ddetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	//  ISS3D parameters
	double iss_salient_radius_ = 5.0f;
	double iss_non_max_radius_ = 3.0f;
	double iss_gamma_21_(0.975); //0.975	0.50
	double iss_gamma_32_(0.975); //0.975	0.50
	double iss_min_neighbors_(5); // 5	20
	//int iss_threads_(4);

	//pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	// Compute keypoints
	//pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;

	detector.setSearchMethod(tree);
	detector.setSalientRadius(iss_salient_radius_);
	detector.setNonMaxRadius(iss_non_max_radius_);
	detector.setThreshold21(iss_gamma_21_);
	detector.setThreshold32(iss_gamma_32_);
	detector.setMinNeighbors(iss_min_neighbors_);
	//detector.setNumberOfThreads(iss_threads_);
	detector.setInputCloud(cloud);
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);


	/////////////////////////
	/*cout << "before sorting" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート
	cout << "after sorting (descending order)" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}*/
	/*cout << "point extraction" << endl;
	int numThreshold = 20; //15
	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	if (count >= numThreshold) break;
	keypointsXYZ->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
	count++;
	//cout << "it->intensity: " << it->intensity << endl;
	}*/
	cout << "keypoints->size(): " << keypoints->size() << endl;
	/////////////////////////


	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);

	Eigen::Vector4f xyz_centroid; // 全体の重心を計算
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			//Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
			//pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}


pcl::PointCloud<pcl::PointNormal>::Ptr SUSANdetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// 点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	// 法線の生成
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud);

	//  parameters
	float radius = 3.0f; // 0.01f
	float distanceThreshold = 0.01f; // 0.001f
	float angularThreshold = 0.01f; // 0.0001f
	float intensityThreshold = 1.0f; // 7.0f 1.5f 1.0f
	bool nonmax = true; // true;
	bool validate = true;
	//int threads = 4;

	/*float radius = 1.0f; // 0.01f
	float distanceThreshold = 0.01f; // 0.001f
	float angularThreshold = 0.001f; // 0.0001f
	float intensityThreshold = 1.0f; // 7.0f 1.5f
	bool nonmax = true; // true;*/


	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());

	// Compute keypoints
	pcl::SUSANKeypoint<pcl::PointXYZ, pcl::PointXYZI> detector;
	//pcl::SUSANKeypoint<pcl::PointNormal, pcl::PointXYZI> detector;

	detector.setSearchMethod(tree);

	detector.setRadius(radius);
	detector.setDistanceThreshold(distanceThreshold);
	detector.setAngularThreshold(angularThreshold);
	detector.setIntensityThreshold(intensityThreshold);
	detector.setNonMaxSupression(nonmax);
	//detector.setGeometricValidation(validate);
	//detector.setNumberOfThreads(threads_);

	detector.setInputCloud(cloud);
	//detector.setInputCloud(cloud_normals);
	//detector.setSearchSurface(cloud_normals);
	//detector.setRadiusSearch(radius);
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	keypointsXYZ->points.resize(keypoints->points.size());
	pcl::copyPointCloud(*keypoints, *keypointsXYZ);


	/////////////////////////
	/*cout << "before sorting" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	std::sort(keypoints->begin(), keypoints->end(), compare_intensity); // 比較関数を指定してソート
	cout << "after sorting (descending order)" << endl;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	cout << "it->intensity: " << it->intensity << endl;
	}
	cout << "point extraction" << endl;
	int numThreshold = 20; //15
	int count = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
	if (count >= numThreshold) break;
	keypointsXYZ->push_back(pcl::PointXYZ((*it).x, (*it).y, (*it).z));
	count++;
	cout << "it->intensity: " << it->intensity << endl;
	}*/
	cout << "keypoints->size(): " << keypoints->size() << endl;
	/////////////////////////


	// 特徴点の法線生成
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float kd_radius = 5.0f; // 3.0f 5.0f
	kdtree.setInputCloud(cloud);

	Eigen::Vector4f xyz_centroid; // 全体の重心を計算
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
		if (kdtree.radiusSearch(*it, kd_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
				neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
			}
			//Eigen::Vector4f xyz_centroid; // 近傍の重心を計算
			//pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
			pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
			cloud_ptr->push_back(normal);
		}
	}

	return cloud_ptr;
}

#endif // _Detectors_H_