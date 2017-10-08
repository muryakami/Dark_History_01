#ifndef _CorrespondenceGrouping_HPP_
#define _CorrespondenceGrouping_HPP_

#include <string>
#include <vector>
#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>
#include "Output.h"
#include "InterpolationSTL.hpp"
#include "SurfaceNormals.hpp"
#include "PPF.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include "FeatureDescription.hpp"
#include "KeypointNormals.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;

using namespace std;

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
//float model_ss_(0.01f);
//float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f); //0.02f
float cg_size_(0.01f);
float cg_thresh_(5.0f);

float cloud_ss_(0.03f);

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i) {
		if (!pcl_isfinite((*cloud)[i].x)) continue;

		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2) {
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}

	if (n_points != 0) res /= n_points;

	return res;
}

//
//  Load clouds
//
pcl::PointCloud<pcl::PointXYZ>::Ptr loadClouds(string filename) {

	// 点群情報
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	return cloud_ptr;
}

//
//  Set up resolution invariance
//
void setUpResolutionInvariance(pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud) {

	if (use_cloud_resolution_) {
		float resolution = static_cast<float> (computeCloudResolution(model_cloud));
		if (resolution != 0.0f)
		{
			//model_ss_ *= resolution;
			//scene_ss_ *= resolution;
			rf_rad_ *= resolution;
			descr_rad_ *= resolution;
			cg_size_ *= resolution;

			cloud_ss_ *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		//std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		//std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;

		std::cout << "Cloud sampling size:    " << cloud_ss_ << std::endl;
	}

	return;
}

//
//  Compute Normals
//
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//法線情報を入れる変数
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud);
	norm_est.compute(*normals);

	return normals;
}

//
//  Downsample Clouds to Extract keypoints
//
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudsToExtractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	//特徴点情報を入れる変数
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(cloud_ss_);
	uniform_sampling.filter(*keypoints);
	std::cout << "Cloud total points: " << cloud->size() << "; Selected Keypoints: " << keypoints->size() << std::endl;

	return keypoints;
}

class CLOUDDATA //点群, 法線, キーポイント
{
private: // メンバ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;
public: // コンストラクタ
	CLOUDDATA() {};
	CLOUDDATA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
		this->cloud = cloud;
		this->normals = normals;
		this->keypoints = keypoints;
	};
public: // メソッド
	void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		this->cloud = cloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() {
		return this->cloud;
	}
	void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals) {
		this->normals = normals;
	}
	pcl::PointCloud<pcl::Normal>::Ptr getNormals() {
		return this->normals;
	}
	void setKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
		this->keypoints = keypoints;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoints() {
		return this->keypoints;
	}
};


//
//  Compute Descriptor for keypoints
//
//pcl::PointCloud<pcl::SHOT352>::Ptr computeDescriptorForKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
pcl::PointCloud<pcl::SHOT352>::Ptr computeDescriptorForKeypoints(CLOUDDATA data) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = data.getCloud();
	pcl::PointCloud<pcl::Normal>::Ptr normals = data.getNormals();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = data.getKeypoints();

	//特徴量情報を入れる変数
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(keypoints);
	descr_est.setInputNormals(normals);
	descr_est.setSearchSurface(cloud);
	descr_est.compute(*descriptors);

	return descriptors;
}

//
//  Find Model-Scene Correspondences with KdTree
//
pcl::CorrespondencesPtr findModelSceneCorrespondencesWithKdTree(pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors, pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors) {

	//対応点情報を入れる変数
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<pcl::SHOT352> match_search;
	match_search.setInputCloud(model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	return model_scene_corrs;
}

//
//  Compute (Keypoints) Reference Frames only for Hough
//
//pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeReferenceFramesOnlyForHough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeReferenceFramesOnlyForHough(CLOUDDATA data) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = data.getCloud();
	pcl::PointCloud<pcl::Normal>::Ptr normals = data.getNormals();
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = data.getKeypoints();

	pcl::PointCloud<pcl::ReferenceFrame>::Ptr cloud_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(rf_rad_);

	rf_est.setInputCloud(keypoints);
	rf_est.setInputNormals(normals);
	rf_est.setSearchSurface(cloud);
	rf_est.compute(*cloud_rf);

	return cloud_rf;
}

class ActualClustering
{
private: // メンバ
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
public: // コンストラクタ
	ActualClustering() {};
	ActualClustering(vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations, vector<pcl::Correspondences> clustered_corrs) {
		this->rototranslations = rototranslations;
		this->clustered_corrs = clustered_corrs;
	};
public: // メソッド
	void setRototranslations(vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations) {
		this->rototranslations = rototranslations;
	}
	vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > getRototranslations() {
		return this->rototranslations;
	}
	void setClustered_corrs(vector<pcl::Correspondences> clustered_corrs) {
		this->clustered_corrs = clustered_corrs;
	}
	vector<pcl::Correspondences> getClustered_corrs() {
		return this->clustered_corrs;
	}
};

//
//  Actual Clustering
//
ActualClustering actualClustering(CLOUDDATA model_data, CLOUDDATA scene_data, pcl::CorrespondencesPtr model_scene_corrs) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = model_data.getKeypoints();
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = scene_data.getKeypoints();

	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_)
	{
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		/*pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

		pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);

		rf_est.setInputCloud(model_keypoints);	//	分割すればなんとかなるんじゃね？
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model_cloud);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene_cloud);
		rf_est.compute(*scene_rf);*/

		pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf = computeReferenceFramesOnlyForHough(model_data);
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf = computeReferenceFramesOnlyForHough(scene_data);



		//  Clustering
		pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize(rototranslations, clustered_corrs);
	}
	else // Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
	}

	return ActualClustering(rototranslations, clustered_corrs);
}

class RotationAndTranslation
{
private: // メンバ
	Eigen::Matrix3f rotation;
	Eigen::Vector3f translation;
public: // コンストラクタ
	RotationAndTranslation() {};
	RotationAndTranslation(Eigen::Matrix3f rotation, Eigen::Vector3f translation) {
		this->rotation = rotation;
		this->translation = translation;
	};
public: // メソッド
	void setRotation(Eigen::Matrix3f rotation) {
		this->rotation = rotation;
	}
	Eigen::Matrix3f getRotation() {
		return this->rotation;
	}
	void setTranslation(Eigen::Vector3f translation) {
		this->translation = translation;
	}
	Eigen::Vector3f getTranslation() {
		return this->translation;
	}
};

//
//  Output results
//
RotationAndTranslation outputResults(ActualClustering ac) {

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = ac.getRototranslations();
	std::vector<pcl::Correspondences> clustered_corrs = ac.getClustered_corrs();

	RotationAndTranslation rt;

	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		rt.setRotation(rotation);
		rt.setTranslation(translation);
	}

	return rt;
}


void CorrespondenceGrouping() {

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = myFeatureDescription_FPFH2(cloud_normals, source_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr source_features = myFeatureDescription_SHOT352(cloud_normals, source_keypointsXYZ);

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // trueでノイズ true
	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_lattice2);
	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	// 特徴量計算
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = myFeatureDescription_FPFH2(cloud_normals2, target_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr target_features = myFeatureDescription_SHOT352(cloud_normals2, target_keypointsXYZ);




	// 対応点探索
	//std::vector<int> correspondences = myFindMatching2(source_features, target_features);
	//std::vector<int> correspondences = myFindMatching_SHOT352(source_features, target_features);
	pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF = make_lightPPFs(attention_point);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF = make_lightPPFs(attention_point2);
	int match_cloud = myLightPPF_matching(keypoint_PPF, stl_PPF);

	outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud << endl;
	outputfile777 << "stl_accuracy: " << (float)match_cloud / stl_PPF.size() * 100 << endl;
	outputfile777 << "keypoint_accuracy: " << (float)match_cloud / keypoint_PPF.size() * 100 << endl;

	outputfile777 << endl;

	// 誤対応除去
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	// RとTの推定
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);

	// 特徴点の法線生成
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point3 = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF2 = make_lightPPFs(attention_point3);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point4 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF2 = make_lightPPFs(attention_point4);
	int match_cloud2 = myLightPPF_matching(keypoint_PPF2, stl_PPF2);

	outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF2.size() << endl;
	outputfile777 << "stl_PPF.size(): " << stl_PPF2.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud2 << endl;
	outputfile777 << "stl_accuracy: " << (float)match_cloud2 / stl_PPF2.size() * 100 << endl;
	outputfile777 << "keypoint_accuracy: " << (float)match_cloud2 / keypoint_PPF2.size() * 100 << endl;

	outputfile777 << endl;



	/*for (int i = 0; i < correspondences.size(); i++) {
	outputfile777 << "[" << i << "]: " << correspondences[i] << endl;
	}
	outputfile777 << endl;

	for (int i = 0; i < pCorrespondences->size(); i++) {
	outputfile777 << "[" << i << "]: " << pCorrespondences->at(i) << endl;
	}
	outputfile777 << endl;*/


	for (pcl::Correspondence i : *correspondences) {
		outputfile777 << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile777 << endl;

	for (pcl::Correspondence i : *pCorrespondences) {
		outputfile777 << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile777 << endl;
}


void CorrespondenceGrouping2() {

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";


	// CLOUDDATA model_data2 = CLOUDDATA();
	// STLデータの内挿補間 (モデルデータ) false(ノイズなし)
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud = interpolation_stl(filename, false);
	// 法線の生成
	pcl::PointCloud<pcl::Normal>::Ptr model_normals = computeNormals(model_cloud);
	// 特徴点抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = downsampleCloudsToExtractKeypoints(model_cloud);
	// 特徴量計算
	CLOUDDATA model_data(model_cloud, model_normals, model_keypoints);
	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors = computeDescriptorForKeypoints(model_data);


	// CLOUDDATA scene_data2 = CLOUDDATA();
	// STLデータの内挿補間 (シーンデータ) true(ノイズあり)
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud = interpolation_stl(filename, true);
	// 法線の生成
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals = computeNormals(scene_cloud);
	// 特徴点抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = downsampleCloudsToExtractKeypoints(scene_cloud);
	// 特徴量計算
	CLOUDDATA scene_data(scene_cloud, scene_normals, scene_keypoints);
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors = computeDescriptorForKeypoints(scene_data);


	// 対応点探索
	pcl::CorrespondencesPtr model_scene_corrs = findModelSceneCorrespondencesWithKdTree(model_descriptors, scene_descriptors);
	// 実際のクラスタリング
	ActualClustering ac = actualClustering(model_data, scene_data, model_scene_corrs);
	// 回転行列と平行移動の計算
	RotationAndTranslation rt = outputResults(ac);


	return;
}



#endif // _CorrespondenceGrouping_HPP_