#ifndef _FeatureDescription_HPP_
#define _FeatureDescription_HPP_

#include "Output.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// 特徴量計算
// pcl::FPFHSignature33 バージョン
pcl::PointCloud<pcl::FPFHSignature33>::Ptr myFeatureDescription_FPFH2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());

	float radius = 0.3f;// 3.0f
						// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::FPFHSignature33>::Ptr descriptor(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius * 10);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	return features;
}

// 対応点探索
std::vector<int> myFindMatching2(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features) {

	std::vector<int> correspondences;
	////
	/*std::vector<pair<int, int>> correspondences2;*/
	////

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	correspondences.resize(source_features->size());

	pcl::KdTreeFLANN<pcl::FPFHSignature33> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); ++i) {
		correspondences[i] = -1; // -1 means no correspondence

		if (isnan(source_features->points[i].histogram[0])) continue;

		//元々
		search_tree.nearestKSearch(*source_features, i, 1, index, L2_distance);
		correspondences[i] = index[0];

		////自分
		/*int K = 5;
		//if (search_tree.nearestKSearch(source_features->at(i).histogram, K, index, L2_distance) > 0) {
		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
		for (int j = 0; j < index.size(); j++) {
		correspondences2.push_back(make_pair(i, index[j]));
		}
		}*/
		////
	}

	////
	/*for (std::vector<pair<int, int>>::iterator it = correspondences2.begin(); it != correspondences2.end(); it++) {
	accuracy_file << "[" << it->first << "]: " << it->second << endl;
	}*/

	/*outputfile << "source_features" << endl;
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = source_features->begin(); it != source_features->end(); it++) {
	for (int i = 0; i < it->descriptorSize(); i++) {
	outputfile << "[" << i << "]: " << it->histogram[i] << " ";
	}
	outputfile << endl;
	}
	outputfile << endl;
	outputfile << "target_features" << endl;
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
	for (int i = 0; i < it->descriptorSize(); i++) {
	outputfile << "[" << i << "]: " << it->histogram[i] << " ";
	}
	outputfile << endl;
	}*/

	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = source_features->begin(); it != source_features->end(); it++) {
		double hissum = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			hissum += it->histogram[i];
		}
		for (int i = 0; i < it->descriptorSize(); i++) {
			it->histogram[i] /= hissum;
		}
	}
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
		double hissum = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			hissum += it->histogram[i];
		}
		for (int i = 0; i < it->descriptorSize(); i++) {
			it->histogram[i] /= hissum;
		}
	}
	for (pcl::PointCloud<pcl::FPFHSignature33>::iterator it = target_features->begin(); it != target_features->end(); it++) {
		double result = 0.0f;
		for (int i = 0; i < it->descriptorSize(); i++) {
			result += it->histogram[i] * source_features->points[0].histogram[i];
		}
		outputfile << "result: " << result << endl;
	}

	////

	return correspondences;
}

// 特徴量計算
// pcl::SHOT352 バージョン
pcl::PointCloud<pcl::SHOT352>::Ptr myFeatureDescription_SHOT352(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<pcl::SHOT352>::Ptr features(new pcl::PointCloud<pcl::SHOT352>());

	float radius = 0.3f;// 3.0f 0.3f
						// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::SHOT352>::Ptr descriptor(new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius * 10);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	return features;
}

// 対応点探索
std::vector<int> myFindMatching_SHOT352(pcl::PointCloud<pcl::SHOT352>::Ptr source_features, pcl::PointCloud<pcl::SHOT352>::Ptr target_features) {

	std::vector<int> correspondences;

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	correspondences.resize(source_features->size());

	pcl::KdTreeFLANN<pcl::SHOT352> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); ++i) {
		correspondences[i] = -1; // -1 means no correspondence

		if (isnan(source_features->points[i].descriptor[0])) continue;

		search_tree.nearestKSearch(*source_features, i, 1, index, L2_distance);
		correspondences[i] = index[0];
	}

	return correspondences;
}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching2(std::vector<int> correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	//cout << "correspondences.size(): " << correspondences.size() << endl;

	int nCorrespondence = 0;
	for (int i = 0; i < correspondences.size(); i++) {
		//cout << "correspondences[i]: " << correspondences[i] << endl;
		if (correspondences[i] >= 0) nCorrespondence++; // do not count "-1" in correspondences
	}

	pCorrespondences->resize(nCorrespondence);
	for (int i = 0, j = 0; i < correspondences.size(); i++)
	{
		if (correspondences[i] > 0)
		{
			//cout << "(*pCorrespondences)[" << j << "].distance_B: " << (*pCorrespondences)[j].distance << endl;
			(*pCorrespondences)[j].index_query = i;
			(*pCorrespondences)[j].index_match = correspondences[i];
			j++;
			//cout << "(*pCorrespondences)[" << j << "].distance_B: " << (*pCorrespondences)[j].distance << endl;
		}
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);

	return pCorrespondences;
}

// 対応点探索
pcl::CorrespondencesPtr myFindMatching_SHOT352_2(pcl::PointCloud<pcl::SHOT352>::Ptr source_features, pcl::PointCloud<pcl::SHOT352>::Ptr target_features) {

	int K = 1; // 3 5
	//std::vector<pair<int, int>> correspondences;
	//correspondences.reserve(source_features->size()*K);
	//std::vector<pair<int, int>> correspondences(source_features->size()*K, make_pair(-1, -1));  // -1 means no correspondence

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	correspondences->reserve(source_features->size()*K);

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	pcl::KdTreeFLANN<pcl::SHOT352> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); i++) {

		if (isnan(source_features->points[i].descriptor[0])) continue;

		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
			for (int j = 0; j < K; j++)
				correspondences->push_back(pcl::Correspondence(i, index[j], L2_distance[j]));
		}
	}

	/*for (int i = 0; i < correspondences->size();i++) {
	accuracy_file << "i: " << correspondences->at(i).index_query << "\tj: " << correspondences->at(i).index_match << "\tdistance: " << correspondences->at(i).distance << endl;
	}*/

	/*for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;*/

	return correspondences;
}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching2_2(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	/*int nCorrespondence = 0;
	for (int i = 0; i < correspondences.size(); i++) {
	if (correspondences[i].first >= 0) nCorrespondence++; // do not count "-1" in correspondences
	}

	pCorrespondences->resize(nCorrespondence);
	for (int i = 0, j = 0; i < correspondences.size(); i++) {
	if (correspondences[i].first >= 0) {
	(*pCorrespondences)[j].index_query = i;
	(*pCorrespondences)[j].index_match = correspondences[i];
	j++;
	}
	}*/

	pCorrespondences->reserve(correspondences->size());
	for (pcl::Correspondence it : *correspondences)
		pCorrespondences->push_back(it);

	/*pCorrespondences->reserve(correspondences->size());
	for (pcl::Correspondence it : *correspondences)
	pCorrespondences->push_back(pcl::Correspondence(it.index_query, it.index_match, FLT_MAX));*/


	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);

	return pCorrespondences;
}

// RとTの推定
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
Eigen::Matrix4f myEstimatingTransformation2(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features) {

	Eigen::Matrix4f transformation;

	// Estimating rigid transformation
	std::cout << "Estimating transformation" << std::endl;

	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;

	std::vector<int> source_index(source_features->size());
	for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;

	return transformation;

	/*
	est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	std::cout << transformation << std::endl;

	pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	*/

}

#endif // _FeatureDescription_HPP_