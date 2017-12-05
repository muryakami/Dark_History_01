#ifndef _FeatureDescription_HPP_
#define _FeatureDescription_HPP_

#include "Output.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

// 特徴量計算
// pcl::FPFHSignature33 バージョン
pcl::PointCloud<pcl::FPFHSignature33>::Ptr myFeatureDescription_FPFH2(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());

	float radius = 3.0f;// 3.0f
	// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::FPFHSignature33>::Ptr descriptor(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius);

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

	float radius = 3.0f;// 3.0f
	// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, pcl::SHOT352>::Ptr descriptor(new pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>());
	//pcl::Feature<pcl::PointXYZ, pcl::SHOT352>::Ptr descriptor(new pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	// デバッグのための出力
	processing_time << "cloud_normals->size(): " << cloud_normals->size() << endl;
	processing_time << "keypointsXYZ->size(): " << keypointsXYZ->size() << endl;
	processing_time << "features->size(): " << features->size() << endl;
	processing_time << endl;
	processing_time << endl;

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
	for (int i = 0; i < correspondences.size(); i++)
		cout << "correspondences[" << i << "]: " << correspondences[i] << endl;



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


	//cout << "nCorrespondence: " << nCorrespondence << endl;
	//cout << "pCorrespondences->size(): " << pCorrespondences->size() << endl;


	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);

	return pCorrespondences;
}

class CorrespondenceCompareAscending {
public:
	bool operator()(const pcl::Correspondence &left, const pcl::Correspondence &right) const {
		if (left.distance < right.distance) return true;
		if (left.distance > right.distance) return false;
		return false;
	}
};

// 対応点探索
pcl::CorrespondencesPtr myFindMatching_SHOT352_2(pcl::PointCloud<pcl::SHOT352>::Ptr source_features, pcl::PointCloud<pcl::SHOT352>::Ptr target_features) {

	int isnan_count = 0;

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

		/*if (isnan(source_features->points[i].descriptor[0])) {
			isnan_count++;
			cout << "isnan_count: " << isnan_count << endl;
			continue;
		}*/

		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
			for (int j = 0; j < K; j++) {
				correspondences->push_back(pcl::Correspondence(i, index[j], L2_distance[j]));
			}
		}
	}

	for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;

	/*outputfile << "ソート前: " << endl;
	for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;
	outputfile << endl;

	std::sort(correspondences->begin(), correspondences->end(), CorrespondenceCompareAscending());*/

	/*for (int i = 0; i < correspondences->size();i++) {
		accuracy_file << "i: " << correspondences->at(i).index_query << "\tj: " << correspondences->at(i).index_match << "\tdistance: " << correspondences->at(i).distance << endl;
	}*/

	/*outputfile << "ソート後: " << endl;
	for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;
	outputfile << endl;*/

	return correspondences;


	/*pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);
	pCorrespondences->reserve(source_features->size()*K);
	std::sort(correspondences->begin(), correspondences->end(), CorrespondenceCompareAscending());
	for (pcl::Correspondence it : *correspondences) {
		if (it.distance > 1.0f) continue;
		pCorrespondences->push_back(it);
	}

	return pCorrespondences;*/

}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching2_2(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointNormal>::Ptr source_keypoints, pcl::PointCloud<pcl::PointNormal>::Ptr target_keypoints) {
//pcl::CorrespondencesPtr myRefiningMatching2_2(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	for (int i = 0; i < correspondences->size(); i++)
		cout << "correspondences[" << i << "]: " << correspondences->at(i) << endl;


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


	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> refine;
	//pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInputSource(source_keypoints);
	refine.setInputTarget(target_keypoints);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences);





	for (pcl::Correspondence it : *pCorrespondences)
		accuracy_file << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;

	return pCorrespondences;
}

// RとTの推定
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
Eigen::Matrix4f myEstimatingTransformation2(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features, pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal, pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal, pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {

	Eigen::Matrix4f transformation;

	// Estimating rigid transformation
	std::cout << "Estimating transformation" << std::endl;

	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;

	std::vector<int> source_index(source_features->size());
	for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;

	est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
	std::cout << transformation << std::endl;

	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);

	return transformation;

	/*
	est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	std::cout << transformation << std::endl;

	pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	*/

}


using PointType = pcl::PointXYZ;
//using PointType = pcl::PointNormal;
using DescriptorType = pcl::SHOT352;
//using DescriptorType = pcl::FPFHSignature33;
using EstimationType = pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>;
//using EstimationType = pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>;
//using EstimationType = pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352>;
//using EstimationType = pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>;

// 特徴量計算
pcl::PointCloud<DescriptorType>::Ptr myFeatureDescription_3(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<DescriptorType>::Ptr features(new pcl::PointCloud<DescriptorType>());

	float radius = 5.0f;// 3.0f
	// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, DescriptorType>::Ptr descriptor(new EstimationType());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, DescriptorType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, DescriptorType> > (descriptor);
	
	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	// デバッグのための出力
	processing_time << "cloud_normals->size(): " << cloud_normals->size() << endl;
	processing_time << "keypointsXYZ->size(): " << keypointsXYZ->size() << endl;
	processing_time << "features->size(): " << features->size() << endl;
	processing_time << endl;
	processing_time << endl;

	return features;
}

// 特徴量計算
pcl::PointCloud<DescriptorType>::Ptr myFeatureDescription_4(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointNormal>::Ptr keypointsNormal) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_normals, *cloud);
	pcl::copyPointCloud(*cloud_normals, *normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypointsNormal, *keypointsXYZ);

	// 特徴量計算
	pcl::PointCloud<DescriptorType>::Ptr features(new pcl::PointCloud<DescriptorType>());

	float radius = 5.0f;// 3.0f
	// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointXYZ, DescriptorType>::Ptr descriptor(new EstimationType());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	descriptor->setRadiusSearch(radius);

	pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, DescriptorType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZ, pcl::Normal, DescriptorType> > (descriptor);

	descriptor->setSearchSurface(cloud);
	descriptor->setInputCloud(keypointsXYZ);
	feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	// デバッグのための出力
	processing_time << "cloud_normals->size(): " << cloud_normals->size() << endl;
	processing_time << "keypointsXYZ->size(): " << keypointsXYZ->size() << endl;
	processing_time << "features->size(): " << features->size() << endl;
	processing_time << endl;
	processing_time << endl;

	return features;
}
/*
// 特徴量計算
//using EstimationType2 = pcl::SHOTEstimationOMP<pcl::PointNormal, pcl::Normal, pcl::SHOT352>;
using EstimationType2 = pcl::SHOTEstimationOMP<pcl::PointNormal, pcl::SHOT352>;
pcl::PointCloud<DescriptorType>::Ptr myFeatureDescription_5(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointNormal>::Ptr keypointsNormal) {

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::copyPointCloud(*cloud_normals, *cloud);
	//pcl::copyPointCloud(*cloud_normals, *normals);

	// 特徴量計算
	pcl::PointCloud<DescriptorType>::Ptr features(new pcl::PointCloud<DescriptorType>());

	float radius = 5.0f;// 3.0f
	// descriptor
	std::cout << "description" << std::endl;
	pcl::Feature<pcl::PointNormal, DescriptorType>::Ptr descriptor(new EstimationType2());
	descriptor->setSearchMethod(pcl::search::Search<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
	descriptor->setRadiusSearch(radius);

	//pcl::FeatureFromNormals<pcl::PointNormal, pcl::Normal, DescriptorType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointNormal, pcl::Normal, DescriptorType> > (descriptor);

	descriptor->setSearchSurface(cloud_normals);
	descriptor->setInputCloud(keypointsNormal);
	//feature_from_normals->setInputNormals(normals);
	descriptor->compute(*features);

	// デバッグのための出力
	processing_time << "cloud_normals->size(): " << cloud_normals->size() << endl;
	processing_time << "keypointsNormal->size(): " << keypointsNormal->size() << endl;
	processing_time << "features->size(): " << features->size() << endl;
	processing_time << endl;
	processing_time << endl;

	return features;
}
*/
// 対応点探索
pcl::CorrespondencesPtr myFindMatching_3(pcl::PointCloud<DescriptorType>::Ptr source_features, pcl::PointCloud<DescriptorType>::Ptr target_features) {

	int isnan_count = 0;

	int K = 1; // 3 5

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	correspondences->reserve(source_features->size()*K);

	// Find matching with Kd-tree
	std::cout << "matching" << std::endl;

	pcl::KdTreeFLANN<DescriptorType> search_tree;
	search_tree.setInputCloud(target_features);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);
	for (int i = 0; i < source_features->size(); i++) { // i=0

		if (isnan(source_features->points[i].descriptor[0])) continue;
		//if (isnan(source_features->points[i].histogram[0])) continue;

		if (search_tree.nearestKSearch(*source_features, i, K, index, L2_distance) > 0) {
			for (int j = 0; j < K; j++) { // j=0
				correspondences->push_back(pcl::Correspondence(i, index[j], L2_distance[j]));
			}
		}
	}

	for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;

	/*outputfile << "ソート前: " << endl;
	for (pcl::Correspondence it : *correspondences)
	outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;
	outputfile << endl;

	std::sort(correspondences->begin(), correspondences->end(), CorrespondenceCompareAscending());*/

	/*for (int i = 0; i < correspondences->size();i++) {
	accuracy_file << "i: " << correspondences->at(i).index_query << "\tj: " << correspondences->at(i).index_match << "\tdistance: " << correspondences->at(i).distance << endl;
	}*/

	/*outputfile << "ソート後: " << endl;
	for (pcl::Correspondence it : *correspondences)
	outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;
	outputfile << endl;*/

	return correspondences;


	/*pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);
	pCorrespondences->reserve(source_features->size()*K);
	std::sort(correspondences->begin(), correspondences->end(), CorrespondenceCompareAscending());
	for (pcl::Correspondence it : *correspondences) {
		if (it.distance > 1.0f) continue;
		pCorrespondences->push_back(it);
	}

	return pCorrespondences;*/

}

// 誤対応除去
pcl::CorrespondencesPtr myRefiningMatching_3(pcl::CorrespondencesPtr source_correspondences, pcl::CorrespondencesPtr target_correspondences, pcl::PointCloud<pcl::PointNormal>::Ptr source_keypoints, pcl::PointCloud<pcl::PointNormal>::Ptr target_keypoints) {

	pcl::CorrespondencesPtr pCorrespondences(new pcl::Correspondences);
	pCorrespondences->reserve(source_correspondences->size());

	// Refining matching by filtering out wrong correspondence
	std::cout << "refineing matching" << std::endl;

	//for (int i = 0; i < correspondences->size(); i++)
		//cout << "correspondences[" << i << "]: " << correspondences->at(i) << endl;

	for (pcl::Correspondence source : *source_correspondences) {
		for (pcl::Correspondence target : *target_correspondences) {
			if (source.index_query == target.index_match && target.index_query == source.index_match) {
				pCorrespondences->push_back(source);
				//pCorrespondences->push_back(pcl::Correspondence(source.index_query, source.index_match, FLT_MAX));
			}
		}
	}

	correspondencesOut << pCorrespondences->size() << endl;
	for (pcl::Correspondence it : *pCorrespondences) {
		correspondencesOut << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
	}
	correspondencesOut << endl;
	correspondencesOut << endl;

	//pcl::CorrespondencesPtr pCorrespondences2(new pcl::Correspondences);

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> refine;
	refine.setInputSource(source_keypoints);
	refine.setInputTarget(target_keypoints);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);

	//refine.getRemainingCorrespondences(*pCorrespondences, *pCorrespondences2);

	correspondencesOut << pCorrespondences->size() << endl;
	for (pcl::Correspondence it : *pCorrespondences) {
		correspondencesOut << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
	}

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*source_keypoints, *source_keypointsXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*target_keypoints, *source_keypointsXYZ);

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	//pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> refine;
	refine.setInputSource(source_keypointsXYZ);
	refine.setInputTarget(target_keypointsXYZ);
	refine.setInputCorrespondences(pCorrespondences);
	refine.getCorrespondences(*pCorrespondences);*/
	



	/*for (pcl::Correspondence it : *pCorrespondences)
		accuracy_file << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;



	for (pcl::Correspondence it : *pCorrespondences){
		processing_time << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
		processing_time << "x: " << source_keypoints->at(it.index_query).x << "\ty: " << source_keypoints->at(it.index_query).y << "\tz: " << source_keypoints->at(it.index_query).z << endl;
		processing_time << "x: " << target_keypoints->at(it.index_match).x << "\ty: " << target_keypoints->at(it.index_match).y << "\tz: " << target_keypoints->at(it.index_match).z << endl;
	}*/

	return pCorrespondences;
}

// RとTの推定
Eigen::Matrix4f myEstimatingTransformation_3(pcl::PointCloud<DescriptorType>::Ptr source_features, pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal, pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal, pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud) {

	Eigen::Matrix4f transformation;

	// Estimating rigid transformation
	std::cout << "Estimating transformation" << std::endl;

	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;

	std::vector<int> source_index(source_features->size());
	for (int i = 0; i < source_features->size(); ++i) source_index[i] = i;

	est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
	std::cout << transformation << std::endl;

	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);

	return transformation;

	/*
	est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	std::cout << transformation << std::endl;

	pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	*/

}




#endif // _FeatureDescription_HPP_