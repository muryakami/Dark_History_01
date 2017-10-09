#ifndef _PointSearch_HPP_
#define _PointSearch_HPP_

#include "BasicCalculation.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>
#include <vector>

using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>()); //近傍点配列

	float r = 7.0f; //半径r 5.0f
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator i = cloud->begin(); i != cloud->end(); i++) { //全体の点群を1点づつ抽出
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		float judge = euclidean_distance(tmp, attention_point); //normを計算
		if (judge < r*r) { //半径rよりも近かったら近傍点配列に追加
			neighborhood->push_back(tmp);
			continue;
		}
	}
	return neighborhood;
}

pcl::PointCloud<pcl::PointNormal>::Ptr radius_search(pcl::PointNormal searchPoint, float radius, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	kdtree.setInputCloud(cloud_in);

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
			cloud_ptr->push_back(cloud_in->points[pointIdxRadiusSearch[i]]);
		}
	}
	//cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr nearest_search(pcl::PointNormal searchPoint, int K, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal point;

	// K nearest neighbor search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	kdtree.setInputCloud(cloud_in);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
			cloud_ptr->push_back(cloud_in->points[pointIdxNKNSearch[i]]);
		}
	}
	//cout << "point_num: " << cloud_ptr->size() << endl;

	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

vector<pair<int, float>> radius_search_inf(pcl::PointNormal searchPoint, float radius, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	vector<pair<int, float>> points;
	points.reserve(300);

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	kdtree.setInputCloud(cloud_in);

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
			points.push_back(make_pair(pointIdxRadiusSearch[i], pointRadiusSquaredDistance[i]));
		}
	}
	//cout << "point_num: " << points.size() << endl;

	return points;
}

vector<pair<int, float>> nearest_search1(pcl::PointNormal searchPoint, int K, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in) {

	//点情報
	vector<pair<int, float>> points;
	points.reserve(K);

	// K nearest neighbor search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	kdtree.setInputCloud(cloud_in);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (int i = 0; i < pointIdxNKNSearch.size(); i++) {
			points.push_back(make_pair(pointIdxNKNSearch[i], pointNKNSquaredDistance[i]));
		}
	}
	//cout << "point_num: " << points.size() << endl;

	return points;
}

// 対応点探索 (異なる点群から最も近い点を取り出す)
std::vector<int> nearest_search_test1(pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	std::vector<int> correspondences;
	correspondences.resize(1);

	pcl::KdTreeFLANN<pcl::PointXYZ> search_tree;
	search_tree.setInputCloud(target_keypointsXYZ);

	std::vector<int> index(1);
	std::vector<float> L2_distance(1);

	for (int i = 0; i < source_keypointsXYZ->size(); i++) {
		search_tree.nearestKSearch(*source_keypointsXYZ, i, 1, index, L2_distance);
		correspondences[i] = index[0];
	}

	/*for (pcl::Correspondence it : *correspondences)
		outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;*/

	return correspondences;
}

// 対応点探索
pcl::CorrespondencesPtr nearest_search_test2(pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ, pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ) {

	int K = 1;

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	correspondences->reserve(source_keypointsXYZ->size()*K);

	pcl::KdTreeFLANN<pcl::PointXYZ> search_tree;
	search_tree.setInputCloud(target_keypointsXYZ);

	std::vector<int> index(K);
	std::vector<float> L2_distance(K);
	for (int i = 0; i < source_keypointsXYZ->size(); i++) {
		if (search_tree.nearestKSearch(*source_keypointsXYZ, i, K, index, L2_distance) > 0) {
			for (int j = 0; j < K; j++)
				correspondences->push_back(pcl::Correspondence(i, index[j], L2_distance[j]));
		}
	}

	/*for (pcl::Correspondence it : *correspondences)
	outputfile << "i: " << it.index_query << "\tj: " << it.index_match << "\tdistance: " << it.distance << endl;*/

	return correspondences;
}

#endif // _PointSearch_HPP_