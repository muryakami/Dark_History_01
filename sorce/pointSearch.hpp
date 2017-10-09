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

#endif // _PointSearch_HPP_