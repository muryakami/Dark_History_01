#ifndef _ReadPointCloud_HPP_
#define _ReadPointCloud_HPP_

//#include <iostream>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

// --------------------------------------------
// -----------Open point cloud data------------
// --------------------------------------------

pcl::PointCloud<pcl::PointXYZ>::Ptr myifstream_test(string filename) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	ifstream ifs(filename);
	string str;

	if (ifs.fail()) {
		cerr << "File do not exist.\n";
		exit(0);
	}

	while (getline(ifs, str)) {
		pcl::PointXYZ point;
		//sscanf_s(str.data(), "%f,%f,%f", &point.x, &point.y, &point.z);
		sscanf_s(str.data(), "%f\t%f\t%f", &point.x, &point.y, &point.z);
		point.x /= 500;		//500
		point.y /= 500;		//500
		point.z /= 1000;	//1000
		/////// H‹ïæ’[‚Ì‚Ý ///////
		//if (point.z < 44) continue;
		////////////////////////////
		cloud_ptr->points.push_back(point);
	}
	//cloud_ptr->width = (int)cloud_ptr->points.size();
	//cloud_ptr->height = 1;

	return cloud_ptr;
}

#endif