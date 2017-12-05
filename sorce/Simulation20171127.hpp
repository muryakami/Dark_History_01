#ifndef _Simulation20171127_HPP_
#define _Simulation20171127_HPP_

#include <pcl/common/centroid.h>
#include "Simulation20171113.hpp"
#include "RANSAC.hpp"

void simulation20171127() {
	
	// 工具データ読み込み
	const fs::path path("..\\DataBase");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			cylinderData << filename << std::endl;

			// STLデータの内挿補間
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename.string());

			// 点群の格子化（ダウンサンプリング）
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud_in);
			sor.setLeafSize(0.5f, 0.5f, 0.5f);
			sor.filter(*cloud_lattice);

			// 法線の生成
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

			// RANSAC（円筒）
			//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder = Detect_Cylinder(cloud_normals);
			cloud_cylinder = Detect_Cylinder(cloud_normals);
			cylinderData << "cloud: " << cloud_lattice->size() << endl;
			cylinderData << "cylinder: " << cloud_cylinder->size() << endl;
			cylinderData << endl;

			float s = 0;
			Eigen::Vector4f xyz_centroid;
			pcl::compute3DCentroid(*cloud_lattice, xyz_centroid);//重心を計算
			//cylinderData << xyz_centroid << std::endl;
			//cylinderData << endl;

			pcl::PointXYZ centroid(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
			for (pcl::PointXYZ point : *cloud_lattice) {
				s += euclidean_distance(point, centroid);
			}
			s /= cloud_lattice->size();
			//cylinderData << s << std::endl;
			//cylinderData << endl;
			s = sqrt(s);
			cylinderData << s << std::endl;
			cylinderData << endl;

			array<array<double, 3>, 3> point_PCA = cal_PCA(cloud_lattice);
			pcl::Normal point_normal0(point_PCA[0][0], point_PCA[0][1], point_PCA[0][2]);
			cylinderData << "x: " << point_normal0.normal_x << "\ty: " << point_normal0.normal_y << "\tz: " << point_normal0.normal_z << endl;
			pcl::Normal point_normal1(point_PCA[1][0], point_PCA[1][1], point_PCA[1][2]);
			cylinderData << "x: " << point_normal1.normal_x << "\ty: " << point_normal1.normal_y << "\tz: " << point_normal1.normal_z << endl;
			pcl::Normal point_normal2(point_PCA[2][0], point_PCA[2][1], point_PCA[2][2]);
			cylinderData << "x: " << point_normal2.normal_x << "\ty: " << point_normal2.normal_y << "\tz: " << point_normal2.normal_z << endl;
			cylinderData << endl;


			float s2 = 0;
			/*vector<array<float, 2>> testV;
			testV.reserve(cloud_lattice->size());
			for (pcl::PointXYZ point : *cloud_lattice) {
				array<float, 2> xy;
				xy[0] = point.x;
				xy[1] = point.y;
				testV.push_back(xy);
			}*/
			for (pcl::PointXYZ point : *cloud_lattice) {
				s2 += (point.x - centroid.x)*(point.x - centroid.x) + (point.y - centroid.y)*(point.y - centroid.y);
			}
			s2 /= cloud_lattice->size();
			s2 = sqrt(s2);
			cylinderData << s2 << std::endl;
			cylinderData << endl;

			/*array<array<double, 2>, 2> point_PCA2 = cal_PCA2(testV);
			cylinderData << "x: " << point_PCA2[0][0] << "\ty: " << point_PCA2[0][1] << endl;
			cylinderData << "x: " << point_PCA2[1][0] << "\ty: " << point_PCA2[1][1] << endl;
			cylinderData << endl;*/

		}
	}
	

	/*
	//const string filename("..\\DataBase\\Tool\\2020\\PBT35R2020.STL");
	const string filename("..\\DataBase\\Tool\\2525\\PSBNL2525M12.STL");
	cylinderData << filename << std::endl;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);

	// 点群の格子化（ダウンサンプリング）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// RANSAC（円筒）
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder = Detect_Cylinder(cloud_normals);
	cloud_cylinder = Detect_Cylinder(cloud_normals);
	cylinderData << "cloud: " << cloud_lattice->size() << endl;
	cylinderData << "cylinder: " << cloud_cylinder->size() << endl;
	cylinderData << endl;
	*/



	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_lattice, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice, cloud_color, "cloud_lattice");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// 円筒点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cylinder_color(cloud_cylinder, 0, 255, 0);
	viewer->addPointCloud(cloud_cylinder, cylinder_color, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud_cylinder");
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud_cylinder, 1, 3.0, "cloud_cylinder_normal");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


#endif // _Simulation20171127_HPP_