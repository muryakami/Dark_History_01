#ifndef _Simulation20171127_HPP_
#define _Simulation20171127_HPP_

#include <pcl/common/centroid.h>
#include "Simulation20171113.hpp"
#include "RANSAC.hpp"
#include "CalRotationMatrix.hpp"

void simulation20171127() {
	
	// 工具データ読み込み
	const fs::path path("..\\DataBase");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
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

			/*
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
			//vector<array<float, 2>> testV;
			//testV.reserve(cloud_lattice->size());
			//for (pcl::PointXYZ point : *cloud_lattice) {
			//array<float, 2> xy;
			//xy[0] = point.x;
			//xy[1] = point.y;
			//testV.push_back(xy);
			//}
			for (pcl::PointXYZ point : *cloud_lattice) {
				s2 += (point.x - centroid.x)*(point.x - centroid.x) + (point.y - centroid.y)*(point.y - centroid.y);
			}
			s2 /= cloud_lattice->size();
			s2 = sqrt(s2);
			cylinderData << s2 << std::endl;
			cylinderData << endl;

			//array<array<double, 2>, 2> point_PCA2 = cal_PCA2(testV);
			//cylinderData << "x: " << point_PCA2[0][0] << "\ty: " << point_PCA2[0][1] << endl;
			//cylinderData << "x: " << point_PCA2[1][0] << "\ty: " << point_PCA2[1][1] << endl;
			//cylinderData << endl;


			cylinderDataPCA << s2 << std::endl;
			*/

			cylinderData << "cloud: " << cloud_lattice->size() << endl;
			cylinderData << "cylinder: " << cloud_cylinder->size() << endl;
			cylinderData << "rate: " << (float)cloud_cylinder->size() / cloud_lattice->size() * 100 << endl;


			//重心の計算
			Eigen::Vector4f xyz_centroid;
			pcl::compute3DCentroid(*cloud_lattice, xyz_centroid);
			pcl::PointXYZ centroid(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

			// 円筒へのフィッティング（RANSAC）
			pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
			cloud_cylinder = Detect_Cylinder2(cloud_normals, coefficients_cylinder);

			// 回転マトリクスの計算 (ロドリゲスの公式)
			pcl::PointXYZ searchN(coefficients_cylinder->values.at(3), coefficients_cylinder->values.at(4), coefficients_cylinder->values.at(5));
			pcl::PointXYZ projectN(0.0f, 0.0f, 1.0f);
			pcl::PointXYZ axisR = cross_product3_cal(searchN, projectN);
			float cosR = dot_product3_cal(searchN, projectN);
			float sinR = sqrt(1.0 - cosR*cosR);

			Eigen::Matrix4d rotate_matrix; // 回転マトリクス
			rotate_matrix(0, 0) = (1 - cosR)*axisR.x*axisR.x + cosR;
			rotate_matrix(0, 1) = (1 - cosR)*axisR.x*axisR.y - sinR*axisR.z;
			rotate_matrix(0, 2) = (1 - cosR)*axisR.z*axisR.x + sinR*axisR.y;
			rotate_matrix(0, 3) = 0.0f;	// 平行移動 x

			rotate_matrix(1, 0) = (1 - cosR)*axisR.x*axisR.y + sinR*axisR.z;
			rotate_matrix(1, 1) = (1 - cosR)*axisR.y*axisR.y + cosR;
			rotate_matrix(1, 2) = (1 - cosR)*axisR.y*axisR.z - sinR*axisR.x;
			rotate_matrix(1, 3) = 0.0f;	// 平行移動 y

			rotate_matrix(2, 0) = (1 - cosR)*axisR.z*axisR.x - sinR*axisR.y;
			rotate_matrix(2, 1) = (1 - cosR)*axisR.y*axisR.z + sinR*axisR.x;
			rotate_matrix(2, 2) = (1 - cosR)*axisR.z*axisR.z + cosR;
			rotate_matrix(2, 3) = 0.0f;	// 平行移動 z

			// アフィン変換のためのダミー次元
			rotate_matrix(3, 0) = 0.0f;
			rotate_matrix(3, 1) = 0.0f;
			rotate_matrix(3, 2) = 0.0f;
			rotate_matrix(3, 3) = 1.0f;

			// 円筒軸の回転
			pcl::transformPointCloud(*cloud_lattice, *cloud_rotated, rotate_matrix);

			// 重心の回転
			pcl::PointXYZ centroid_axis;
			centroid_axis.x = rotate_matrix(0, 0) * centroid.x + rotate_matrix(0, 1) * centroid.y + rotate_matrix(0, 2) * centroid.z;
			centroid_axis.y = rotate_matrix(1, 0) * centroid.x + rotate_matrix(1, 1) * centroid.y + rotate_matrix(1, 2) * centroid.z;
			centroid_axis.z = rotate_matrix(2, 0) * centroid.x + rotate_matrix(2, 1) * centroid.y + rotate_matrix(2, 2) * centroid.z;

			// 標準偏差σの計算
			float s_axis = 0;
			for (pcl::PointXYZ point : *cloud_rotated) {
				s_axis += (point.x - centroid_axis.x)*(point.x - centroid_axis.x) + (point.y - centroid_axis.y)*(point.y - centroid_axis.y);
			}
			s_axis /= cloud_rotated->size();
			s_axis = sqrt(s_axis);


			// 工具の軸計算（PCA）
			array<array<double, 3>, 3> point_PCA = cal_PCA(cloud_lattice);
			pcl::PointXYZ axis2_PCA(point_PCA[2][0], point_PCA[2][1], point_PCA[2][2]);
			pcl::PointXYZ axis_z(0.0f, 0.0f, 1.0f);
			Eigen::Matrix4d rotate_matrix2 = calRotationMatrix(axis2_PCA, axis_z);

			// 最大主軸の回転
			pcl::transformPointCloud(*cloud_lattice, *cloud_rotated, rotate_matrix2);

			// 重心の回転
			pcl::PointXYZ centroid_axis2;
			centroid_axis2.x = rotate_matrix2(0, 0) * centroid.x + rotate_matrix2(0, 1) * centroid.y + rotate_matrix2(0, 2) * centroid.z;
			centroid_axis2.y = rotate_matrix2(1, 0) * centroid.x + rotate_matrix2(1, 1) * centroid.y + rotate_matrix2(1, 2) * centroid.z;
			centroid_axis2.z = rotate_matrix2(2, 0) * centroid.x + rotate_matrix2(2, 1) * centroid.y + rotate_matrix2(2, 2) * centroid.z;

			// 標準偏差σの計算（距離の偏差の二乗平均平方根）
			float s_axis2 = 0;
			for (pcl::PointXYZ point : *cloud_rotated) {
				s_axis2 += (point.x - centroid_axis.x)*(point.x - centroid_axis.x) + (point.y - centroid_axis.y)*(point.y - centroid_axis.y);
			}
			s_axis2 /= cloud_rotated->size();
			s_axis2 = sqrt(s_axis2);


			cylinderDataRadius << filename << std::endl;
			cylinderDataRadius << coefficients_cylinder->values.at(6) << endl;
			cylinderDataAxis << filename << std::endl;
			cylinderDataAxis << s_axis << std::endl;
			cylinderDataPCA << filename << std::endl;
			cylinderDataPCA << s_axis2 << std::endl;
			
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