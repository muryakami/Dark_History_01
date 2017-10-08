#ifndef _SurfaceNormals_HPP_
#define _SurfaceNormals_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//視点の決定
	ne.setInputCloud(cloud);//法線の計算を行いたい点群を指定する
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREEを作る
	ne.setSearchMethod(tree);//検索方法にKDTREEを指定する
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);//法線情報を入れる変数
	ne.setRadiusSearch(3.0);//検索する半径を指定する 3.0, 1.0
	ne.compute(*cloud_normals);//法線情報の出力先を指定する

	for (int i = 0; i < cloud->size(); i++) {
		cloud_normals->at(i).x = cloud->at(i).x;
		cloud_normals->at(i).y = cloud->at(i).y;
		cloud_normals->at(i).z = cloud->at(i).z;
	}

	for (int i = 0; i < cloud->size(); i++) {
		cloud_normals->at(i).normal_x *= (-1);
		cloud_normals->at(i).normal_y *= (-1);
		cloud_normals->at(i).normal_z *= (-1);
	}

	return cloud_normals;
}

#endif // _SurfaceNormals_HPP_