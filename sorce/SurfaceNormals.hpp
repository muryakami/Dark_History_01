#ifndef _SurfaceNormals_HPP_
#define _SurfaceNormals_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::PointNormal>::Ptr surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//���_�̌���
	ne.setInputCloud(cloud);//�@���̌v�Z���s�������_�Q���w�肷��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREE�����
	ne.setSearchMethod(tree);//�������@��KDTREE���w�肷��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);//�@����������ϐ�
	ne.setRadiusSearch(3.0);//�������锼�a���w�肷�� 3.0, 1.0
	ne.compute(*cloud_normals);//�@�����̏o�͐���w�肷��

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