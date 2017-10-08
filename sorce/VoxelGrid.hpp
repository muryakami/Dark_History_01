#ifndef _VoxelGrid_HPP_
#define _VoxelGrid_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	double max_height = 0, max_width = 0, max_length = 0;
	double min_height = 0, min_width = 0, min_length = 0;
	double height = 0, width = 0, length = 0;

	for (size_t i = 0; i < cloud->points.size(); i++) {//“ü—Í“_ŒQ‚Ìc‰¡‚‚³‚ÌÅ‘å’l‚ÆÅ¬’l‚ðŒvŽZ‚·‚é
		if (max_height > cloud->points[i].z)
			max_height = max_height;
		else
			max_height = cloud->points[i].z;

		if (max_width > cloud->points[i].y)
			max_width = max_width;
		else
			max_width = cloud->points[i].y;

		if (max_length > cloud->points[i].x)
			max_length = max_length;
		else
			max_length = cloud->points[i].x;

		if (min_height < cloud->points[i].z)
			min_height = min_height;
		else
			min_height = cloud->points[i].z;

		if (min_width < cloud->points[i].y)
			min_width = min_width;
		else
			min_width = cloud->points[i].y;

		if (min_length < cloud->points[i].x)
			min_length = min_length;
		else
			min_length = cloud->points[i].x;
	}

	height = fabs(max_height) + fabs(min_height);
	width = fabs(max_width) + fabs(min_width);
	length = fabs(max_length) + fabs(min_length);

	double model_volume = 0, voxel_volume, voxel_length = 0;
	model_volume = height * width * length;
	voxel_volume = model_volume / 150000;

	voxel_length = pow(voxel_volume, 1.0 / 3.0);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(voxel_length, voxel_length, voxel_length);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid0(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	double max_height = -100000, max_width = -100000, max_length = -100000;
	double min_height = 100000, min_width = 100000, min_length = 100000;
	double height = 0, width = 0, length = 0;

	for (size_t i = 0; i < cloud->points.size(); i++) {//“ü—Í“_ŒQ‚Ìc‰¡‚‚³‚ÌÅ‘å’l‚ÆÅ¬’l‚ðŒvŽZ‚·‚é
		if (max_height > cloud->points[i].z)
			max_height = max_height;
		else
			max_height = cloud->points[i].z;

		if (max_width > cloud->points[i].y)
			max_width = max_width;
		else
			max_width = cloud->points[i].y;

		if (max_length > cloud->points[i].x)
			max_length = max_length;
		else
			max_length = cloud->points[i].x;

		if (min_height < cloud->points[i].z)
			min_height = min_height;
		else
			min_height = cloud->points[i].z;

		if (min_width < cloud->points[i].y)
			min_width = min_width;
		else
			min_width = cloud->points[i].y;

		if (min_length < cloud->points[i].x)
			min_length = min_length;
		else
			min_length = cloud->points[i].x;
	}

	height = max_height - min_height;
	width = max_width - min_width;
	length = max_length - min_length;


	double model_volume = 0, voxel_volume, voxel_length = 0;
	model_volume = height * width * length;
	voxel_volume = model_volume / 150000; // ŠÔˆø‚­ŠÔŠu‚ÌŒˆ’è (64*64*64)=262144 (32*32*32)=32768

	voxel_length = pow(voxel_volume, 1.0 / 3.0); // xyz‚Å“™‚µ‚¢ŠiŽq

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(voxel_length, voxel_length, voxel_length); // xyz‚ÌŠiŽq‚ÌƒTƒCƒY
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

#endif // _VoxelGrid_HPP_