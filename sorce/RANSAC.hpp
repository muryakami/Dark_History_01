#ifndef _RANSAC_HPP_
#define _RANSAC_HPP_

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr Detect_wall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool TF)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);	// ���o���郂�f���̃^�C�v���w��
	seg.setMethodType(pcl::SAC_RANSAC);		// ���o�Ɏg�p������@���w��
	seg.setDistanceThreshold(0.10);			// 臒l 0.01, 0.05
	//seg.setDistanceThreshold(0.10);		// 臒l 0.01, 0.05

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(TF);//True�̏ꍇ�o�͂͌��o���ꂽ�ǈȊO�̃f�[�^ false�̏ꍇ�͕ǂ̃f�[�^
	extract.filter(*cloud_output);

	return cloud_output;//�o��
}

#include <pcl/filters/passthrough.h>
//pcl::PointCloud<pcl::PointXYZ>::Ptr Detect_Cylinder(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
pcl::PointCloud<pcl::PointNormal>::Ptr Detect_Cylinder(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	pcl::copyPointCloud(*cloud, *cloud_filtered);
	pcl::copyPointCloud(*cloud, *cloud_normals);
	for (int i = 0; i < 100; i++) {
		cout << "PointXYZ: " << cloud_filtered->at(i).x << " " << cloud_filtered->at(i).y << " " << cloud_filtered->at(i).z << endl;
		cout << "Normal: " << cloud_normals->at(i).normal_x << " " << cloud_normals->at(i).normal_y << " " << cloud_normals->at(i).normal_z << endl;
	}

	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals <pcl::PointXYZ, pcl::Normal> seg;

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER); // SACMODEL_CYLINDER SACMODEL_CIRCLE3D
	seg.setMethodType(pcl::SAC_RANSAC); // SAC_RANSAC SAC_RRANSAC SAC_RMSAC SAC_MLESAC
	seg.setNormalDistanceWeight(0.087); //0.01  0.087=sin5�x 0.174=sin10�x
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(1.00); //0.05	0.10
	seg.setRadiusLimits(0, 30.0); // 0,0.1		0, 20.0
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	//seg.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f)); // �C�ӎ��H(�����Őݒ�) �����I�ɒǉ�


	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//extract.setInputCloud(cloud_filtered);
	pcl::ExtractIndices<pcl::PointNormal> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>());
	extract.filter(*cloud_cylinder);

	return cloud_cylinder;//�o��
}

#endif // _RANSAC_HPP_