#ifndef _RemoveOutliers_HPP_
#define _RemoveOutliers_HPP_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool TF)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);		// �O��l����������_�Q�����
	sor.setMeanK(50);				// ��������ߖT�_�̐���ݒ�
	sor.setStddevMulThresh(1.0);	// �����̕W���΍��̒l��ݒ�
	sor.setNegative(TF);			// �O��l���o�͂���ꍇ��true�ɂ���
	sor.filter(*cloud_filtered);	// �o��
	return cloud_filtered;
}

#endif // _RemoveOutliers_HPP_