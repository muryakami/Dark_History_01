#ifndef _RemoveOutliers_HPP_
#define _RemoveOutliers_HPP_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool TF)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);		// 外れ値を除去する点群を入力
	sor.setMeanK(50);				// 検索する近傍点の数を設定
	sor.setStddevMulThresh(1.0);	// 距離の標準偏差の値を設定
	sor.setNegative(TF);			// 外れ値を出力する場合はtrueにする
	sor.filter(*cloud_filtered);	// 出力
	return cloud_filtered;
}

#endif // _RemoveOutliers_HPP_