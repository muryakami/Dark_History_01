#ifndef _PCLViewer_HPP_
#define _PCLViewer_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// 【点群の可視化】
// 点群データを受け取り，可視化する

void viewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2) {

	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(stl_cloud, 0, 255, 0);
	//viewer->addPointCloud(stl_cloud, stl_color, "stl_cloud");
	//viewer->addPointCloudNormals<pcl::PointNormal>(stl_cloud, 1, 3.0, "stl_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "stl_cloud");

	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(surfaceN, 0, 255, 0);
	//viewer->addPointCloud(surfaceN, stl_color, "stl_cloud");
	//viewer->addPointCloudNormals<pcl::PointNormal>(surfaceN, 1, 3.0, "stl_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "stl_cloud");


	// point cloud
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(cloud, 255, 255, 255);
	//viewer->addPointCloud(cloud, pccolor, "point_cloud");
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(keypoints3D, 255, 0, 0);
	viewer->addPointCloud(keypoints3D, kpcolor, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	viewer->addPointCloudNormals<pcl::PointNormal>(keypoint_normals, 1, 7.0, "keypoint_normals");*/


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_holder(holder_face, 255, 255, 255);
	//viewer->addPointCloud(holder_face, pccolor_in, "holder_face");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_in(cloud_in, 255, 255, 255);
	//viewer->addPointCloud(cloud_in, pccolor_in, "cloud_in");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC(sample_vec, 0, 0, 255);
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pccolor_SAC(sample_vec, 0, 0, 255);
	viewer->addPointCloud(sample_vec, pccolor_SAC, "sample_vec");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC2(sample_vec2, 0, 255, 0);
	viewer->addPointCloud(sample_vec2, pccolor_SAC2, "sample_vec2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec2");*/

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tip_color(tip_point, 255, 0, 0);
	//viewer->addPointCloud(tip_point, tip_color, "tip_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "tip_point");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> attention_color(attention_point, 255, 0, 0);
	//viewer->addPointCloud(attention_point, attention_color, "attention_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PrincipalCurvatures> pccolor_C(principal_curvatures, 255, 255, 255);
	//viewer->addPointCloud(principal_curvatures, pccolor_C, "principal_curvatures");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(keypointsCurvature, 255, 0, 0);
	//viewer->addPointCloud(keypointsCurvature, kccolor, "keypointsCurvature");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(keypointsCurvature, 255, 0, 0);
	//viewer->addPointCloud(keypointsCurvature, kccolor, "keypointsCurvature");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypointsCurvature");


	/*int vp[2];
	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(attention_point2);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp[1]);
	viewer->addSphere(attention_point2->points[10], 3.0, 0.0, 0.0, 0.0, "sphere");*/

	/*int vp;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp);
	viewer->addSphere(attention_point2->points[10], 3.0, 0.0, 0.5, 0.5, "sphere", vp);*/




	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lattice_color(cloud_lattice2, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice2, lattice_color, "cloud_lattice2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice2");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloudN_color(cloud_normals3, 255, 255, 255);
	//viewer->addPointCloud(cloud_normals3, cloudN_color, "cloud_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_normals");



	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color(surfaceN, 255, 255, 255);
	//viewer->addPointCloud(surfaceN, stl_color, "stl_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stl_cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color(attention_point, 255, 0, 0);
	//viewer->addPointCloud(attention_point, attention_color, "attention_point");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");
	//viewer->addPointCloudNormals<pcl::PointNormal>(attention_point, 1, 7.0, "attention_pointn");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> stl_color2(surfaceN2, 255, 255, 255);
	//viewer->addPointCloud(surfaceN2, stl_color2, "stl_cloud2");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stl_cloud2");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color2(attention_point2, 0, 255, 0);
	viewer->addPointCloud(attention_point2, attention_color2, "attention_point2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point2");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_point2n");






	/*
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> lattice_color(cloud_lattice5, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice5, lattice_color, "cloud_lattice2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_lattice2");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_color2(attention_point5, 0, 255, 0);
	viewer->addPointCloud(attention_point5, attention_color2, "attention_point2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point2");
	//viewer->addPointCloudNormals<pcl::PointNormal>(attention_point2, 1, 7.0, "attention_point2n");
	*/











	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(cloud, 0, 255, 0);
	//viewer->addPointCloud(cloud, pccolor, "point_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "point_cloud");


	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(cloud, 255, 255, 255);
	viewer->addPointCloud(cloud, kccolor, "cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> pccolor_SAC(sample_vec, 0, 0, 255);
	viewer->addPointCloud(sample_vec, pccolor_SAC, "sample_vec");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor_SAC2(sample_vec2, 255, 0, 0);
	viewer->addPointCloud(sample_vec2, pccolor_SAC2, "sample_vec2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample_vec2");*/


	/*pcl::PointCloud<pcl::PointXYZ>::Ptr surface0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr surfaceN0(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*surfaceN, *surface0);
	pcl::copyPointCloud(*surfaceN, *surfaceN0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kccolor(surface0, 255, 255, 255);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(surface0, surfaceN0, 2, 0.5, "surface0");*/

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(surfaceN, 255, 255, 255);
	viewer->addPointCloudNormals<pcl::PointNormal>(surfaceN, 1, 0.1, "surfaceN");*/



	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kccolor(surfaceN, 255, 255, 255);
	//viewer->addPointCloud(surfaceN, kccolor, "surfaceN");

	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> uni_color(uniquenessPoints, 255, 0, 0);
	viewer->addPointCloud(uniquenessPoints, uni_color, "uniquenessPoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "uniquenessPoints");*/



	// StL
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> kp_color(keypoints, 255, 0, 0);
	//viewer->addPointCloud(keypoints, kp_color, "keypoints");
	//viewer->addPointCloudNormals<pcl::PointNormal>(keypoints, 1, 3.0, "kp_normals");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");




	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

// --------------
// -----Main-----
// --------------

/*
unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);

	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	return (viewer);
}

void viewPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2) {

	// 表示パラメータ
	//bool simple(false), rgb(false), custom_c(false), normals(false), shapes(false), viewports(false), interaction_customization(false);
	//int viewParameter = 0;
	//bool viewParameter[3];
	//bool viewParameter(false);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//if(viewParameter)




	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}
*/




#endif // _PCLViewer_HPP_