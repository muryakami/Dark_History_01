#ifndef _MakeModelPPF_HPP_
#define _MakeModelPPF_HPP_

#include "Output.h"
#include "InterpolationSTL.hpp"
#include "SurfaceNormals.hpp"
#include "FeaturePointExtractionHarris.hpp"
#include "PPF.hpp"
#include "KeypointNormals.hpp"

void makeModelPPF() {

	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			// ���݂̃p�X
			filenameIO << filename << std::endl;
			
			// PPF���o�͂���p�X�ϐ�
			fs::path fileO;

			// �f�B���N�g����ύX
			for (auto& part : filename) {
				if (part == "Tool")
					fileO.append("PPF");
				else
					fileO.append(part.c_str());
			}
			filenameIO << fileO << std::endl;

			// �g���q��ύX
			fileO.replace_extension("txt");
			filenameIO << fileO << std::endl;
			filenameIO << std::endl;

			/*
			for (const auto &p : filename) {
				filenameIO << p.filename() << endl;
			}
			filenameIO << std::endl;

			for (auto& part : filename)
				filenameIO << part << endl;
			filenameIO << std::endl;

			for (auto& part : filename)
				filenameIO << part.c_str() << endl;
			filenameIO << std::endl;
			*/


			/*
			// �p�X�̕���
			filenameIO << filename.root_name().c_str() << std::endl; //=> 'C:'
			filenameIO << filename.root_directory() << std::endl;    //=> '\'
			filenameIO << filename.root_path() << std::endl;         //=> "C:\" (���[�g�f�B���N�g�� root_name() / root_directory())
			filenameIO << filename.relative_path() << std::endl;     //=> "hom\hom.txt"
			filenameIO << filename.parent_path() << std::endl;       //=> 'C:\hom'
			filenameIO << filename.filename() << std::endl;          //=> 'hom.txt'
			filenameIO << filename.stem() << std::endl;              //=> 'hom' (�g���q���������t�@�C����)
			filenameIO << filename.extension() << std::endl;         //=> '.txt' (�g���q)

			// �p�X�̕ύX
			fs::path filename2 = filename;
			// �g���q��ύX
			//filenameIO << path.replace_extension(".cpp") << std::endl; //=> 'C:\hom\hom.cpp'
			filenameIO << filename2.replace_extension("txt") << std::endl; //=> 'C:\hom\hom.cpp'
			// �t�@�C��������菜��
			filenameIO << filename2.remove_filename() << std::endl; //=> 'C:\hom'
			*/

			
			// STL�f�[�^�̓��}���
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename.string());

			// �_�Q�̊i�q���i�_�E���T���v�����O�j
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud_in);
			sor.setLeafSize(0.5f, 0.5f, 0.5f);
			sor.filter(*cloud_lattice);

			// �@���̐���
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

			// �����_���o
			pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);

			// �����_�̖@������
			*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);

			// PPF�̍쐬
			vector<myPPF> model_PPF = make_lightPPFs(attention_point);
			
			// PPF�̏o��
			fs::ofstream savefile(fileO);
			for (vector<myPPF>::iterator it = model_PPF.begin(); it != model_PPF.end(); it++) {
				savefile << it->distance << " " << it->angle_between << " " << it->angle_n1 << " " << it->angle_n2 << endl;
			}

		}
	}

}

void makeModelPPF(const string targetPath) {

	// �H��f�[�^�ǂݍ���
	const fs::path filename(targetPath);
		
	
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);

	// ���݂̃p�X
	filenameIO << filename << std::endl;

	// PPF���o�͂���p�X�ϐ�
	fs::path fileO = filename;

	// �g���q��ύX
	fileO.replace_extension("txt");
	filenameIO << fileO << std::endl;
	filenameIO << std::endl;

	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename.string());

	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	/*
	// �����_���o
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);

	// �����_�̖@������
	*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);
	*/


	/*
	// Harris�����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice);
	// Harris�����_�̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);
	*/
	/*
	// �I�N���[�W�����̍쐬
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // ���_
	//pcl::PointXYZ view_point(20.0f, 20.0f, -20.0f); // ���_
	pcl::PointCloud<pcl::PointNormal>::Ptr occlusion_cloud_normals = make_occlusion(cloud_normals, view_point, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr occlusion_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	occlusion_cloud->points.resize(occlusion_cloud_normals->points.size());
	pcl::copyPointCloud(*occlusion_cloud_normals, *occlusion_cloud);
	// Harris�����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(occlusion_cloud);
	// Harris�����_�̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe2(keypointsXYZINormal, occlusion_cloud_normals);
	*/

	/*
	// ���ȗ��̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(cloud_normals);
	*/
	/*
	// �I�N���[�W�����̍쐬
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // ���_
	//pcl::PointXYZ view_point(20.0f, 20.0f, -20.0f); // ���_
	pcl::PointCloud<pcl::PointNormal>::Ptr occlusion_cloud_normals = make_occlusion(cloud_normals, view_point, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr occlusion_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	occlusion_cloud->points.resize(occlusion_cloud_normals->points.size());
	pcl::copyPointCloud(*occlusion_cloud_normals, *occlusion_cloud);
	// ���ȗ��̒��œƎ����̍������̂𒊏o
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtractionRe(occlusion_cloud_normals);
	*/



	// PPF�̍쐬
	vector<myPPF> model_PPF = make_lightPPFs(attention_point);

	// PPF�̏o��
	fs::ofstream savefile(fileO);
	for (vector<myPPF>::iterator it = model_PPF.begin(); it != model_PPF.end(); it++) {
		savefile << it->distance << " " << it->angle_between << " " << it->angle_n1 << " " << it->angle_n2 << endl;
	}

	// �f�o�b�O�̂��߂̏o��
	//processing_time << "keypointsXYZINormal->size(): " << keypointsXYZINormal->size() << endl;
	processing_time << "attention_point->size(): " << attention_point->size() << endl;
	processing_time << endl;
	processing_time << endl;



	// ���͓_�Q�Ɩ@����\��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// ���͓_�Q�̕\��
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_lattice, 255, 255, 255);
	//viewer->addPointCloud(cloud_lattice, cloud_color, "cloud_lattice");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// ���͓_�Q�̕\���i�I�N���[�W�����j
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(occlusion_cloud, 255, 255, 255);
	viewer->addPointCloud(occlusion_cloud, cloud_color, "cloud_lattice");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// �����_�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> attention_point_color(attention_point, 0, 255, 0);
	viewer->addPointCloud(attention_point, attention_point_color, "attention_point");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "attention_point");
	viewer->addPointCloudNormals<pcl::PointNormal>(attention_point, 1, 7.0, "attention_pointn");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

#endif // _MakeModelPPF_HPP_