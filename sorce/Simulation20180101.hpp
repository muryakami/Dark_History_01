#ifndef _Simulation20180101_HPP_
#define _Simulation20180101_HPP_

#include <pcl/common/centroid.h>
#include "Simulation20171113.hpp"
#include "RANSAC.hpp"
#include "CalRotationMatrix.hpp"

void simulation20180101() {

	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			filenameIO << filename << std::endl;
			
			// �g���q��ύX
			fs::path fileO;
			//fileO.replace_extension("txt");
			//filenameIO << fileO << std::endl;

			for (auto& part : filename) {
				if (part == "Tool")
					fileO.append("PPF");
				else
					fileO.append(part.c_str());
			}
			filenameIO << fileO << std::endl;
			fileO.replace_extension("txt");
			filenameIO << fileO << std::endl;

			filenameIO << std::endl;
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


	/*
	// ���͓_�Q�Ɩ@����\��
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// ���͓_�Q�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_lattice, 255, 255, 255);
	viewer->addPointCloud(cloud_lattice, cloud_color, "cloud_lattice");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_lattice");

	// �~���_�̕\��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cylinder_color(cloud_cylinder, 0, 255, 0);
	viewer->addPointCloud(cloud_cylinder, cylinder_color, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud_cylinder");
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud_cylinder, 1, 3.0, "cloud_cylinder_normal");

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
}


#endif // _Simulation20180101_HPP_