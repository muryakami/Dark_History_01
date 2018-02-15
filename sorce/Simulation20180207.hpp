#ifndef _Simulation20180207_HPP_
#define _Simulation20180207_HPP_

#include "Simulation20171113.hpp"
#include "MakeModelPPF.hpp"


pcl::PointCloud<DescriptorType>::Ptr procedure3(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point, std::mt19937 engine, float density) {

	// STL�f�[�^�̓��}���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl_density(filename, density);
	

	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	/*pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);*/

	// �_�E���T���v�����O���Ȃ�
	cloud_lattice = cloud_in;

	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);


	// �����_���o
	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	// �����_�̖@������
	//*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);



	// Harris, ��Ď�@�@�I�N���[�W���������
	// �I�N���[�W�����̍쐬
	/*pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // ���_
	//pcl::PointXYZ view_point(20.0f, 20.0f, -20.0f); // ���_
	pcl::PointCloud<pcl::PointNormal>::Ptr occlusion_cloud_normals = make_occlusion(cloud_normals, view_point, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr occlusion_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	occlusion_cloud->points.resize(occlusion_cloud_normals->points.size());
	pcl::copyPointCloud(*occlusion_cloud_normals, *occlusion_cloud);
	// Harris�����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(occlusion_cloud);
	// Harris�����_�̒��œƎ����̍������̂𒊏o
	*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, occlusion_cloud_normals);*/


	// Harris, ��Ď�@
	// Harris�����_���o
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice);
	// Harris�����_�̒��œƎ����̍������̂𒊏o
	*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);


	// �����_���@�i�V���b�t���j
	/*std::shuffle(cloud_normals->begin(), cloud_normals->end(), engine);
	int numThreshold = 100; //15 20
	int count = 0;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_normals->begin(); it != cloud_normals->end(); it++) {
		if (count >= numThreshold) break; //
		attention_point->push_back(*it);
		count++;
	}*/


	// �����ʂ̋L�q
	return myFeatureDescription_4(cloud_normals, attention_point);

}


void simulation20180207() {

	std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
	double elapsed;

	// ���������̂��߂̃V�[�h
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// �H��f�[�^�ǂݍ���
	const string filenameT = ".\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";
	// �^�[�Q�b�g
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filenameT, target_cloud, target_keypointsNormal, engine, densityT);


	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filenameS)) {

			// ���݂̃p�X
			filenameIO << filenameS << std::endl;
			
			// �\�[�X
			float densityS = 2.0f;
			pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

			// �Ή��_�T��
			pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
			pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

			// ��Ή�����
			pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);


			// R��T�̐���
			//Eigen::Matrix4f transformation;
			//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;
			//est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
			//Eigen::Matrix4f transformation2;
			//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est2;
			//est2.estimateRigidTransformation(*target_keypointsNormal, *source_keypointsNormal, *correspondences2, transformation2);
			Eigen::Matrix4f transformation3;
			pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
			est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);



			//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputCloud(source_cloud);
			icp.setInputTarget(target_cloud);

			icp.setTransformationEpsilon(1e-6);
			icp.setMaxCorrespondenceDistance(5.0f);
			icp.setMaximumIterations(200);
			icp.setEuclideanFitnessEpsilon(1.0f);
			icp.setRANSACOutlierRejectionThreshold(1.0f);

			pcl::PointCloud<pcl::PointXYZ> Final;
			icp.align(Final);

			//�ϊ�matrix��\������
			Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
			transformation_matrix = icp.getFinalTransformation().cast<double>();
			print4x4Matrix(transformation_matrix);


			//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
			//pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp_keypoint;
			//icp_keypoint.setInputCloud(source_keypointsNormal);
			//icp_keypoint.setInputTarget(target_keypointsNormal);

			//icp_keypoint.setTransformationEpsilon(1e-6);
			//icp_keypoint.setMaxCorrespondenceDistance(5.0f);
			//icp_keypoint.setMaximumIterations(200);
			//icp_keypoint.setEuclideanFitnessEpsilon(1.0f);
			//icp_keypoint.setRANSACOutlierRejectionThreshold(1.0f);

			//pcl::PointCloud<pcl::PointNormal> Final_keypoint;
			//icp_keypoint.align(Final_keypoint);

			//�ϊ�matrix��\������
			//Eigen::Matrix4d transformation_matrix_keypoint = Eigen::Matrix4d::Identity();
			//transformation_matrix_keypoint = icp_keypoint.getFinalTransformation().cast<double>();
			//print4x4Matrix(transformation_matrix_keypoint);


			// �_�Q�̍��̕ϊ��iICP�A���S���Y���ɂ�鐄��j
			pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

			// ��Ή������i�����Ή��_�̐��딻��j
			pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

			// ��Ή������i��]��̑Ή��_�j
			pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


			filenameIO << "correspondences->size(): " << correspondences->size() << endl;
			filenameIO << "correspondences2->size(): " << correspondences2->size() << endl;
			filenameIO << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
			filenameIO << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
			filenameIO << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
			filenameIO << endl;
		}
	}

	// �������Ԃ̏o��
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;
}



void simulation20180209_1() {

	std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
	double elapsed;

	// ���������̂��߂̃V�[�h
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// �H��f�[�^�ǂݍ���
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\Tool\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";

	// �^�[�Q�b�g
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filename, target_cloud, target_keypointsNormal, engine, densityT);

	// �o��
	const string fileO = "C:\\Users\\yuki\\Documents\\DataBase\\Keypoints\\STL files\\NikonTurningTool.txt";
	fs::ofstream savefile(fileO);

	for (float densityS = densityT; 0.0f < densityS; densityS -= 0.2f) {

		// �\�[�X
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filename, source_cloud, source_keypointsNormal, engine, densityS);

		// �Ή��_�T��
		pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
		pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

		// ��Ή�����
		pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

		// R��T�̐���
		//Eigen::Matrix4f transformation3;
		//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
		//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

		//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(source_cloud);
		icp.setInputTarget(target_cloud);
		icp.setTransformationEpsilon(1e-6);
		icp.setMaxCorrespondenceDistance(5.0f);
		icp.setMaximumIterations(200);
		icp.setEuclideanFitnessEpsilon(1.0f);
		icp.setRANSACOutlierRejectionThreshold(1.0f);
		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
		transformation_matrix = icp.getFinalTransformation().cast<double>();

		// �_�Q�̍��̕ϊ��iICP�A���S���Y���ɂ�鐄��j
		pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

		// ��Ή������i�����Ή��_�̐��딻��j
		pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

		// ��Ή������i��]��̑Ή��_�j
		pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


		//savefile << "correspondences->size(): " << correspondences->size() << endl;
		//savefile << "correspondences2->size(): " << correspondences2->size() << endl;
		//savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
		//savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
		//savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;

		// �}�b�`���O�ɗL���ȓ����_�i���Ή��j���̏o��
		savefile << (float)pCorrespondences3->size() / correspondences2->size() * 100 << "\t";
	}
	//savefile << endl;
	savefile.close();

}


void simulation20180209_2() {

	std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
	double elapsed;

	// ���������̂��߂̃V�[�h
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// �H��f�[�^�ǂݍ���
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\Tool\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";

	// �^�[�Q�b�g
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filename, target_cloud, target_keypointsNormal, engine, densityT);

	// �o��
	const string fileO = "C:\\Users\\yuki\\Documents\\DataBase\\Keypoints\\STL files\\NikonTurningTool.txt";
	fs::ofstream savefile(fileO);

	for (float densityS = densityT; 0.0f < densityS; densityS -= 0.2f) {

		// �\�[�X
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filename, source_cloud, source_keypointsNormal, engine, densityS);

		// �Ή��_�T��
		pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
		pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

		// ��Ή�����
		pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

		// R��T�̐���
		//Eigen::Matrix4f transformation3;
		//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
		//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

		//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(source_cloud);
		icp.setInputTarget(target_cloud);
		icp.setTransformationEpsilon(1e-6);
		icp.setMaxCorrespondenceDistance(5.0f);
		icp.setMaximumIterations(200);
		icp.setEuclideanFitnessEpsilon(1.0f);
		icp.setRANSACOutlierRejectionThreshold(1.0f);
		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
		transformation_matrix = icp.getFinalTransformation().cast<double>();

		// �_�Q�̍��̕ϊ��iICP�A���S���Y���ɂ�鐄��j
		pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

		// ��Ή������i�����Ή��_�̐��딻��j
		pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

		// ��Ή������i��]��̑Ή��_�j
		pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


		//savefile << "correspondences->size(): " << correspondences->size() << endl;
		//savefile << "correspondences2->size(): " << correspondences2->size() << endl;
		//savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
		//savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
		//savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;

		// �}�b�`���O�ɗL���ȓ����_�i���Ή��j���̏o��
		savefile << (float)pCorrespondences3->size() / correspondences2->size() * 100 << "\t";
	}
	//savefile << endl;
	savefile.close();



	/*
	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
	if (!fs::is_directory(filenameS)) {

	// ���݂̃p�X
	filenameIO << filenameS << std::endl;

	// �\�[�X
	float densityS = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

	// �Ή��_�T��
	pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
	pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

	// ��Ή�����
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);


	// R��T�̐���
	//Eigen::Matrix4f transformation;
	//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;
	//est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
	//Eigen::Matrix4f transformation2;
	//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est2;
	//est2.estimateRigidTransformation(*target_keypointsNormal, *source_keypointsNormal, *correspondences2, transformation2);
	Eigen::Matrix4f transformation3;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
	est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);



	//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(source_cloud);
	icp.setInputTarget(target_cloud);

	icp.setTransformationEpsilon(1e-6);
	icp.setMaxCorrespondenceDistance(5.0f);
	icp.setMaximumIterations(200);
	icp.setEuclideanFitnessEpsilon(1.0f);
	icp.setRANSACOutlierRejectionThreshold(1.0f);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	//�ϊ�matrix��\������
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = icp.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix);


	//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
	//pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp_keypoint;
	//icp_keypoint.setInputCloud(source_keypointsNormal);
	//icp_keypoint.setInputTarget(target_keypointsNormal);

	//icp_keypoint.setTransformationEpsilon(1e-6);
	//icp_keypoint.setMaxCorrespondenceDistance(5.0f);
	//icp_keypoint.setMaximumIterations(200);
	//icp_keypoint.setEuclideanFitnessEpsilon(1.0f);
	//icp_keypoint.setRANSACOutlierRejectionThreshold(1.0f);

	//pcl::PointCloud<pcl::PointNormal> Final_keypoint;
	//icp_keypoint.align(Final_keypoint);

	//�ϊ�matrix��\������
	//Eigen::Matrix4d transformation_matrix_keypoint = Eigen::Matrix4d::Identity();
	//transformation_matrix_keypoint = icp_keypoint.getFinalTransformation().cast<double>();
	//print4x4Matrix(transformation_matrix_keypoint);


	// �_�Q�̍��̕ϊ��iICP�A���S���Y���ɂ�鐄��j
	pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

	// ��Ή������i�����Ή��_�̐��딻��j
	pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

	// ��Ή������i��]��̑Ή��_�j
	pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


	filenameIO << "correspondences->size(): " << correspondences->size() << endl;
	filenameIO << "correspondences2->size(): " << correspondences2->size() << endl;
	filenameIO << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
	filenameIO << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
	filenameIO << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
	filenameIO << endl;
	}
	}

	// �������Ԃ̏o��
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;
	*/
}


void simulation20180209_3() {

	// ���������̂��߂̃V�[�h
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// ���ԏo��
	const string fileTime = "C:\\Users\\yuki\\Documents\\DataBase\\Matching\\time.txt";
	fs::ofstream savefileTime(fileTime);

	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameT, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filenameT)) {

			// �o�͂���p�X�ϐ�
			fs::path fileO;

			// �f�B���N�g����ύX
			for (auto& part : filenameT) {
				if (part == "Tool")
					fileO.append("Matching");
				else
					fileO.append(part.c_str());
			}

			// �g���q��ύX
			fileO.replace_extension("txt");

			// �o��
			fs::ofstream savefile(fileO);

			// �^�[�Q�b�g
			float densityT = 2.0f;
			pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filenameT.string(), target_cloud, target_keypointsNormal, engine, densityT);

			BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
				if (!fs::is_directory(filenameS)) {

					// ���Ԍv��
					std::chrono::system_clock::time_point start, end; // �^�� auto �ŉ�
					double elapsed;

					// ���݂̃p�X
					savefile << filenameS << std::endl;

					// �\�[�X
					float densityS = 2.0f;
					pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
					pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

					// �Ή��_�T��
					pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
					pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

					// ��Ή�����
					pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

					// R��T�̐���
					//Eigen::Matrix4f transformation3;
					//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
					//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

					//cloud_in��cloud_out��ICP�A���S�Y���ɂ��ϊ�matrix�����߂�B
					pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
					icp.setInputCloud(source_cloud);
					icp.setInputTarget(target_cloud);
					icp.setTransformationEpsilon(1e-6);
					icp.setMaxCorrespondenceDistance(5.0f);
					icp.setMaximumIterations(200);
					icp.setEuclideanFitnessEpsilon(1.0f);
					icp.setRANSACOutlierRejectionThreshold(1.0f);
					Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
					transformation_matrix = icp.getFinalTransformation().cast<double>();

					// �_�Q�̍��̕ϊ��iICP�A���S���Y���ɂ�鐄��j
					pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

					// ��Ή������i�����Ή��_�̐��딻��j
					pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

					// ��Ή������i��]��̑Ή��_�j
					pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


					savefile << "correspondences->size(): " << correspondences->size() << endl;
					savefile << "correspondences2->size(): " << correspondences2->size() << endl;
					savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
					savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
					savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
					savefile << endl;

					// �������Ԃ̏o��
					elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
					savefileTime << elapsed << " ";
				}
			}
			savefile.close();
			savefileTime << endl;
		}
	}
	savefileTime.close();
}



void outputKeypoints() {

	// �H��f�[�^�ǂݍ���
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			// ���݂̃p�X
			filenameIO << filename << std::endl;

			// �����_���o�͂���p�X�ϐ�
			fs::path fileO;

			// �f�B���N�g����ύX
			for (auto& part : filename) {
				if (part == "Tool")
					fileO.append("Keypoints");
				else
					fileO.append(part.c_str());
			}

			// �g���q��ύX
			fileO.replace_extension("txt");

			// �_�Q�f�[�^�̐���
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename.string());


			// �����_�̏o��
			fs::ofstream savefile(fileO);
			for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
				savefile << it->x << " " << it->y << " " << it->z << " " << endl;
			}
			savefile.close();

		}
	}

}



#endif // _Simulation20180207_HPP_