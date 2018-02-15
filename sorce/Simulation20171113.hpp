#ifndef _Simulation20171113_HPP_
#define _Simulation20171113_HPP_

#include "Simulation20171030.hpp"
#include "FeatureDescription.hpp"

int viewParam = 0;
void keyboardEventOccurred2(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	if (event.keyDown()) {
		/*try {
			viewParam = boost::lexical_cast<int>(event.getKeySym());
		}
		catch (const boost::bad_lexical_cast& e) {
			cout << e.what() << endl;
			viewParam = 1;
		}*/

		/*try {
			viewParam = std::stoi(event.getKeySym());
			cout << "keyDown: " << viewParam << endl;
		}
		catch (std::invalid_argument e) {
			cout << e.what() << endl;
			viewParam = 1;
			cout << "keyDown: " << viewParam << endl;
		}
		catch (std::out_of_range e) {
			cout << e.what() << endl;
			viewParam = 2;
			cout << "keyDown: " << viewParam << endl;
		}*/

		if (event.getKeySym() == "v") {
			viewParam = 1;
		}
		else if (event.getKeySym() == "b") {
			viewParam = 2;
		}
		else if (event.getKeySym() == "n") {
			viewParam = 3;
		}
		else if (event.getKeySym() == "m") {
			viewParam = 4;
		}
		else if (event.getKeySym() == "k") {
			viewParam = 5;
		}
		else if (event.getKeySym() == "l") {
			cout << "unregistered" << endl;
		}
		cout << "keyDown: " << viewParam << endl;
	}
}

void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

// 【マッチング手法の性能評価実験】
// 抽出した特徴点にノイズを加え，一致率を計算する
pcl::PointCloud<DescriptorType>::Ptr procedure2(string filename, boolean TFC, boolean TFA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {
//pcl::PointCloud<pcl::SHOT352>::Ptr procedure2(string filename, boolean TFC, boolean TFA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {
//pcl::PointCloud<pcl::FPFHSignature33>::Ptr procedure2(string filename, boolean TFC, boolean TFA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point) {

	float maxNoiseC = 1.0f;
	float maxNoiseA = 1.0f;
	//OFFSET offset = (0.7f, 0.5f, 0.0f);
	//OFFSET offset;
	//offset.x = 70.0f;
	//offset.y = 50.0f;
	//offset.z = 0.0f;

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// STLデータの内挿補間
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);

	// 点群へのノイズ付与(trueで付与)
	if (TFC) addingNoise2(cloud_in, maxNoiseC);

	//if (TFO) addingOffset(cloud_in, offset);

	// 点群の格子化（ダウンサンプリング）
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	
	/*
	// Harrisとか提案手法とかオクルージョンありで
	// オクルージョンの作成
	pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // 視点
	//pcl::PointXYZ view_point(20.0f, 20.0f, -20.0f); // 視点
	pcl::PointCloud<pcl::PointNormal>::Ptr occlusion_cloud_normals = make_occlusion(cloud_normals, view_point, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr occlusion_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	occlusion_cloud->points.resize(occlusion_cloud_normals->points.size());
	pcl::copyPointCloud(*occlusion_cloud_normals, *occlusion_cloud);
	*/
	// Harris特徴点検出
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(occlusion_cloud);
	// Harris特徴点の中で独自性の高いものを抽出
	//*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, occlusion_cloud_normals);
	
	
	// Harrisとか提案手法とか
	// Harris特徴点検出
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice);
	// Harris特徴点の中で独自性の高いものを抽出
	*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);

	
	/*
	// シャッフル
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::shuffle(cloud_normals->begin(), cloud_normals->end(), engine);
	int numThreshold = 20; //15 20
	int count = 0;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_normals->begin(); it != cloud_normals->end(); it++) {
		if (count >= numThreshold) break; //
		attention_point->push_back(*it);
		count++;
	}
	*/


	/*
	元々のやつ Harris
	// 特徴点検出
	start = std::chrono::system_clock::now(); // 計測開始時間
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	end = std::chrono::system_clock::now();  // 計測終了時間

	// 特徴点へのノイズ付与
	if (TFA) addingNoise2(keypointsXYZ, maxNoiseA);

	// 特徴点の法線生成
	*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);
	*/



	// 処理時間の出力
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;

	// デバッグのための出力
	/*
	//processing_time << "cloud_lattice->size(): " << cloud_lattice->size() << endl;
	//processing_time << "cloud_normals->size(): " << cloud_normals->size() << endl;
	processing_time << "keypointsXYZ->size(): " << keypointsXYZ->size() << endl;
	processing_time << "attention_point->size(): " << attention_point->size() << endl;
	processing_time << endl;
	processing_time << endl;

	keypointsXYZ_IO << "keypointsXYZ->size(): " << keypointsXYZ->size() << endl;
	keypointsXYZ_IO << endl;
	keypointsXYZ_IO << endl;
	for (pcl::PointXYZ it : *keypointsXYZ) {
		keypointsXYZ_IO << "x: " << it.x << "\ty: " << it.y << "\tz: " << it.z << endl;
	}
	keypointsXYZ_IO << endl;
	keypointsXYZ_IO << endl;
	
	keypointsNormal_IO << "keypointsNormal->size(): " << attention_point->size() << endl;
	keypointsNormal_IO << endl;
	keypointsNormal_IO << endl;
	for (pcl::PointNormal it : *attention_point) {
		keypointsNormal_IO << "x: " << it.x << "\ty: " << it.y << "\tz: " << it.z << "\tnormal_x: " << it.normal_x << "\tnormal_y: " << it.normal_y << "\tnormal_z: " << it.normal_z << endl;
	}
	keypointsNormal_IO << endl;
	keypointsNormal_IO << endl;
	*/

	// 特徴量の記述
	//return myFeatureDescription_3(cloud_normals, keypointsXYZ);
	return myFeatureDescription_4(cloud_normals, attention_point);
	//return myFeatureDescription_5(cloud_normals, attention_point);

	//return myFeatureDescription_SHOT352(cloud_normals, keypointsXYZ); // PointNormalにするのもあり
	//return myFeatureDescription_FPFH2(cloud_normals, keypointsXYZ);
}

void simulation20171113() {
	// 工具データ読み込み
	const string filename = ".\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";

	// PPFの生成(STLデータ:A)	ソース
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	//vector<myPPF> source_PPF = procedure(filename, false, false, cloud_lattice, attention_point); //点群誤差，特徴点誤差
	//pcl::PointCloud<pcl::SHOT352>::Ptr source_features = procedure2(filename, false, false, source_cloud, source_keypointsNormal);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = procedure2(filename, false, false, false, source_cloud, source_keypointsNormal);
	pcl::PointCloud<DescriptorType>::Ptr source_features = procedure2(filename, false, false, source_cloud, source_keypointsNormal);

	// PPFの生成(STLデータ:A)	ターゲット
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	//vector<myPPF> target_PPF = procedure(filename, true, false, cloud_lattice2, attention_point2); //点群誤差，特徴点誤差
	//pcl::PointCloud<pcl::SHOT352>::Ptr target_features = procedure2(filename, true, false, target_cloud, target_keypointsNormal);
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = procedure2(filename, true, true, false, target_cloud, target_keypointsNormal);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure2(filename, true, false, target_cloud, target_keypointsNormal);

	// 誤差の出力
	//nearest_search_test3(attention_point, attention_point2);

	// 一致率の出力
	//outputAccuracy(&source_PPF, &target_PPF);	// AとA

	// 対応点探索
	//pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	//pcl::CorrespondencesPtr correspondences2 = myFindMatching_SHOT352_2(target_features, source_features);
	pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
	pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

	processing_time << "correspondences->size(): " << correspondences->size() << endl;
	processing_time << "correspondences2->size(): " << correspondences2->size() << endl;
	processing_time << "source_features->size(): " << source_features->size() << endl;
	processing_time << "target_features->size(): " << target_features->size() << endl;
	processing_time << "source_keypointsNormal->size(): " << source_keypointsNormal->size() << endl;
	processing_time << "target_keypointsNormal->size(): " << target_keypointsNormal->size() << endl;
	processing_time << endl;
	processing_time << endl;

	/*source_keypointsNormal_IO << "source_features->size(): " << source_features->size() << endl;
	source_keypointsNormal_IO << endl;
	source_keypointsNormal_IO << endl;
	for (pcl::PointNormal it : *source_keypointsNormal) {
		source_keypointsNormal_IO << "x: " << it.x << "\ty: " << it.y << "\tz: " << it.z << "\tnormal_x: " << it.normal_x << "\tnormal_y: " << it.normal_y << "\tnormal_z: " << it.normal_z << endl;
	}
	target_keypointsNormal_IO << "target_features->size(): " << target_features->size() << endl;
	target_keypointsNormal_IO << endl;
	target_keypointsNormal_IO << endl;
	for (pcl::PointNormal it : *target_keypointsNormal) {
		target_keypointsNormal_IO << "x: " << it.x << "\ty: " << it.y << "\tz: " << it.z << "\tnormal_x: " << it.normal_x << "\tnormal_y: " << it.normal_y << "\tnormal_z: " << it.normal_z << endl;
	}*/

	for (pcl::Correspondence it : *correspondences) {
		processing_time << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
		//processing_time << "x: " << source_keypointsNormal->at(it.index_query).x << "\ty: " << source_keypointsNormal->at(it.index_query).y << "\tz: " << source_keypointsNormal->at(it.index_query).z << endl;
		//processing_time << "x: " << target_keypointsNormal->at(it.index_match).x << "\ty: " << target_keypointsNormal->at(it.index_match).y << "\tz: " << target_keypointsNormal->at(it.index_match).z << endl;
	}
	processing_time << endl;
	processing_time << endl;
	for (pcl::Correspondence it : *correspondences2) {
		processing_time << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
		//processing_time << "x: " << target_keypointsNormal->at(it.index_query).x << "\ty: " << target_keypointsNormal->at(it.index_query).y << "\tz: " << target_keypointsNormal->at(it.index_query).z << endl;
		//processing_time << "x: " << source_keypointsNormal->at(it.index_match).x << "\ty: " << source_keypointsNormal->at(it.index_match).y << "\tz: " << source_keypointsNormal->at(it.index_match).z << endl;
	}

	// 誤対応除去
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);
	processing_time << endl;
	processing_time << endl;
	for (pcl::Correspondence it : *pCorrespondences) {
		processing_time << "i: " << it.index_query << "\tj: " << it.index_match << "\td: " << it.distance << endl;
		//processing_time << "x: " << target_keypointsNormal->at(it.index_query).x << "\ty: " << target_keypointsNormal->at(it.index_query).y << "\tz: " << target_keypointsNormal->at(it.index_query).z << endl;
		//processing_time << "x: " << source_keypointsNormal->at(it.index_match).x << "\ty: " << source_keypointsNormal->at(it.index_match).y << "\tz: " << source_keypointsNormal->at(it.index_match).z << endl;
	}



	// 誤対応除去
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsNormal, target_keypointsNormal);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*source_keypointsNormal, *source_keypointsXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*target_keypointsNormal, *target_keypointsXYZ);
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	correspondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);*/


	//pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(target_features, source_features);
	//correspondences = myRefiningMatching2_2(correspondences, target_keypointsXYZ, source_keypointsXYZ);


	//std::vector<int>  correspondences = myFindMatching_SHOT352(source_features, target_features);
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);


	//pcl::CorrespondencesPtr correspondences = myFindMatching_FPFH33_3(source_features, target_features);
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_3(correspondences, source_keypointsNormal, target_keypointsNormal);

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*source_keypointsNormal, *source_keypointsXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*target_keypointsNormal, *target_keypointsXYZ);
	pcl::CorrespondencesPtr correspondences = myFindMatching_FPFH33_3(source_features, target_features);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_3(correspondences, source_keypointsXYZ, target_keypointsXYZ);*/

	
	// RとTの推定
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);


	// RとTの推定
	/*Eigen::Matrix4f transformation;
	std::cout << "Estimating transformation" << std::endl;
	pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ> est;
	est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *correspondences, transformation);
	std::cout << transformation << std::endl;*/


	
	// RとTの推定
	Eigen::Matrix4f transformation;
	//Eigen::Matrix4f transformation = Eigen::Matrix4d::Identity();
	std::cout << "Estimating transformation" << std::endl;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;
	est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
	std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//print4x4Matrix(transformation);
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));


	Eigen::Matrix4f transformation2;
	std::cout << "Estimating transformation2" << std::endl;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est2;
	est2.estimateRigidTransformation(*target_keypointsNormal, *source_keypointsNormal, *correspondences2, transformation2);
	std::cout << transformation2 << std::endl;
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));


	Eigen::Matrix4f transformation3;
	std::cout << "Estimating transformation3" << std::endl;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
	est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);
	std::cout << transformation3 << std::endl;
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", transformation3(0, 0), transformation3(0, 1), transformation3(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation3(1, 0), transformation3(1, 1), transformation3(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", transformation3(2, 0), transformation3(2, 1), transformation3(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", transformation3(0, 3), transformation3(1, 3), transformation3(2, 3));


	std::cout << "Estimating transformation4" << std::endl;
	pcl::registration::TransformationEstimation< pcl::PointNormal, pcl::PointNormal>::Ptr transformation_estimation(new pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal>);
	Eigen::Matrix4f transformation4;
	transformation_estimation->estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation4);
	std::cout << transformation4 << std::endl;
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", transformation4(0, 0), transformation4(0, 1), transformation4(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation4(1, 0), transformation4(1, 1), transformation4(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", transformation4(2, 0), transformation4(2, 1), transformation4(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", transformation4(0, 3), transformation4(1, 3), transformation4(2, 3));



	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(source_cloud);
	icp.setInputTarget(target_cloud);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	//変換matrixを表示する
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = icp.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix);


	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp_keypoint;
	icp_keypoint.setInputCloud(source_keypointsNormal);
	icp_keypoint.setInputTarget(target_keypointsNormal);
	pcl::PointCloud<pcl::PointNormal> Final_keypoint;
	icp_keypoint.align(Final_keypoint);

	//変換matrixを表示する
	Eigen::Matrix4d transformation_matrix_keypoint = Eigen::Matrix4d::Identity();
	transformation_matrix_keypoint = icp_keypoint.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix_keypoint);
	



	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	icp2.setInputCloud(source_cloud);
	icp2.setInputTarget(target_cloud);

	icp2.setTransformationEpsilon(1e-6);
	icp2.setMaxCorrespondenceDistance(5.0f);
	icp2.setMaximumIterations(200);
	icp2.setEuclideanFitnessEpsilon(1.0f);
	icp2.setRANSACOutlierRejectionThreshold(1.0f);

	pcl::PointCloud<pcl::PointXYZ> Final2;
	icp2.align(Final2);

	//変換matrixを表示する
	Eigen::Matrix4d transformation_matrix2 = Eigen::Matrix4d::Identity();
	transformation_matrix2 = icp2.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix2);


	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp3;
	icp3.setInputCloud(source_keypointsNormal);
	icp3.setInputTarget(target_keypointsNormal);

	icp3.setTransformationEpsilon(1e-6);
	icp3.setMaxCorrespondenceDistance(5.0f);
	icp3.setMaximumIterations(200);
	icp3.setEuclideanFitnessEpsilon(1.0f);
	icp3.setRANSACOutlierRejectionThreshold(1.0f);

	pcl::PointCloud<pcl::PointNormal> Final3;
	icp3.align(Final3);

	//変換matrixを表示する
	Eigen::Matrix4d transformation_matrix3 = Eigen::Matrix4d::Identity();
	transformation_matrix3 = icp3.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix3);




	//回転
	//pcl::transformPointCloud(in_point_cloud, out_point_cloud, rotation_matrix);
	//pcl::transformPointCloudWithNormals(*source_keypointsNormal, *target_keypointsNormal, transformation_matrix3.matrix);
	//pcl::transformPointCloudWithNormals(*source_keypointsNormal, *target_keypointsNormal, transformation_matrix3);

	
	pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

	pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

	pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);



	cout << "correspondences->size(): " << correspondences->size() << endl;
	cout << "correspondences2->size(): " << correspondences2->size() << endl;
	cout << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
	cout << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
	cout << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;


	/*DIR *dir;
	struct dirent *dp;
	char path[64] = "..\\DataBase\\STL files\\";
	dir = opendir(path);

	for (dp = readdir(dir); dp != NULL; dp = readdir(dir)) {
		printf("%s\n", dp->d_name);
	}

	closedir(dir);*/

	//const fs::path path("..\\DataBase\\STL files");

	/*BOOST_FOREACH(const fs::path& p, std::make_pair(fs::directory_iterator(path), fs::directory_iterator())) {
		if (!fs::is_directory(p))
			std::cout << p.filename() << std::endl;
	}*/

	// この例の場合、 e は fs::path 型ではなく fs::directory_entry 型となる点に注意する。
	/*for (const auto& e : boost::make_iterator_range(fs::directory_iterator(path), {}))
		if (!fs::is_directory(e))
			std::cout << e.path().filename() << std::endl;*/

	/*
	vector<string> datafiles;

	const fs::path path("..\\DataBase");

	BOOST_FOREACH(const fs::path& p, std::make_pair(fs::recursive_directory_iterator(path),
		fs::recursive_directory_iterator())) {
		if (!fs::is_directory(p))
			std::cout << p << std::endl;
	}
	*/
	

	/*Eigen::Matrix4d transformation5;
	//std::cout << std::endl;
	//std::cout << transformation5 << std::endl;
	//transformation5.isZero();
	//std::cout << std::endl;
	//std::cout << transformation5 << std::endl;
	//std::cout << std::endl;
	transformation5(0, 0) = 1.0f;	transformation5(0, 1) = 0.0f;	transformation5(0, 2) = 0.0f;	transformation5(0, 3) = 70.0f;
	transformation5(1, 0) = 0.0f;	transformation5(1, 1) = 1.0f;	transformation5(1, 2) = 0.0f;	transformation5(1, 3) = 50.0f;
	transformation5(2, 0) = 0.0f;	transformation5(2, 1) = 0.0f;	transformation5(2, 2) = 1.0f;	transformation5(2, 3) = 0.0f;
	transformation5(3, 0) = 0.0f;	transformation5(3, 1) = 0.0f;	transformation5(3, 2) = 0.0f;	transformation5(3, 3) = 1.0f;
	//std::cout << transformation5 << std::endl;
	//std::cout << std::endl;

	pcl::transformPointCloud(*target_cloud, *target_cloud, transformation5);
	pcl::transformPointCloudWithNormals(*target_keypointsNormal, *target_keypointsNormal, transformation5);*/

	// 表示のためにオフセット
	OFFSET offset;
	offset.x = 70.0f;
	offset.y = 50.0f;
	offset.z = 0.0f;
	addingOffset(target_cloud, offset);
	addingOffsetN(target_keypointsNormal, offset);


	// 入力点群と法線を表示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// 表示時のパラメータ
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	/*
	// createViewPort
	int vp[2];
	viewer->createViewPort(0.0f, 0.0f, 0.5f, 1.0f, vp[0]);
	viewer->createViewPort(0.5f, 0.0f, 1.0f, 1.0f, vp[1]);

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color(source_cloud, 255, 255, 255);
	viewer->addPointCloud(source_cloud, source_cloud_color, "source_cloud", vp[0]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud", vp[0]);

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> source_keypointsNormal_color(source_keypointsNormal, 0, 255, 0);
	viewer->addPointCloud(source_keypointsNormal, source_keypointsNormal_color, "source_keypointsNormal", vp[0]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source_keypointsNormal", vp[0]);
	viewer->addPointCloudNormals<pcl::PointNormal>(source_keypointsNormal, 1, 7.0, "source_keypointsNormaln", vp[0]);

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_cloud, 255, 255, 255);
	viewer->addPointCloud(target_cloud, target_cloud_color, "target_cloud", vp[0]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud", vp[0]);

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> target_keypointsNormal_color(target_keypointsNormal, 0, 255, 0);
	viewer->addPointCloud(target_keypointsNormal, target_keypointsNormal_color, "target_keypointsNormal",vp[0]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target_keypointsNormal", vp[0]);
	viewer->addPointCloudNormals<pcl::PointNormal>(target_keypointsNormal, 1, 7.0, "target_keypointsNormaln", vp[0]);

	// 対応の表示
	viewer->addCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *correspondences, "correspondences", vp[0]);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspondences", vp[0]);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "correspondences", vp[0]);


	// 入力点群の表示
	viewer->addPointCloud(source_cloud, source_cloud_color, "source_cloud2", vp[1]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud2", vp[1]);

	// 特徴点の表示
	viewer->addPointCloud(source_keypointsNormal, source_keypointsNormal_color, "source_keypointsNormal2", vp[1]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source_keypointsNormal2", vp[1]);
	viewer->addPointCloudNormals<pcl::PointNormal>(source_keypointsNormal, 1, 7.0, "source_keypointsNormaln2", vp[1]);

	// 入力点群の表示
	viewer->addPointCloud(target_cloud, target_cloud_color, "target_cloud2", vp[1]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud2", vp[1]);

	// 特徴点の表示
	viewer->addPointCloud(target_keypointsNormal, target_keypointsNormal_color, "target_keypointsNormal2", vp[1]);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target_keypointsNormal2", vp[1]);
	viewer->addPointCloudNormals<pcl::PointNormal>(target_keypointsNormal, 1, 7.0, "target_keypointsNormaln2", vp[1]);

	// 対応の表示
	viewer->addCorrespondences<pcl::PointNormal>(target_keypointsNormal, source_keypointsNormal, *correspondences2, "correspondences2", vp[1]);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspondences2", vp[1]);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "correspondences2", vp[1]);
	*/

	
	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color(source_cloud, 255, 255, 255);
	viewer->addPointCloud(source_cloud, source_cloud_color, "source_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> source_keypointsNormal_color(source_keypointsNormal, 0, 255, 0);
	viewer->addPointCloud(source_keypointsNormal, source_keypointsNormal_color, "source_keypointsNormal");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "source_keypointsNormal");
	viewer->addPointCloudNormals<pcl::PointNormal>(source_keypointsNormal, 1, 7.0, "source_keypointsNormaln");

	// 入力点群の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_cloud, 255, 255, 255);
	viewer->addPointCloud(target_cloud, target_cloud_color, "target_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

	// 特徴点の表示
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> target_keypointsNormal_color(target_keypointsNormal, 0, 255, 0);
	viewer->addPointCloud(target_keypointsNormal, target_keypointsNormal_color, "target_keypointsNormal");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target_keypointsNormal");
	viewer->addPointCloudNormals<pcl::PointNormal>(target_keypointsNormal, 1, 7.0, "target_keypointsNormaln");

	// 対応の表示
	viewer->addCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *pCorrespondences3, "correspondences");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspondences");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "correspondences");
	
	
	// 対応の表示
	/*switch (viewParam) {
	case 1:
		viewer->addCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *correspondences, "correspondences");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspondences");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "correspondences");
		break;
	case 2:
		viewer->addCorrespondences<pcl::PointNormal>(target_keypointsNormal, source_keypointsNormal, *correspondences2, "correspondences2");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "correspondences2");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "correspondences2");
		break;
	case 3:
		viewer->addCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *pCorrespondences, "pCorrespondences");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "pCorrespondences");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "pCorrespondences");
		break;
	default:
		break;
	}*/

	// 表示時のパラメータ
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();



	viewer->registerKeyboardCallback(keyboardEventOccurred2, (void*)viewer.get());

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		switch (viewParam) {
		case 1:
			viewer->updateCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *correspondences, "correspondences");
			break;
		case 2:
			viewer->updateCorrespondences<pcl::PointNormal>(target_keypointsNormal, source_keypointsNormal, *correspondences2, "correspondences");
			break;
		case 3:
			viewer->updateCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *pCorrespondences, "correspondences");
			break;
		case 4:
			viewer->updateCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *pCorrespondences2, "correspondences");
			break;
		case 5:
			viewer->updateCorrespondences<pcl::PointNormal>(source_keypointsNormal, target_keypointsNormal, *pCorrespondences3, "correspondences");
			break;
		}
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


#endif // _Simulation20171113_HPP_