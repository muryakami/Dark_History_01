#ifndef _Simulation20180207_HPP_
#define _Simulation20180207_HPP_

#include "Simulation20171113.hpp"
#include "MakeModelPPF.hpp"


pcl::PointCloud<DescriptorType>::Ptr procedure3(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice, pcl::PointCloud<pcl::PointNormal>::Ptr attention_point, std::mt19937 engine, float density) {

	// STLデータの内挿補間
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl_density(filename, density);
	

	// 点群の格子化（ダウンサンプリング）
	/*pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);*/

	// ダウンサンプリングしない
	cloud_lattice = cloud_in;

	// 法線の生成
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);


	// 特徴点検出
	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	// 特徴点の法線生成
	//*attention_point = *myKeypoint_normals(cloud_lattice, keypointsXYZ);



	// Harris, 提案手法　オクルージョンありで
	// オクルージョンの作成
	/*pcl::PointXYZ view_point(0.0f, 0.0f, 0.0f); // 視点
	//pcl::PointXYZ view_point(20.0f, 20.0f, -20.0f); // 視点
	pcl::PointCloud<pcl::PointNormal>::Ptr occlusion_cloud_normals = make_occlusion(cloud_normals, view_point, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr occlusion_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	occlusion_cloud->points.resize(occlusion_cloud_normals->points.size());
	pcl::copyPointCloud(*occlusion_cloud_normals, *occlusion_cloud);
	// Harris特徴点検出
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(occlusion_cloud);
	// Harris特徴点の中で独自性の高いものを抽出
	*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, occlusion_cloud_normals);*/


	// Harris, 提案手法
	// Harris特徴点検出
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr keypointsXYZINormal = myFeaturePointExtraction_HarrisN4(cloud_lattice);
	// Harris特徴点の中で独自性の高いものを抽出
	*attention_point = *myFeaturePointExtractionRe2(keypointsXYZINormal, cloud_normals);


	// ランダム法（シャッフル）
	/*std::shuffle(cloud_normals->begin(), cloud_normals->end(), engine);
	int numThreshold = 100; //15 20
	int count = 0;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_normals->begin(); it != cloud_normals->end(); it++) {
		if (count >= numThreshold) break; //
		attention_point->push_back(*it);
		count++;
	}*/


	// 特徴量の記述
	return myFeatureDescription_4(cloud_normals, attention_point);

}


void simulation20180207() {

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// 乱数生成のためのシード
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// 工具データ読み込み
	const string filenameT = ".\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";
	// ターゲット
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filenameT, target_cloud, target_keypointsNormal, engine, densityT);


	// 工具データ読み込み
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filenameS)) {

			// 現在のパス
			filenameIO << filenameS << std::endl;
			
			// ソース
			float densityS = 2.0f;
			pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

			// 対応点探索
			pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
			pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

			// 誤対応除去
			pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);


			// RとTの推定
			//Eigen::Matrix4f transformation;
			//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;
			//est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
			//Eigen::Matrix4f transformation2;
			//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est2;
			//est2.estimateRigidTransformation(*target_keypointsNormal, *source_keypointsNormal, *correspondences2, transformation2);
			Eigen::Matrix4f transformation3;
			pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
			est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);



			//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

			//変換matrixを表示する
			Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
			transformation_matrix = icp.getFinalTransformation().cast<double>();
			print4x4Matrix(transformation_matrix);


			//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

			//変換matrixを表示する
			//Eigen::Matrix4d transformation_matrix_keypoint = Eigen::Matrix4d::Identity();
			//transformation_matrix_keypoint = icp_keypoint.getFinalTransformation().cast<double>();
			//print4x4Matrix(transformation_matrix_keypoint);


			// 点群の剛体変換（ICPアルゴリズムによる推定）
			pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

			// 誤対応除去（初期対応点の正誤判定）
			pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

			// 誤対応除去（回転後の対応点）
			pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


			filenameIO << "correspondences->size(): " << correspondences->size() << endl;
			filenameIO << "correspondences2->size(): " << correspondences2->size() << endl;
			filenameIO << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
			filenameIO << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
			filenameIO << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
			filenameIO << endl;
		}
	}

	// 処理時間の出力
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;
}



void simulation20180209_1() {

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// 乱数生成のためのシード
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\Tool\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";

	// ターゲット
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filename, target_cloud, target_keypointsNormal, engine, densityT);

	// 出力
	const string fileO = "C:\\Users\\yuki\\Documents\\DataBase\\Keypoints\\STL files\\NikonTurningTool.txt";
	fs::ofstream savefile(fileO);

	for (float densityS = densityT; 0.0f < densityS; densityS -= 0.2f) {

		// ソース
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filename, source_cloud, source_keypointsNormal, engine, densityS);

		// 対応点探索
		pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
		pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

		// 誤対応除去
		pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

		// RとTの推定
		//Eigen::Matrix4f transformation3;
		//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
		//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

		//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

		// 点群の剛体変換（ICPアルゴリズムによる推定）
		pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

		// 誤対応除去（初期対応点の正誤判定）
		pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

		// 誤対応除去（回転後の対応点）
		pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


		//savefile << "correspondences->size(): " << correspondences->size() << endl;
		//savefile << "correspondences2->size(): " << correspondences2->size() << endl;
		//savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
		//savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
		//savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;

		// マッチングに有効な特徴点（正対応）率の出力
		savefile << (float)pCorrespondences3->size() / correspondences2->size() * 100 << "\t";
	}
	//savefile << endl;
	savefile.close();

}


void simulation20180209_2() {

	std::chrono::system_clock::time_point start, end; // 型は auto で可
	double elapsed;

	// 乱数生成のためのシード
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// 工具データ読み込み
	const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\Tool\\STL files\\NikonTurningTool.STL";
	//const string filename = "..\\DataBase\\tool\\2020\\BRS2000S25.STL";

	// ターゲット
	float densityT = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filename, target_cloud, target_keypointsNormal, engine, densityT);

	// 出力
	const string fileO = "C:\\Users\\yuki\\Documents\\DataBase\\Keypoints\\STL files\\NikonTurningTool.txt";
	fs::ofstream savefile(fileO);

	for (float densityS = densityT; 0.0f < densityS; densityS -= 0.2f) {

		// ソース
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filename, source_cloud, source_keypointsNormal, engine, densityS);

		// 対応点探索
		pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
		pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

		// 誤対応除去
		pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

		// RとTの推定
		//Eigen::Matrix4f transformation3;
		//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
		//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

		//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

		// 点群の剛体変換（ICPアルゴリズムによる推定）
		pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

		// 誤対応除去（初期対応点の正誤判定）
		pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

		// 誤対応除去（回転後の対応点）
		pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


		//savefile << "correspondences->size(): " << correspondences->size() << endl;
		//savefile << "correspondences2->size(): " << correspondences2->size() << endl;
		//savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
		//savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
		//savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;

		// マッチングに有効な特徴点（正対応）率の出力
		savefile << (float)pCorrespondences3->size() / correspondences2->size() * 100 << "\t";
	}
	//savefile << endl;
	savefile.close();



	/*
	// 工具データ読み込み
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
	if (!fs::is_directory(filenameS)) {

	// 現在のパス
	filenameIO << filenameS << std::endl;

	// ソース
	float densityS = 2.0f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

	// 対応点探索
	pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
	pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

	// 誤対応除去
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);


	// RとTの推定
	//Eigen::Matrix4f transformation;
	//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est;
	//est.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *correspondences, transformation);
	//Eigen::Matrix4f transformation2;
	//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est2;
	//est2.estimateRigidTransformation(*target_keypointsNormal, *source_keypointsNormal, *correspondences2, transformation2);
	Eigen::Matrix4f transformation3;
	pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
	est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);



	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

	//変換matrixを表示する
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = icp.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix);


	//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

	//変換matrixを表示する
	//Eigen::Matrix4d transformation_matrix_keypoint = Eigen::Matrix4d::Identity();
	//transformation_matrix_keypoint = icp_keypoint.getFinalTransformation().cast<double>();
	//print4x4Matrix(transformation_matrix_keypoint);


	// 点群の剛体変換（ICPアルゴリズムによる推定）
	pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

	// 誤対応除去（初期対応点の正誤判定）
	pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

	// 誤対応除去（回転後の対応点）
	pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


	filenameIO << "correspondences->size(): " << correspondences->size() << endl;
	filenameIO << "correspondences2->size(): " << correspondences2->size() << endl;
	filenameIO << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
	filenameIO << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
	filenameIO << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
	filenameIO << endl;
	}
	}

	// 処理時間の出力
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;
	*/
}


void simulation20180209_3() {

	// 乱数生成のためのシード
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	// 時間出力
	const string fileTime = "C:\\Users\\yuki\\Documents\\DataBase\\Matching\\time.txt";
	fs::ofstream savefileTime(fileTime);

	// 工具データ読み込み
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filenameT, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filenameT)) {

			// 出力するパス変数
			fs::path fileO;

			// ディレクトリを変更
			for (auto& part : filenameT) {
				if (part == "Tool")
					fileO.append("Matching");
				else
					fileO.append(part.c_str());
			}

			// 拡張子を変更
			fileO.replace_extension("txt");

			// 出力
			fs::ofstream savefile(fileO);

			// ターゲット
			float densityT = 2.0f;
			pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<DescriptorType>::Ptr target_features = procedure3(filenameT.string(), target_cloud, target_keypointsNormal, engine, densityT);

			BOOST_FOREACH(const fs::path& filenameS, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
				if (!fs::is_directory(filenameS)) {

					// 時間計測
					std::chrono::system_clock::time_point start, end; // 型は auto で可
					double elapsed;

					// 現在のパス
					savefile << filenameS << std::endl;

					// ソース
					float densityS = 2.0f;
					pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal(new pcl::PointCloud<pcl::PointNormal>);
					pcl::PointCloud<DescriptorType>::Ptr source_features = procedure3(filenameS.string(), source_cloud, source_keypointsNormal, engine, densityS);

					// 対応点探索
					pcl::CorrespondencesPtr correspondences = myFindMatching_3(source_features, target_features);
					pcl::CorrespondencesPtr correspondences2 = myFindMatching_3(target_features, source_features);

					// 誤対応除去
					pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching_3(correspondences, correspondences2, source_keypointsNormal, target_keypointsNormal);

					// RとTの推定
					//Eigen::Matrix4f transformation3;
					//pcl::registration::TransformationEstimationSVD< pcl::PointNormal, pcl::PointNormal> est3;
					//est3.estimateRigidTransformation(*source_keypointsNormal, *target_keypointsNormal, *pCorrespondences, transformation3);

					//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
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

					// 点群の剛体変換（ICPアルゴリズムによる推定）
					pcl::transformPointCloudWithNormals(*source_keypointsNormal, *source_keypointsNormal, transformation_matrix);

					// 誤対応除去（初期対応点の正誤判定）
					pcl::CorrespondencesPtr pCorrespondences2 = nearest_search_test4(source_keypointsNormal, target_keypointsNormal, pCorrespondences);

					// 誤対応除去（回転後の対応点）
					pcl::CorrespondencesPtr pCorrespondences3 = nearest_search_test5(source_keypointsNormal, target_keypointsNormal, pCorrespondences);


					savefile << "correspondences->size(): " << correspondences->size() << endl;
					savefile << "correspondences2->size(): " << correspondences2->size() << endl;
					savefile << "pCorrespondences->size(): " << pCorrespondences->size() << endl;
					savefile << "pCorrespondences2->size(): " << pCorrespondences2->size() << endl;
					savefile << "pCorrespondences3->size(): " << pCorrespondences3->size() << endl;
					savefile << endl;

					// 処理時間の出力
					elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //処理に要した時間をミリ秒に変換
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

	// 工具データ読み込み
	const fs::path path("..\\DataBase\\Tool");
	//const fs::path path("C:\\Users\\yuki\\Documents\\DataBase\\Tool");
	BOOST_FOREACH(const fs::path& filename, std::make_pair(fs::recursive_directory_iterator(path), fs::recursive_directory_iterator())) {
		if (!fs::is_directory(filename)) {

			// 現在のパス
			filenameIO << filename << std::endl;

			// 特徴点を出力するパス変数
			fs::path fileO;

			// ディレクトリを変更
			for (auto& part : filename) {
				if (part == "Tool")
					fileO.append("Keypoints");
				else
					fileO.append(part.c_str());
			}

			// 拡張子を変更
			fileO.replace_extension("txt");

			// 点群データの生成
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl2(filename.string());


			// 特徴点の出力
			fs::ofstream savefile(fileO);
			for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
				savefile << it->x << " " << it->y << " " << it->z << " " << endl;
			}
			savefile.close();

		}
	}

}



#endif // _Simulation20180207_HPP_