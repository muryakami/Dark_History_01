#include "main.h"



















/*
// �@������
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// �_���
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// �����_�̖@������
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // �S�̂̏d�S���v�Z
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//���_�̌���
ne.setInputCloud(cloud);//�@���̌v�Z���s�������_�Q���w�肷��
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREE�����
ne.setSearchMethod(tree);//�������@��KDTREE���w�肷��
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);//�@����������ϐ�
ne.setRadiusSearch(3.0);//�������锼�a���w�肷�� 3.0, 1.0
ne.compute(*cloud_normals);//�@�����̏o�͐���w�肷��

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).x = cloud->at(i).x;
cloud_normals->at(i).y = cloud->at(i).y;
cloud_normals->at(i).z = cloud->at(i).z;
}

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).normal_x *= (-1);
cloud_normals->at(i).normal_y *= (-1);
cloud_normals->at(i).normal_z *= (-1);
}

return cloud_normals;


}

pcl::PointNormal flipNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ attention_point, pcl::PointXYZ average_point) {
array<array<double, 3>, 3> point_pca = cal_PCA(cloud);
pcl::PointNormal normal;
normal.x = attention_point.x;
normal.y = attention_point.y;
normal.z = attention_point.z;
normal.normal_x = point_pca[0][0];
normal.normal_y = point_pca[0][1];
normal.normal_z = point_pca[0][2];

pcl::PointXYZ n(point_pca[0][0], point_pca[0][1], point_pca[0][2]);
pcl::PointXYZ vppi(average_point.x - attention_point.x, average_point.y - attention_point.y, average_point.z - attention_point.z);
if (dot_product3_cal(n, vppi) > 0) {
normal.normal_x *= (-1);
normal.normal_y *= (-1);
normal.normal_z *= (-1);
}

return normal;
}

// �@������
pcl::PointCloud<pcl::PointNormal>::Ptr myEstimatingNormals2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsXYZ) {

// �_���
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

// �����_�̖@������
pcl::search::KdTree<pcl::PointXYZ> kdtree;
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;
kdtree.setInputCloud(cloud);

//Eigen::Vector4f xyz_centroid; // �S�̂̏d�S���v�Z
//pcl::compute3DCentroid(*cloud, xyz_centroid);

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = keypointsXYZ->begin(); it != keypointsXYZ->end(); it++) {
if (kdtree.radiusSearch(*it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroid; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroid);
pcl::PointXYZ cloud_average(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}

return cloud_ptr;


//�@����������ϐ�
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);

// �p�����[�^
float radius = 3.0f; // 3.0f 5.0f 7.0f 0.3f

Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//KDTREE�����
pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
ne.setSearchMethod(tree);// �������@��KDTREE���w�肷��
ne.setInputCloud(keypointsXYZ); // �@���̌v�Z���s�������_�Q���w�肷��
ne.setSearchSurface(cloud); // �@���v�Z�Ɏg���_�Q���w�肷��
ne.setRadiusSearch(radius);//�������锼�a���w�肷�� 3.0, 1.0
ne.setViewPoint(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);//���_�̌���
ne.compute(*cloud_normals);//�@�����̏o�͐���w�肷��


// ne.setInputCloud(keypointsXYZ);	�Ŗ@�����v�Z�����ꍇ��
// ne.setSearchSurface(cloud);		��ǉ����Ė@�����v�Z�����ꍇ�̈Ⴂ���m�F����(visual��)
// ���̌�Ckdtree�ɉ��������Ă��邩�m�F(cloud�������Ă���Ƃ悢�C�����炭�Ⴄ����)
// kdtree��keypointsXYZ�������Ă����ꍇ�͐V���ɍ�蒼��(cloud��������kdtree)

for (pcl::PointNormal it : *cloud_normals) {
if (kdtree.radiusSearch(it, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud(new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
neighborhood_cloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
}
Eigen::Vector4f xyz_centroidN; // �ߖT�̏d�S���v�Z
pcl::compute3DCentroid(*neighborhood_cloud, xyz_centroidN);
pcl::PointXYZ cloud_average(xyz_centroidN[0], xyz_centroidN[1], xyz_centroidN[2]);
pcl::PointNormal normal = normal_direction(neighborhood_cloud, *it, cloud_average);
cloud_ptr->push_back(normal);
}
}


for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).x = cloud->at(i).x;
cloud_normals->at(i).y = cloud->at(i).y;
cloud_normals->at(i).z = cloud->at(i).z;
}

for (int i = 0; i < cloud->size(); i++) {
cloud_normals->at(i).normal_x *= (-1);
cloud_normals->at(i).normal_y *= (-1);
cloud_normals->at(i).normal_z *= (-1);
}

return cloud_normals;


}
*/







#include <pcl/features/principal_curvatures.h>
// --------------
// -----Main-----
// --------------
int main(int argc, char* argv[]) {
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\06\\outputfile\\result004.txt");
	//ofstream outputfile("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�ł����킹\\20\\outputfile\\20_5\\test\\endmill\\output000.txt");
	//std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	//double elapsed;


	// ------------------------------------------
	// �H��z���_�[�ʂƍH��˂��o���ʂ����߂鏈��
	// ------------------------------------------


	// �H��p���i�Œ�j
	/*pcl::Normal tool_normal(0.0f, 0.0f, 1.0f);
	float toolN_norm = sqrt((tool_normal.normal_x * tool_normal.normal_x) + (tool_normal.normal_y * tool_normal.normal_y) + (tool_normal.normal_z * tool_normal.normal_z));
	double theta_cos = 0.9962; // 5�x*/


	/*
	// ����f�[�^�ǂݍ��݁i�S�āj
	const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\dr1_whitesprayed2mod2.stl";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\ex1_whitesprayed2mod2.stl";
	STLDATA stl_object(filename);

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\endmill\\OCTACUT322S32RB.STL";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\drill_20170119\\merged_pointcloud_�m�C�Y�����L_ascii.stl";
	//STLDATA stl_object(filename);
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\2017_03_03\\5.0_-5.0.STL";
	//STLDATA stl_object(filename);



	// ����f�[�^�ǂݍ��݁i�H���[���j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	/////// �H���[�̂� ///////
	// �h����
	//if (point.y > -6) continue;	// �O��l�̏���������΂����OK
	if (point.y > -7) continue;		// �O��l�̏����Ȃ�
	//if (point.y > -20) continue;
	//if (point.y < -21) continue;
	// �o�C�g
	//if (point.y > -35) continue;	// ���s���̂�
	//if (point.y > -27) continue;
	//if (point.z < 65) continue;
	////////////////////////////
	cloud_in->points.push_back(point);
	}
	}
	//cout << "cloud_in->size(): " << cloud_in->size() << endl;


	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypointsCurvature->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypointsCurvature->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypointsCurvature->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(keypoint_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(keypointsCurvature, false);
	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	cout << "pointN: " << keypointsCurvature->size() << endl;


	// �����_����
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	//detector.setNonMaxSupression(false);
	detector.setRadius(5.0);//3.0 5.0 7.0
	detector.setInputCloud(keypointsCurvature);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
	tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
	keypoints3D->push_back(tmp);
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D = Remove_outliers2(keypoints3D_0, false);


	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(keypointsCurvature);
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(keypointsCurvature, keypoints3D->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}
	*/









	/*
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\dr1_whitesprayed2mod2.stl";
	const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\object\\ex1_whitesprayed2mod2.stl";

	// ����f�[�^�ǂݍ��݁i�H���[���j
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in0 = stl_points0(filename);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in1 = stl_nAveraging(cloud_in0);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in2 = stl_decimating0(cloud_in1);

	// ����f�[�^�ǂݍ��݁i�H���[���j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = stl_pointsXYZ(filename);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);

	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	// �����_�̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(surfaceN);
	*/

	/*
	// ����f�[�^��PPF
	vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);
	*/



	/*
	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);
	*/













	// �H��f�[�^�ǂݍ���
	//const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";

	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\������.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\������.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\Geometry\\�~��.STL";



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);

	// �����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myFeaturePointExtraction(cloud_normals);
	// �H��f�[�^��PPF
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> source_PPF = make_PPFs(attention_point);
	vector<myPPF> source_PPF = make_lightPPFs(attention_point);
	*/



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	// �_�Q�f�[�^
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�O�a�o�C�g_PVJNR2525M.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�O�a�o�C�g_STGER2020K16.txt";
	//const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����_MDW0300GS4.txt";
	const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����_MWE0300MA.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_in2);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in2, false); // �_�Q�f�[�^�̂Ƃ�
	// RANSAC ���ʏ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Detect_wall(cloud_in, true);
	// �O��l�̏���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2 = Remove_outliers(cloud_lattice, false); // �_�Q�f�[�^�̂Ƃ�

	cout << "cloud_in->size(): " << cloud_in->size() << endl;
	//cout << "cloud_in2->size(): " << cloud_in2->size() << endl;
	//cout << "cloud->size(): " << cloud->size() << endl;
	//cout << "cloud_lattice->size(): " << cloud_lattice->size() << endl;
	cout << "cloud_lattice2->size(): " << cloud_lattice2->size() << endl;
	*/



	/*
	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // true�Ńm�C�Y true
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f); // 0.5f, 0.5f, 0.5f		10.0f, 10.0f, 10.0f
	sor2.filter(*cloud_lattice2);*/



	// �@���̐���
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// �����_���o�C�����ʌv�Z
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals2);
	//�I�N���[�W�����L�� (�e�X�g)
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction(cloud_normals3);*/


	// Harris �̓����_���o
	// �I�N���[�W�����Ȃ�
	/*pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(cloud_lattice2); //Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(cloud_lattice2); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN*/
	// �I�N���[�W��������
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals3 = make_occlusion(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_normals3, *surface0);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myFeaturePointExtraction_HarrisN(surface0); // Harris
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = ISS3Ddetector(surface0); // ISS
	//pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = SUSANdetector(cloud_lattice2); // SUSAN


	// �H��f�[�^��PPF
	/*vector<myPPF> target_PPF = make_lightPPFs(attention_point2);*/



	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = Detect_Cylinder(cloud_normals2);*/

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	/*
	// �f�[�^�}�b�`���O (�ȈՔ�)
	//pcl::PointCloud<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pair<pcl::PointXYZ, pcl::PointXYZ>>>::Ptr match_cloud = myPPF_matching(target_PPF, source_PPF);
	int match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "source_PPF.size(): " << source_PPF.size() << endl;
	outputfile777 << "target_PPF.size(): " << target_PPF.size() << endl;
	outputfile777 << "match_cloud->size(): " << match_cloud << endl;
	outputfile777 << "source_accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	outputfile777 << "target_accuracy: " << (float)match_cloud / target_PPF.size() * 100 << endl;

	outputfile777 << endl;
	*/












	/*
	ifstream outputfile456;
	string path;
	vector<myPPF> source_PPF;

	path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database\\06\\Alexandre\\NikonTurningTool.txt";
	string line;
	int count = 0;
	outputfile456.open(path, ios::in);

	getline(outputfile456, line); //

	while (getline(outputfile456, line)) {
	std::string word;
	std::istringstream stream(line);
	//while (getline(stream, word,  ' ')) {
	//double test = stod(word);
	//}
	myPPF sample;
	getline(stream, word, ' ');
	sample.distance = stod(word);
	getline(stream, word, ' ');
	sample.angle_between = stod(word);
	getline(stream, word, ' ');
	sample.angle_n1 = stod(word);
	getline(stream, word, ' ');
	sample.angle_n2 = stod(word);
	source_PPF.push_back(sample);
	}
	outputfile456.close();

	for (vector<myPPF>::iterator it = source_PPF.begin(); it != source_PPF.end(); it++) {
	cout << it->distance << " " << it->angle_between << " " << it->angle_n1 << " " << it->angle_n2 << endl;
	}
	*/









	/*
	string path;
	vector<myPPF> source_PPF;
	int match_cloud;

	path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database\\07\\Alexandre\\NikonTurningTool.txt";
	source_PPF = load_MyLightPPF(path);
	// �H��f�[�^���
	outputfile777 << path << endl;
	// �f�[�^�}�b�`���O (�ȈՔ�)
	match_cloud = myLightPPF_matching(target_PPF, source_PPF);

	outputfile777 << "match_cloud->size: " << match_cloud << endl;
	outputfile777 << "accuracy: " << (float)match_cloud / source_PPF.size() * 100 << endl;
	outputfile777 << endl;
	*/












	/*
	//std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	//double elapsed;
	// �_�Q�f�[�^
	//start = std::chrono::system_clock::now(); // �v���J�n����
	const string filename = "C:\\Users\\yuki\\Desktop\\���㑪��f�[�^\\�h����.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud);

	outputfile777 << "cloud_in->size(): " << cloud_in->size() << endl;
	outputfile777 << endl;
	end = std::chrono::system_clock::now();  // �v���I������
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/

















	/*
	// �f�[�^�}�b�`���O
	string path;
	string database_path;
	string feature_value = "";
	string type_company;
	string extension = ".STL";


	// 11��
	database_path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files";
	type_company = "";
	path = database_path + feature_value + type_company + "\\NikonTurningTool" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative0" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative2" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative3" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative4" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative5" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative6" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative7" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative8" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative9" + extension;
	save_MyLightPPF(path);

	// 15��
	database_path = "C:\\Users\\yuki\\Desktop\\yuki\\database";
	type_company = "\\mitsubishi\\bite";
	path = database_path + feature_value + type_company + "\\DCLNR3225P12" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DDJNR1616H11" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DDJNR2020K15" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DVJNR2020K16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MTJNR2525M16N" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MTJNR2525M22N" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PVJNR2020K16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PWLNR2525M06" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SDNCR0808K07-SM" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SRGCR1616H08" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SSSCR1212F09" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SVPPR1616M11-SM" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SVVCN1616H16" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SXZCR2020K15" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TLHR2020K5" + extension;
	save_MyLightPPF(path);

	// 47��
	type_company = "\\sumitomo";
	path = database_path + feature_value + type_company + "\\DTR55CL2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CL2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CR2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55CR2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QL2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QL2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QR2020-K17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\DTR55QR2525-M17" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-600S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-600W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-800S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDEL2525-800W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-600S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-600W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-800S" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\GDER2525-800W" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT35R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT55R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\PBT80R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SBT35R2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SFTR2020" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SGWR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SGWR1616" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0707-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0808-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBL0909-70" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR0808" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR0808-60" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1010" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1010-60" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1212" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\SPBR1212-60" + extension;
	save_MyLightPPF(path);

	// 12��
	type_company = "\\mitsubishi\\drill";
	path = database_path + feature_value + type_company + "\\BRA2000S25" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRA2500S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRA3000S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS2000S25" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS2500S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\BRS3000S32" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX2000X2F25 - MVX2000X2F25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX2500X2F25 - MVX2500X2F25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\MVX3000X2F32 - MVX3000X2F32-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN2000S25 - TAWSN2000S25-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN2500S32 - TAWSN2500S32-1" + extension;
	save_MyLightPPF(path);
	path = database_path + feature_value + type_company + "\\TAWSN3000S32 - TAWSN3000S32-1" + extension;
	save_MyLightPPF(path);
	*/











	/*
	// �f�[�^�}�b�`���O
	string path;
	string database_path = "C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\PPF_database";
	string feature_value = "\\Harris_20";
	string type_company;
	string extension = ".txt";


	// 11��
	type_company = "\\Alexandre";
	path = database_path + feature_value + type_company + "\\NikonTurningTool" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative0" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative2" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative3" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative4" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative5" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative6" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative7" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative8" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\CuttingTool_Alternative9" + extension;
	save_accuracy(path, target_PPF);

	// 15��
	type_company = "\\mitsubishi\\bite";
	path = database_path + feature_value + type_company + "\\DCLNR3225P12" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DDJNR1616H11" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DDJNR2020K15" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DVJNR2020K16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MTJNR2525M16N" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MTJNR2525M22N" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PVJNR2020K16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PWLNR2525M06" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SDNCR0808K07-SM" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SRGCR1616H08" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SSSCR1212F09" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SVPPR1616M11-SM" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SVVCN1616H16" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SXZCR2020K15" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TLHR2020K5" + extension;
	save_accuracy(path, target_PPF);

	// 47��
	type_company = "\\sumitomo";
	path = database_path + feature_value + type_company + "\\DTR55CL2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CL2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CR2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55CR2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QL2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QL2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QR2020-K17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\DTR55QR2525-M17" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-600S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-600W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-800S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDEL2525-800W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-600S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-600W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-800S" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\GDER2525-800W" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT35R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT55R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\PBT80R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SBT35R2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SFTR2020" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SGWR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SGWR1616" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0707-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0808-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBL0909-70" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR0808" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR0808-60" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1010" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1010-60" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1212" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\SPBR1212-60" + extension;
	save_accuracy(path, target_PPF);

	// 12��
	type_company = "\\mitsubishi\\drill";
	path = database_path + feature_value + type_company + "\\BRA2000S25" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRA2500S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRA3000S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS2000S25" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS2500S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\BRS3000S32" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX2000X2F25 - MVX2000X2F25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX2500X2F25 - MVX2500X2F25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\MVX3000X2F32 - MVX3000X2F32-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN2000S25 - TAWSN2000S25-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN2500S32 - TAWSN2500S32-1" + extension;
	save_accuracy(path, target_PPF);
	path = database_path + feature_value + type_company + "\\TAWSN3000S32 - TAWSN3000S32-1" + extension;
	save_accuracy(path, target_PPF);
	*/










	/*
	// �H��f�[�^�ǂݍ��݁i�f�[�^�}�b�`���O�j
	// 11��
	string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative0.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative2.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative3.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative4.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative5.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative6.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative7.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative8.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative9.STL";
	data_matchingL(path, target_PPF);

	// 15��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DCLNR3225P12.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR1616H11.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR2020K15.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DVJNR2020K16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M16N.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M22N.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PWLNR2525M06.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SDNCR0808K07-SM.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SRGCR1616H08.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SSSCR1212F09.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVPPR1616M11-SM.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVVCN1616H16.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SXZCR2020K15.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\TLHR2020K5.STL";
	data_matchingL(path, target_PPF);

	// 47��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2020-K17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2525-M17.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800S.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800W.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR2020.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1616.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0707-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0808-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0909-70.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808-60.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010-60.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212-60.STL";
	data_matchingL(path, target_PPF);

	// 12��
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2000S25.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2500S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA3000S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS3000S32.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2500X2F25 - MVX2500X2F25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX3000X2F32 - MVX3000X2F32-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2500S32 - TAWSN2500S32-1.STL";
	data_matchingL(path, target_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN3000S32 - TAWSN3000S32-1.STL";
	data_matchingL(path, target_PPF);
	*/



































	/*
	// �H��f�[�^�ǂݍ���
	const string filename = ".\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Documents\\DataBase\\STL files\\NikonTurningTool.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	//const string filename = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";



	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = interpolation_stl(filename, false);
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = surface_normals(cloud_lattice);
	// �����_���o
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice);
	//pcl::PointCloud<pcl::PointNormal>::Ptr source_keypointsNormal = myFeaturePointExtraction(cloud_normals);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*source_keypointsNormal, *source_keypointsXYZ);
	// �����ʌv�Z
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features = myFeatureDescription_FPFH2(cloud_normals, source_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr source_features = myFeatureDescription_SHOT352(cloud_normals, source_keypointsXYZ);



	// STL�f�[�^�̓��}���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 = interpolation_stl(filename, true); // true�Ńm�C�Y true
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor2;
	sor2.setInputCloud(cloud_in2);
	sor2.setLeafSize(0.5f, 0.5f, 0.5f);
	sor2.filter(*cloud_lattice2);
	// �@���̐���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2 = surface_normals(cloud_lattice2);
	// �����_���o
	std::chrono::system_clock::time_point  start, end; // �^�� auto �ŉ�
	double elapsed;

	start = std::chrono::system_clock::now(); // �v���J�n����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN2(cloud_normals2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ = myFeaturePointExtraction_HarrisN3(cloud_lattice2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr target_keypointsNormal = myFeaturePointExtraction(cloud_normals2);
	end = std::chrono::system_clock::now();  // �v���I������

	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	processing_time << "input_cloud: " << elapsed << "[ms]" << endl;
	processing_time << endl;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypointsXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*target_keypointsNormal, *target_keypointsXYZ);
	// �����ʌv�Z
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = myFeatureDescription_FPFH2(cloud_normals2, target_keypointsXYZ);
	pcl::PointCloud<pcl::SHOT352>::Ptr target_features = myFeatureDescription_SHOT352(cloud_normals2, target_keypointsXYZ);




	// �Ή��_�T��
	//std::vector<int> correspondences = myFindMatching2(source_features, target_features);
	//std::vector<int> correspondences = myFindMatching_SHOT352(source_features, target_features);

	pcl::CorrespondencesPtr correspondences = myFindMatching_SHOT352_2(source_features, target_features);
	//pcl::CorrespondencesPtr correspondences = nearest_search_test2(source_keypointsXYZ, target_keypointsXYZ);


	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF = make_lightPPFs(attention_point);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF = make_lightPPFs(attention_point2);
	int match_cloud = myLightPPF_matching(keypoint_PPF, stl_PPF);

	accuracy_file << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;
	accuracy_file << "stl_PPF.size(): " << stl_PPF.size() << endl;
	accuracy_file << "match_cloud->size(): " << match_cloud << endl;
	accuracy_file << "stl_accuracy: " << (float)match_cloud / stl_PPF.size() * 100 << endl;
	accuracy_file << "keypoint_accuracy: " << (float)match_cloud / keypoint_PPF.size() * 100 << endl;

	accuracy_file << endl;

	// ��Ή�����
	//pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	pcl::CorrespondencesPtr pCorrespondences = myRefiningMatching2_2(correspondences, source_keypointsXYZ, target_keypointsXYZ);
	// R��T�̐���
	//Eigen::Matrix4f transformation = myEstimatingTransformation2(source_features);
	//pcl::registration::TransformationEstimationSVD< pcl::PointXYZ, pcl::PointXYZ > est;
	//est.estimateRigidTransformation(*source_keypointsXYZ, *target_keypointsXYZ, *pCorrespondences, transformation);
	//std::cout << transformation << std::endl;
	//pcl::transformPointCloud(*cloud_source, *cloud_source_trans, transformation);
	//pcl::transformPointCloud(*cloud_source_normals, *cloud_source_trans_normals, transformation);

	// �����_�̖@������
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point3 = myKeypoint_normals(cloud_lattice, source_keypointsXYZ);
	vector<myPPF> keypoint_PPF2 = make_lightPPFs(attention_point3);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point4 = myKeypoint_normals(cloud_lattice2, target_keypointsXYZ);
	vector<myPPF> stl_PPF2 = make_lightPPFs(attention_point4);
	int match_cloud2 = myLightPPF_matching(keypoint_PPF2, stl_PPF2);

	accuracy_file << "keypoint_PPF.size(): " << keypoint_PPF2.size() << endl;
	accuracy_file << "stl_PPF.size(): " << stl_PPF2.size() << endl;
	accuracy_file << "match_cloud->size(): " << match_cloud2 << endl;
	accuracy_file << "stl_accuracy: " << (float)match_cloud2 / stl_PPF2.size() * 100 << endl;
	accuracy_file << "keypoint_accuracy: " << (float)match_cloud2 / keypoint_PPF2.size() * 100 << endl;

	accuracy_file << endl;


	for (pcl::Correspondence i : *correspondences) {
	outputfile << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile << endl;

	for (pcl::Correspondence i : *pCorrespondences) {
	outputfile << "i: " << i.index_query << "\tj: " << i.index_match << "\tdistance: " << i.distance << endl;
	}
	outputfile << endl;
	*/


	/*for (int i = 0; i < correspondences.size(); i++) {
	outputfile777 << "[" << i << "]: " << correspondences[i] << endl;
	}
	outputfile777 << endl;

	for (int i = 0; i < pCorrespondences->size(); i++) {
	outputfile777 << "[" << i << "]: " << pCorrespondences->at(i) << endl;
	}
	outputfile777 << endl;*/


	





	//CorrespondenceGrouping2();









/*
	// �t�B�b�N�X�|�C���g�p

	// �H��f�[�^�ǂݍ���
	const string filename = ".\\STL files\\NikonTurningTool.STL";
	const string filename2 = ".\\STL files\\NikonTurningTool.STL";
	const string filename3 = ".\\STL files\\CuttingTool_Alternative0.STL";
	const string filename4 = ".\\STL files\\CuttingTool_Alternative5.STL";
	
	// PPF�̐���(STL�f�[�^:A)	�\�[�X
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> source_PPF = forFixPoint(filename, false, cloud_lattice, attention_point);

	// PPF�̐���(STL�f�[�^:A)	�^�[�Q�b�g
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> target_PPF2 = forFixPoint(filename2, true, cloud_lattice2, attention_point2);

	// PPF�̐���(STL�f�[�^:B)	�^�[�Q�b�g
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point3(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> target_PPF3 = forFixPoint(filename3, false, cloud_lattice3, attention_point3);

	// PPF�̐���(STL�f�[�^:C)	�^�[�Q�b�g
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point4(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> target_PPF4 = forFixPoint(filename4, false, cloud_lattice4, attention_point4);

	// ��v���̏o��
	outputAccuracy(&source_PPF, &target_PPF2);	// A��A
	outputAccuracy(&source_PPF, &target_PPF3);	// A��B
	outputAccuracy(&source_PPF, &target_PPF4);	// A��C
	*/



	/*
	// �H��f�[�^�ǂݍ���
	const string filename = ".\\STL files\\NikonTurningTool.STL";
	
	// PPF�̐���(STL�f�[�^:A)	�\�[�X
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> source_PPF = forFixPoint(filename, false, cloud_lattice, attention_point);

	// PPF�̐���(STL�f�[�^:A)	�^�[�Q�b�g
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point2(new pcl::PointCloud<pcl::PointNormal>);
	vector<myPPF> target_PPF2 = forFixPoint(filename, true, cloud_lattice2, attention_point2);

	// �덷�̏o��
	nearest_search_test3(attention_point, attention_point2);

	// ��v���̏o��
	outputAccuracy(&source_PPF, &target_PPF2);	// A��A
	*/


	//simulation20171030();
	//simulation20171113();
	//simulation20171127();
	simulation20180101();



	// ���̑̐ς��o��
	for(double degree=0.0f;degree<=180;degree+=10)
		calSphereVolume(degree);

	// ���̕\�ʐς��o��
	for (double degree = 0.0f; degree <= 180; degree += 10)
		calSphereSurface(degree);

	
	cout << endl;
	double hemisphere = 2.0f*M_PI;
	double unit = hemisphere / 16;
	/*for (double i = -hemisphere; i <= hemisphere; i += unit) {
		double c = 1 - i / (2.0f*M_PI);
		double rad = acos(c);
		double degree = rad * 180 / M_PI;
		cout << "���W�A��: " << rad << "\t�x: " << degree << endl;
	}*/
	cout << endl;
	for (double i = 0; i < 2*hemisphere+unit; i += unit) {
		double c = 1 - i / (2.0f*M_PI);
		if (c > 1) c = 1;
		if (c < -1) c = -1;
		double rad = acos(c);
		double degree = rad * 180 / M_PI;
		cout << fixed << setprecision(6) << "�R�T�C��: "  << c <<"\t���W�A��: " << rad << "\t�x: " << degree << endl;
	}



	/*
	cout << endl << endl;
	init_genrand((unsigned)time(NULL));
	//for (int i = 0; i<100; i++) { printf("%ld\n", genrand_int32()); }
	for (int i = 0; i<100; i++) { printf("%lf\n", genrand_real1()); }


	cout << endl << endl;
	vector3* sampleTest = new vector3;
	random_generator sampleTime((unsigned)time(NULL));
	for (int i = 0; i < 100; i++) {
		randUnitVector(sampleTest, sampleTime);
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
	}


	cout << endl << endl;
	for (int i = 0; i < 100; i++) {
		randUnitVector(sampleTest, sampleTime);
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
		cout << "length: " << sampleTest->x*sampleTest->x + sampleTest->y*sampleTest->y + sampleTest->z*sampleTest->z << endl;;
		float error = sqrt(0.04);
		//error *= genrand_real1();
		sampleTest->x *= error;
		sampleTest->y *= error;
		sampleTest->z *= error;
		cout << "x: " << sampleTest->x << "\ty: " << sampleTest->y << "\tz: " << sampleTest->z << endl;
		cout << "length: " << sampleTest->x*sampleTest->x + sampleTest->y*sampleTest->y + sampleTest->z*sampleTest->z << endl;;
	}
	*/




	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice5(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point5(new pcl::PointCloud<pcl::PointNormal>);
	//for (pcl::PointXYZ point : cloud_lattice2) {
	//
	//}

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_lattice2->begin(); it != cloud_lattice2->end();it++) {
		if (it->x < 6) cloud_lattice5->push_back(*it);
	}
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = attention_point2->begin(); it != attention_point2->end(); it++) {
		if (it->x < 6) attention_point5->push_back(*it);
	}
	*/






































	/*
	for (pcl::PointCloud<pcl::PointXYZ>::iterator iter = cloud_lattice2->begin(); iter != cloud_lattice2->end(); iter++) {
	iter->z = 0.0f;
	}


	testPCA(3, cloud_lattice2);
	cout << endl;

	array<array<double, 3>, 3> holder_PCA = cal_PCA(cloud_lattice2);
	pcl::Normal holder_normal0(holder_PCA[0][0], holder_PCA[0][1], holder_PCA[0][2]);
	cout << "x: " << holder_normal0.normal_x << "\ty: " << holder_normal0.normal_y << "\tz: " << holder_normal0.normal_z << endl;
	pcl::Normal holder_normal1(holder_PCA[1][0], holder_PCA[1][1], holder_PCA[1][2]);
	cout << "x: " << holder_normal1.normal_x << "\ty: " << holder_normal1.normal_y << "\tz: " << holder_normal1.normal_z << endl;
	pcl::Normal holder_normal2(holder_PCA[2][0], holder_PCA[2][1], holder_PCA[2][2]);
	cout << "x: " << holder_normal2.normal_x << "\ty: " << holder_normal2.normal_y << "\tz: " << holder_normal2.normal_z << endl;


	cout << endl;
	cout << "�e�X�g�F " << "test" << endl;
	pm();
	cout << endl;
	cout << endl;


	vector<array<float, 2>> testV;
	testV.reserve(cloud_lattice2->size());
	for (int i = 0; i < cloud_lattice2->size(); i++) {
	array<float, 2> xy;
	xy[0] = cloud_lattice2->at(i).x;
	xy[1] = cloud_lattice2->at(i).y;
	testV.push_back(xy);
	}

	array<array<double, 2>, 2> holder_PCA2 = cal_PCA2(testV);
	cout << "x: " << holder_PCA2[0][0] << "\ty: " << holder_PCA2[0][1] << endl;
	cout << "x: " << holder_PCA2[1][0] << "\ty: " << holder_PCA2[1][1] << endl;
	*/






















	// ���K��
	/*for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result003.txt");
	//for (int i = 0; i < Sn.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn[i] << endl;
	//}
	//outputfileNDH.close();

	vector<pair<float, int>> Sn_high;
	Sn_high.reserve((int)(Sn.size() / 10));
	for (int i = 0; i < Sn.size(); i++) {
	if (Sn[i] > 0.4) {
	Sn_high.push_back(make_pair(Sn[i], i));
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result005.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessPoints(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < Sn_high.size(); i++) {
	uniquenessPoints->push_back(surfaceN->at(Sn_high[i].second));
	}
	*/





	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	/*pcl::PointCloud<pcl::PointNormal>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointNormal>);
	pcl::VoxelGrid<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_ptr);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);

	// �O��l�̏���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
	sor.setInputCloud(cloud_lattice);		// �O��l����������_�Q�����
	sor.setMeanK(50);				// ��������ߖT�_�̐���ݒ�
	sor.setStddevMulThresh(1.0);	// �����̕W���΍��̒l��ݒ�
	sor.setNegative(false);			// �O��l���o�͂���ꍇ��true�ɂ���
	sor.filter(*cloud_filtered);	// �o��

	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_lattice;*/

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud = cloud_ptr;

	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = cloud_ptr;



	/*
	// STL�̖@�������𗘗p���������_�̎������o
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = cloud_ptr;
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>);

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 1.0f;

	kdtree.setInputCloud(surfaceN);

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	pcl::PointNormal searchPoint = *it;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	double angle_max = 0;
	pcl::PointXYZ pN;
	pcl::PointXYZ qN;
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	for (int j = i; j < pointIdxRadiusSearch.size(); j++) {
	pcl::PointXYZ p(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	pcl::PointXYZ q(surfaceN->points[pointIdxRadiusSearch[j]].normal_x, surfaceN->points[pointIdxRadiusSearch[j]].normal_y, surfaceN->points[pointIdxRadiusSearch[j]].normal_z);
	double angle = cal_angle(p, q);
	if (angle < angle_max) continue;
	angle_max = angle;
	pN = p;
	qN = q;
	}
	}
	if (angle_max < 60) continue;
	//pcl::PointXYZ crossN = cross_product3_cal(pN, qN);
	//double angle_min = 180;
	//for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	//	pcl::PointXYZ p(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	//	double angle = cal_angle(p, crossN);
	//	angle = min(angle, 180 - angle); //
	//	if (angle > angle_min) continue;
	//	angle_min = angle;
	//}
	//if (angle_min > 60) continue;
	keypoints->push_back(searchPoint);
	}
	}
	*/


	/*double RV_x = 1.0;
	double RV_z = 0.0;

	array<float, 36> angleH;
	for (int j = 0; j < angleH.size(); j++)
	angleH[j] = 0;

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_lattice->begin(); it != cloud_lattice->end(); it++) {
	double angle_cos = (RV_x*it->normal_x) + (RV_z*it->normal_z);
	double angle_sin = (RV_x*it->normal_z) - (RV_z*it->normal_x);
	double radian = acos(angle_cos);
	double angle = radian * 180 / M_PI;
	//if (angle_sin < 0) angle += 180;
	if (angle_sin < 0) angle = 360 - angle;
	angle /= 10;
	int indexH = (int)angle;
	angleH[indexH]++;
	}

	for (int j = 0; j < angleH.size(); j++)
	cout << "angleH[" << j << "]: " << angleH[j] << endl;
	*/



	/*
	// �_�Q�̊i�q���i�_�E���T���v�����O�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud_lattice);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lattice = voxel_grid(cloud_in);



	// �O��l�̏���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_lattice, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Remove_outliers(cloud_in, false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_in;
	*/


	// ����f�[�^�ǂݍ��݁i�z���_�[�ʁj�i�@���x�N�g�����H��p���Ɠ����ɂȂ�O�p�`�|���S���̒��o�j
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);

	if (angle_cos < theta_cos) { // �p�x���傫�����cos�Ƃ͏������Ȃ�
	continue;
	}

	pcl::PointXYZ point;
	for (int j = 0; j < 3; j++) {
	point.x = tri.vertex[j].x;
	point.y = tri.vertex[j].y;
	point.z = tri.vertex[j].z;
	//////// �z���_�[�� ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;*/


	// ����f�[�^�ǂݍ��݁i�z���_�[�ʁj�i�@���x�N�g�����H��p���Ɠ����ɂȂ�O�p�`�|���S���̒��o�j
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr holder_face(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<stl_object.getDatanum(); i++) {
	TRIANGLE tri;
	stl_object.getData(i, &tri);
	float dot_product = (tri.normal.nx * tool_normal.normal_x) + (tri.normal.ny * tool_normal.normal_y) + (tri.normal.nz * tool_normal.normal_z);
	float triN_norm = sqrt((tri.normal.nx * tri.normal.nx) + (tri.normal.ny * tri.normal.ny) + (tri.normal.nz * tri.normal.nz));
	double angle_cos = dot_product / (triN_norm*toolN_norm);
	if (angle_cos < theta_cos) { // �p�x���傫�����cos�Ƃ͏������Ȃ�
	continue;
	}
	pcl::PointXYZ point;
	point.x = tri.vertex[0].x;
	point.y = tri.vertex[0].y;
	point.z = tri.vertex[0].z;
	//////// �z���_�[�� ////////
	if (point.y > -5) continue;
	if (point.y < -6) continue;
	////////////////////////////
	holder_face->points.push_back(point);
	}
	//cout << "holder_face->size(): " << holder_face->size() << endl;



	// �z���_�[�ʂ̌��o�iRANSAC�j
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_wall(holder_face, false);*/


	// �z���_�[�ʂ̌��o�i�ʎq���ɂ��Y�l��r�j
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y�l�̗ʎq��
	int offset_i = 27*10;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = holder_face->begin(); it != holder_face->end(); it++) {
	float y_i = it->y;
	y_i *= 10;
	y_i = round(y_i);
	int index = y_i + offset_i;
	sample_vec0[index].push_back(*it);
	}
	int max = -1;
	int max_i = 0;
	for (int i = 0; i < 21; i++) {
	int size = sample_vec0[i].size();
	//cout << "sample_vec[" << i << "].size(): " << size << endl;
	if (size > max) {
	max = size;
	max_i = i;
	}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = sample_vec0[max_i].makeShared();
	//cout << "max_i: " << max_i << endl;*/


	// �z���_�[�ʂ̌��o�iY�l�ɂ��O���[�s���O�j �� ������
	/*pcl::PointCloud<pcl::PointXYZ> sample_vec0[21]; // y�l�̗ʎq��
	int offset_i = 27 * 10;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = holder_face->begin(); it != holder_face->end(); it++) {
	float y_i = it->y;
	y_i *= 10;
	y_i = round(y_i);
	int index = y_i + offset_i;
	sample_vec0[index].push_back(*it);
	}
	int max = -1;
	int max_i = 0;
	for (int i = 0; i < 21; i++) {
	int size = sample_vec0[i].size();
	if (size > max) {
	max = size;
	max_i = i;
	}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = sample_vec0[max_i].makeShared();
	//cout << "max_i: " << max_i << endl;*/



	// �z���_�[�ʂ̌���i�ŏ����@�j
	/*array<array<double, 3>, 3> holder_PCA = cal_PCA(sample_vec);
	pcl::Normal holder_normal(holder_PCA[0][0], holder_PCA[0][1], holder_PCA[0][2]);
	//cout << "x: " << holder_normal.normal_x << "\ty: " << holder_normal.normal_y << "\tz: " << holder_normal.normal_z << endl;



	// �H���[�_�̒��o
	priority_queue <pcl::PointXYZ, vector<pcl::PointXYZ>, PointYCompareMin> minpq;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++) {
	minpq.push(*it);
	}
	//cout << "minpq" << endl;
	pcl::PointXYZ min_value = minpq.top();
	//cout << "x: " << min_value.x << "\ty: " << min_value.y << "\tz: " << min_value.z << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tip_point(new pcl::PointCloud<pcl::PointXYZ>);
	tip_point->push_back(min_value);



	// �_�Ɩʂ̋����Z�o
	pcl::PointXYZ A = sample_vec->at(0);
	float coef_d = -(holder_normal.normal_x*A.x + holder_normal.normal_y*A.y + holder_normal.normal_z*A.z);
	float error_d = -(holder_normal.normal_x*min_value.x + holder_normal.normal_y*min_value.y + holder_normal.normal_z*min_value.z);
	float distance = fabsf(coef_d - error_d);
	cout << "distance: " << distance << endl;*/


	/*
	// �ȗ�
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr keypointsCurvature(new pcl::PointCloud<pcl::PointNormal>());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	if (it->curvature < 0.01) continue; //0.05
	//cout << "it->curvature: " <<it->curvature << endl;
	keypointsCurvature->push_back(*it);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	cout << "keypointsCurvature->points.size(): " << keypointsCurvature->points.size() << endl;


	pcl::PointCloud<pcl::PointNormal>::Ptr attention_point(new pcl::PointCloud<pcl::PointNormal>);

	priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending> cQueue;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = keypointsCurvature->begin(); it != keypointsCurvature->end(); it++) {
	cQueue.push(*it);
	}

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	kdtree.setInputCloud(keypointsCurvature);
	pcl::PointNormal searchPoint = cQueue.top();
	cout << "searchPoint.curvature: " << searchPoint.curvature << endl;

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	keypointsCurvature->erase(keypointsCurvature->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	attention_point->push_back(searchPoint);

	while (!keypointsCurvature->empty()) {
	cQueue = priority_queue <pcl::PointNormal, vector<pcl::PointNormal>, CurvatureCompareAscending>();
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = keypointsCurvature->begin(); it != keypointsCurvature->end(); it++) {
	cQueue.push(*it);
	}
	kdtree.setInputCloud(keypointsCurvature);
	searchPoint = cQueue.top();
	cout << "searchPoint.curvature: " << searchPoint.curvature << endl;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	keypointsCurvature->erase(keypointsCurvature->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "keypointsCurvature->size(): " << keypointsCurvature->size() << endl;
	attention_point->push_back(searchPoint);
	}


	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(cloud, false);

	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	*/



	/*
	// RANSAC�ɂ��`�󕪗�
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN2 = surface_normals(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN(new pcl::PointCloud<pcl::PointNormal>());
	surfaceN->reserve(surfaceN2->size());
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN2->begin(); it != surfaceN2->end(); it++) {
	if (isnan(it->normal_x)) continue;
	surfaceN->push_back(*it);
	}
	*/

	/*ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result001.txt");
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	outputfileNDH << "it->x: " << it->x << endl;
	outputfileNDH << "it->y: " << it->y << endl;
	outputfileNDH << "it->z: " << it->z << endl;
	outputfileNDH << "it->normal_x: " << it->normal_x << endl;
	outputfileNDH << "it->normal_y: " << it->normal_y << endl;
	outputfileNDH << "it->normal_z: " << it->normal_z << endl;
	}
	outputfileNDH.close();*/




	/*
	// �Ǐ��`��̓Ǝ����ɒ��ڂ������̔F���ɗL����3-D�����_�̎������o
	//pcl::PointCloud<pcl::PointNormal>::Ptr surfaceN = surface_normals(cloud);
	std::vector<array<float, 18>> NDH;
	NDH.reserve(surfaceN->size());

	// Neighbors within radius search
	pcl::search::KdTree<pcl::PointNormal> kdtree;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 3.0f;

	kdtree.setInputCloud(surfaceN);

	for (pcl::PointCloud<pcl::PointNormal>::iterator it = surfaceN->begin(); it != surfaceN->end(); it++) {
	pcl::PointNormal searchPoint = *it;
	array<float, 18> angleH;
	for (int j = 0; j < angleH.size(); j++)
	angleH[j] = 0;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
	pcl::PointXYZ p(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z);
	pcl::PointXYZ q(surfaceN->points[pointIdxRadiusSearch[i]].normal_x, surfaceN->points[pointIdxRadiusSearch[i]].normal_y, surfaceN->points[pointIdxRadiusSearch[i]].normal_z);
	double angle = cal_angle(p, q);
	//if (isnan(angle)) continue; //
	angle /= 10;
	int indexH = (int)angle;
	//cout << "[" << i << "]:\t" << "angle: " << angle << "\tindexH: " << indexH << endl;
	angleH[indexH]++;
	}
	}
	NDH.push_back(angleH);
	}

	// ���K��
	for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	int countNum = 0;
	for (int i = 0; i < it->size(); i++) {
	countNum += it->at(i);
	}
	for (int i = 0; i < it->size(); i++) {
	it->at(i) /= countNum;
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result002.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result002.txt");
	//for (std::vector<array<float, 18>>::iterator it = NDH.begin(); it != NDH.end(); it++) {
	//for (int i = 0; i < it->size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << it->at(i) << endl;
	//}
	//outputfileNDH << endl;
	//}
	//outputfileNDH.close();


	vector<float> Sn = all_uniqueness(NDH);

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result003.txt");
	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\MVX2000X2F25_result003.txt");
	//for (int i = 0; i < Sn.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn[i] << endl;
	//}
	//outputfileNDH.close();

	vector<pair<float, int>> Sn_high;
	Sn_high.reserve((int)(Sn.size() / 10));
	for (int i = 0; i < Sn.size(); i++) {
	if (Sn[i] > 0.4) {
	Sn_high.push_back(make_pair(Sn[i], i));
	}
	}

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result004.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	//sort(Sn_high.begin(), Sn_high.end(), greater<pair<float, int>>());

	//ofstream outputfileNDH("C:\\Users\\yuki\\Desktop\\yuki\\�������֘A\\�֍u�E�ł����킹\\M2\\�ł����킹\\02\\outputfile\\NDH_result005.txt");
	//for (int i = 0; i < Sn_high.size(); i++) {
	//outputfileNDH << "[" << i << "]:\t" << Sn_high[i].first << endl;
	//}
	//outputfileNDH.close();


	pcl::PointCloud<pcl::PointNormal>::Ptr uniquenessPoints(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < Sn_high.size(); i++) {
	uniquenessPoints->push_back(surfaceN->at(Sn_high[i].second));
	}





	//pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointNormal>::Ptr sample_vec = Detect_Cylinder(surfaceN);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_vec2 = Detect_wall(cloud, false);

	cout << "cylinder: " << sample_vec->size() << endl;
	cout << "plane: " << sample_vec2->size() << endl;
	cout << "surfaceN: " << surfaceN->size() << endl;
	cout << "cloud: " << cloud->size() << endl;
	*/






	// �����_����
	/*pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	//detector.setNonMaxSupression(false);
	detector.setRadius(3.0);//3.0 5.0 7.0
	detector.setInputCloud(cloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);

	//std::cout << "keypoints detected: " << keypoints->size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ tmp;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++) {
	tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
	//if ((*i).intensity>max) {
	//std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
	//max = (*i).intensity;
	//}
	//if ((*i).intensity<min) {
	//min = (*i).intensity;
	//}
	keypoints3D->push_back(tmp);
	}
	//std::cout << "maximal responce: " << max << " min responce:  " << min << std::endl;



	// �����_�̖@������
	// average �H��S�̂̏d�S
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointXYZ cloud_average = cal_cloud_average(cloud);
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/
	// average �����_�ߖT�̏d�S
	/*pcl::PointCloud<pcl::PointNormal>::Ptr keypoint_normals(new pcl::PointCloud<pcl::PointNormal>());
	for (int i = 0; i < keypoints3D->size(); i++) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood_cloud = neighborhood_cal(cloud, keypoints3D->at(i));
	pcl::PointXYZ cloud_average = cal_cloud_average(neighborhood_cloud);
	pcl::PointNormal normal = normal_direction(neighborhood_cloud, keypoints3D->at(i), cloud_average);
	keypoint_normals->push_back(normal);
	}*/



	// ����f�[�^��PPF
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(keypoint_normals);
	//vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs2(keypoint_normals);
	//outputfile777 << "keypoint_cloud->size(): " << keypoint_cloud->size() << endl;
	//outputfile777 << "keypoint_PPF.size(): " << keypoint_PPF.size() << endl;*/
	/*vector<pair<myPPF, pair<pcl::PointXYZ, pcl::PointXYZ>>> keypoint_PPF = make_PPFs(attention_point);



	// �H��f�[�^�ǂݍ��݁i�f�[�^�}�b�`���O�j
	string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative0.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative2.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative3.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative4.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative5.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative6.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative7.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative8.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\CuttingTool_Alternative9.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DCLNR3225P12.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR1616H11.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DDJNR2020K15.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\DVJNR2020K16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M16N.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\MTJNR2525M22N.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PVJNR2020K16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\PWLNR2525M06.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SDNCR0808K07-SM.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SRGCR1616H08.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SSSCR1212F09.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVPPR1616M11-SM.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SVVCN1616H16.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\SXZCR2020K15.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\bite\\TLHR2020K5.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CL2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55CR2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QL2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2020-K17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\DTR55QR2525-M17.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-600W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDEL2525-800W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-600W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800S.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\GDER2525-800W.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT35R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT55R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\PBT80R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SBT35R2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SFTR2020.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SGWR1616.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0707-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0808-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBL0909-70.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR0808-60.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1010-60.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\sumitomo\\SPBR1212-60.STL";
	data_matching(path, keypoint_PPF);

	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2000S25.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA2500S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRA3000S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2000S25.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS2500S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\BRS3000S32.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2500X2F25 - MVX2500X2F25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX3000X2F32 - MVX3000X2F32-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2000S25 - TAWSN2000S25-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN2500S32 - TAWSN2500S32-1.STL";
	data_matching(path, keypoint_PPF);
	path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\TAWSN3000S32 - TAWSN3000S32-1.STL";
	data_matching(path, keypoint_PPF);*/



	//string path = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\3D models\\Turning tool database\\STL files\\NikonTurningTool.STL";
	//string path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	//pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud = stl_points(path);


	//string path = "C:\\Users\\yuki\\Desktop\\yuki\\database\\mitsubishi\\drill\\MVX2000X2F25 - MVX2000X2F25-1.STL";
	/*
	// �_���Ԉ�������
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud2 = stl_points(path);
	// Neighbors within radius search
	while (!stl_cloud2->empty()) {
	kdtree.setInputCloud(stl_cloud2);
	searchPoint = stl_cloud2->at(0);
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
	sort(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
	for (int i = pointIdxRadiusSearch.size() - 1; 0 <= i; i--)
	stl_cloud2->erase(stl_cloud2->begin() + pointIdxRadiusSearch[i]);
	}
	cout << "stl_cloud2->size(): " << stl_cloud2->size() << endl;
	stl_cloud->push_back(searchPoint);
	}
	*/

	/*
	// ���ʍ������
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stl_cloud2 = stl_points(path);
	// Neighbors within radius search
	kdtree.setInputCloud(stl_cloud2);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = stl_cloud2->begin(); it != stl_cloud2->end(); it++) {
	searchPoint = *it;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) < 10) {
	stl_cloud->push_back(searchPoint);
	}
	}
	*/



	// �_�Q�f�[�^
	/*start = std::chrono::system_clock::now(); // �v���J�n����
	const string filename = "C:\\Users\\yuki\\Desktop\\Alexandre Raguenes\\Point cloud files\\Turning tool in tool holder\\point_cloud_External_Turning_Tool_Moved.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = myifstream_test(filename);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = voxel_grid(cloud_in);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*cloud);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = myifstream_test(filename);
	outputfile777 << "cloud_in->size(): " << cloud_in->size() << endl;
	outputfile777 << endl;
	end = std::chrono::system_clock::now();  // �v���I������
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); //�����ɗv�������Ԃ��~���b�ɕϊ�
	outputfile << "input_cloud: " << elapsed << "[ms]" << endl;
	outputfile << endl;*/


	////////////////////////////////////////////

	//outputfile777.close();
	outputfile.close();


	// ���͓_�Q�Ɩ@����\��
	//viewPointCloud(cloud_lattice2, attention_point2);

	return (0);
}