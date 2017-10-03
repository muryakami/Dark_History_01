#ifndef _MAIN_H_
#define _MAIN_H_


#include "readSTL.hpp"
#include "preprocessingSTL.hpp"
#include "pointSearch.hpp"
#include "readPointCloud.hpp"
#include "voxel_grid.hpp"
#include "surface_normals.hpp"
#include "basicCalculation.hpp"
#include "least_squares_methods.hpp"
#include "cal_PCA.hpp"

#include <random>
#include <queue>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/integral_image_normal.h>


ofstream accuracy_file("C:\\Users\\yuki\\Documents\\êiíªïÒçê\\03\\outputfile\\accuracy001.txt");
ofstream outputfile777("C:\\Users\\yuki\\Documents\\êiíªïÒçê\\03\\outputfile\\result001.txt");



#endif // !_MAIN_H_
