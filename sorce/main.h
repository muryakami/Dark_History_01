#ifndef _MAIN_H_
#define _MAIN_H_


#include "ReadSTL.hpp"
#include "PreprocessingSTL.hpp"
#include "PointSearch.hpp"
#include "ReadPointCloud.hpp"
#include "VoxelGrid.hpp"
#include "SurfaceNormals.hpp"
#include "NormalDirection.hpp"
#include "BasicCalculation.hpp"
#include "LeastSquaresMethods.hpp"
#include "CalPCA.hpp"
#include "InterpolationSTL.hpp"

#include "PPF.hpp"
#include "KeypointNormals.hpp"

#include "FeaturePointExtractionHarris.hpp"
#include "Detectors.hpp"
#include "FeatureDescription.hpp"
#include "CorrespondenceGrouping.hpp"

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

#include "Output.h"


#endif // _MAIN_H_
