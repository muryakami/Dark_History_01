cmake_minimum_required( VERSION 2.8 )



# Create Project

project( solution )

add_executable( project
 sorce/main.cpp
 sorce/main.h
 sorce/BasicCalculation.hpp
 sorce/CalPCA.hpp
 sorce/CorrespondenceGrouping.hpp
 sorce/DataMatching.hpp
 sorce/Detectors.hpp
 sorce/Errata.hpp
 sorce/FeatureDescription.hpp
 sorce/FeaturePointExtraction36.hpp
 sorce/FeaturePointExtractionHarris.hpp
 sorce/GaussianElimination.hpp
 sorce/InterpolationSTL.hpp
 sorce/Jacobi.hpp
 sorce/KeypointNormals.hpp
 sorce/LeastSquaresMethods.hpp
 sorce/MT.h
 sorce/NormalDirection.hpp
 sorce/Output.h
 sorce/PointSearch.hpp
 sorce/PPF.hpp
 sorce/PPFio.hpp
 sorce/PreprocessingSTL.hpp
 sorce/Principal.hpp
 sorce/RandomGenerator.cpp
 sorce/RandomGenerator.h
 sorce/RandUnitVector.cpp
 sorce/RandUnitVector.h
 sorce/RANSAC.hpp
 sorce/ReadPointCloud.hpp
 sorce/ReadSTL.hpp
 sorce/RemoveOutliers.hpp
 sorce/RunJacobi.hpp
 sorce/SurfaceNormals.hpp
 sorce/VoxelGrid.hpp)



# Set StartUp Project (Option)

# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )



# Find Packages

find_package( PCL 1.8.1 REQUIRED )



if( PCL_FOUND )

  # [C/C++]>[General]>[Additional Include Directories]

  include_directories( ${PCL_INCLUDE_DIRS} )



  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]

  add_definitions( ${PCL_DEFINITIONS} )


  
# For Use Not PreCompiled Features
 
 add_definitions( -DPCL_NO_PRECOMPILE )



  # [Linker]>[General]>[Additional Library Directories]

  link_directories( ${PCL_LIBRARY_DIRS} )



  # [Linker]>[Input]>[Additional Dependencies]

  target_link_libraries( project ${PCL_LIBRARIES} )

endif()