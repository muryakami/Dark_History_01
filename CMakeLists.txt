cmake_minimum_required( VERSION 2.8 )



# Create Project

project( solution )

add_executable( project
 sorce/main.cpp
 sorce/basicCalculation.hpp
 sorce/cal_PCA.hpp
 sorce/gaussianElimination.hpp
 sorce/jacobi.hpp
 sorce/least_squares_methods.hpp
 sorce/main.h
 sorce/MT.h
 sorce/pointSearch.hpp
 sorce/preprocessingSTL.hpp
 sorce/principal.hpp
 sorce/random_generator.cpp
 sorce/random_generator.h
 sorce/randUnitVector.cpp
 sorce/randUnitVector.h
 sorce/readPointCloud.hpp
 sorce/readSTL.hpp
 sorce/run_Jacobi.hpp
 sorce/surface_normals.hpp
 sorce/voxel_grid.hpp)



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