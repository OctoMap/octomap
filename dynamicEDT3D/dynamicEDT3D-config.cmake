# - Config file for the OctoMap package
# (example from http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file)
# It defines the following variables
#  OCTOMAP_INCLUDE_DIRS - include directories for OctoMap
#  OCTOMAP_LIBRARIES    - libraries to link against

set(DYNAMICEDT3D_INCLUDE_DIRS)
set(DYNAMICEDT3D_INCLUDE DYNAMICEDT3D_INCLUDE-NOTFOUND)
find_path(DYNAMICEDT3D_INCLUDE dynamicEDT3D/dynamicEDT3D.h)
if(DYNAMICEDT3D_INCLUDE)
  list(APPEND DYNAMICEDT3D_INCLUDE_DIRS ${DYNAMICEDT3D_INCLUDE})
endif()

set(DYNAMICEDT3D_LIBRARIES)
set(DYNAMICEDT3D_LIB DYNAMICEDT3D_LIB-NOTFOUND)
find_library(DYNAMICEDT3D_LIB dynamicedt3d)
if(DYNAMICEDT3D_LIB)
  list(APPEND DYNAMICEDT3D_LIBRARIES ${DYNAMICEDT3D_LIB})
endif()
