###############################################################################
# 
# CMake script for finding the Eigen library.
# 
# http://eigen.tuxfamily.org/index.php?title=Main_Page
# 
# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Copyright (c) 2008, 2009 Gael Guennebaud, <g.gael@free.fr>
# Copyright (c) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
# Redistribution and use is allowed according to the terms of the 2-clause BSD
# license.
# 
# 
# Input variables:
# 
# - Eigen_ROOT_DIR (optional): When specified, header files and libraries
#   will be searched for in `${Eigen_ROOT_DIR}/include` and
#   `${Eigen_ROOT_DIR}/libs` respectively, and the default CMake search order
#   will be ignored. When unspecified, the default CMake search order is used.
#   This variable can be specified either as a CMake or environment variable.
#   If both are set, preference is given to the CMake variable.
#   Use this variable for finding packages installed in a nonstandard location,
#   or for enforcing that one of multiple package installations is picked up.
# 
# Cache variables (not intended to be used in CMakeLists.txt files)
# 
# - Eigen_INCLUDE_DIR: Absolute path to package headers.
# 
# 
# Output variables:
# 
# - Eigen_FOUND: Boolean that indicates if the package was found
# - Eigen_INCLUDE_DIRS: Paths to the necessary header files
# - Eigen_VERSION: Version of Eigen library found
# - Eigen_DEFINITIONS: Definitions to be passed on behalf of eigen
# 
# 
# Example usage:
# 
#   # Passing the version means Eigen_FOUND will only be TRUE if a
#   # version >= the provided version is found.
#   find_package(Eigen 3.1.2)
#   if(NOT Eigen_FOUND)
#     # Error handling
#   endif()
#   ...
#   add_definitions(${Eigen_DEFINITIONS})
#   ...
#   include_directories(${Eigen_INCLUDE_DIRS} ...)
# 
###############################################################################

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN eigen3)
set(EIGEN_DEFINITIONS ${PC_EIGEN_CFLAGS_OTHER})


find_path(EIGEN_INCLUDE_DIR Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS}
          "${Eigen_ROOT_DIR}" "$ENV{EIGEN_ROOT_DIR}"
          "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"  # Backwards Compatibility
    PATHS "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"
          "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
    PATH_SUFFIXES eigen3 include/eigen3 include)

set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)

mark_as_advanced(EIGEN_INCLUDE_DIR)

if(EIGEN_FOUND)
  message(STATUS "Eigen found (include: ${EIGEN_INCLUDE_DIRS})")
endif(EIGEN_FOUND)


set(Eigen_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
set(Eigen_FOUND ${EIGEN_FOUND})
set(Eigen_VERSION ${EIGEN_VERSION})
set(Eigen_DEFINITIONS ${EIGEN_DEFINITIONS})
