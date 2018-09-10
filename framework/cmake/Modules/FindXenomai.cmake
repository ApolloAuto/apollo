# (C) Copyright 2005-2014 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
# 
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
# 
# --- end cisst license ---
#
# CMake script for finding Xenomai
#
# This will find Xenomai on Linux systems and define flags for each of the
# supported Xenomai "skins". The current supported skins are:
#
# - NATIVE
# - POSIX
# 
# Input variables:
# 
# - ${Xenomai_ROOT_DIR} (optional): Used as a hint to find the Xenomai root dir
# - $ENV{XENOMAI_ROOT_DIR} (optional): Used as a hint to find the Xenomai root dir
#
# Cache variables:
#
# - Xenomai_ROOT_DIR
# - Xenomai_INCLUDE_DIRS
#
# Output Variables:
#
# - Xenomai_FOUND: Boolean that indicates if the package was found. See the
#   Xenomai_*_FOUND variables below for individual skins.
# - Xenomai_VERSION: major.minor.patch Xenomai version string
# - Xenomai_XENO_CONFIG: Path to xeno-config program
#
# - Individual library variables:
#   - Xenomai_LIBRARY_XENOMAI
#   - Xenomai_LIBRARY_NATIVE
#   - Xenomai_LIBRARY_PTHREAD_RT
#   - Xenomai_LIBRARY_RTDM
#   - Xenomai_LIBRARY_RTDK ( this will be empty, deprecated after Xenomai 2.6.0)
#
# - Native Flags:
#   - Xenomai_NATIVE_FOUND: Boolean that indicates if the native skin was found
#   - Xenomai_NATIVE_DEFINITIONS
#   - Xenomai_NATIVE_INCLUDE_DIRS
#   - Xenomai_NATIVE_LIBRARY_DIRS
#   - Xenomai_NATIVE_LIBRARIES
#   - Xenomai_NATIVE_LDFLAGS
# 
# - POSIX Flags:
#   - Xenomai_POSIX_FOUND: Boolean that indicates if the posix skin was found
#   - Xenomai_POSIX_DEFINITIONS
#   - Xenomai_POSIX_INCLUDE_DIRS
#   - Xenomai_POSIX_LIBRARY_DIRS
#   - Xenomai_POSIX_LIBRARIES
#   - Xenomai_POSIX_LDFLAGS
#
# - RTDM Flags:
#   - Xenomai_RTDM_FOUND: Boolean that indicates if the RTDM skin was found
#   - Xenomai_RTDM_DEFINITIONS
#   - Xenomai_RTDM_INCLUDE_DIRS
#   - Xenomai_RTDM_LIBRARY_DIRS
#   - Xenomai_RTDM_LIBRARIES
#   - Xenomai_RTDM_LDFLAGS

if( UNIX )

  # Get hint from environment variable (if any)
  if(NOT $ENV{XENOMAI_ROOT_DIR} STREQUAL "")
    set(XENOMAI_ROOT_DIR $ENV{XENOMAI_ROOT_DIR} CACHE PATH "Xenomai base directory location (optional, used for nonstandard installation paths)" FORCE)
    mark_as_advanced(XENOMAI_ROOT_DIR)
  endif()

  # set the search paths
  set( Xenomai_SEARCH_PATH /usr/local /usr $ENV{XENOMAI_ROOT_DIR} ${Xenomai_ROOT_DIR})
  
  # find xeno_config.h
  find_path( Xenomai_INCLUDE_DIR
    xeno_config.h 
    PATHS ${Xenomai_SEARCH_PATH} 
    PATH_SUFFIXES xenomai include xenomai/include include/xenomai
    )

  # did we find xeno_config.h?
  if(Xenomai_INCLUDE_DIR) 
    MESSAGE(STATUS "xenomai found: \"${Xenomai_INCLUDE_DIR}\"")
    
    # set the root directory
    if( "${Xenomai_INCLUDE_DIR}" MATCHES "/usr/include/xenomai" )
      # on ubuntu linux, xenomai install is not rooted to a single dir
      set( Xenomai_ROOT_DIR /usr CACHE PATH "The Xenomai FHS root")
      set( Xenomai_INCLUDE_POSIX_DIR ${Xenomai_INCLUDE_DIR}/posix )
    else()
      # elsewhere, xenomai install is packaged
      get_filename_component(Xenomai_ROOT_DIR ${Xenomai_INCLUDE_DIR} PATH CACHE)
      set( Xenomai_INCLUDE_POSIX_DIR ${Xenomai_ROOT_DIR}/include/posix )
    endif()

    # Find xeno-config
    find_program(Xenomai_XENO_CONFIG NAMES xeno-config  PATHS ${Xenomai_ROOT_DIR}/bin NO_DEFAULT_PATH)

    # get xenomai version
    execute_process(COMMAND ${Xenomai_XENO_CONFIG} --version OUTPUT_VARIABLE Xenomai_VERSION)
    
    # find the xenomai pthread library
    find_library( Xenomai_LIBRARY_NATIVE     native     ${Xenomai_ROOT_DIR}/lib )
    find_library( Xenomai_LIBRARY_XENOMAI    xenomai    ${Xenomai_ROOT_DIR}/lib )
    find_library( Xenomai_LIBRARY_PTHREAD_RT pthread_rt ${Xenomai_ROOT_DIR}/lib )
    find_library( Xenomai_LIBRARY_RTDM       rtdm       ${Xenomai_ROOT_DIR}/lib )

    # In 2.6.0 RTDK was merged into the main xenomai library
    if(Xenomai_VERSION VERSION_GREATER 2.6.0)
      set(Xenomai_LIBRARY_RTDK_FOUND ${Xenomai_LIBRARY_XENOMAI_FOUND})
      set(Xenomai_LIBRARY_RTDK ${Xenomai_LIBRARY_XENOMAI})
    else()
      find_library( Xenomai_LIBRARY_RTDK rtdk ${Xenomai_ROOT_DIR}/lib )
    endif()

    # Xenomai libraries for each skin
    set(Xenomai_NATIVE_LIBRARIES ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_XENOMAI} pthread)
    set(Xenomai_POSIX_LIBRARIES ${Xenomai_LIBRARY_PTHREAD_RT} ${Xenomai_LIBRARY_XENOMAI} pthread rt)
    set(Xenomai_RTDM_LIBRARIES ${Xenomai_LIBRARY_RTDM} ${Xenomai_LIBRARY_XENOMAI} pthread rt)

    # Xenomai LDFLAGS for each skin
    set(Xenomai_NATIVE_LDFLAGS "")
    set(Xenomai_POSIX_LDFLAGS "-Wl,@${Xenomai_ROOT_DIR}/lib/posix.wrappers")
    set(Xenomai_RTDM_LDFLAGS "")

    # Xenomai compiler definitions for each supported skin
    set(Xenomai_NATIVE_DEFINITIONS -D_GNU_SOURCE -D_REENTRANT -D__XENO__)
    set(Xenomai_POSIX_DEFINITIONS ${Xenomai_NATIVE_DEFINITIONS})
    set(Xenomai_RTDM_DEFINITIONS ${Xenomai_NATIVE_DEFINITIONS})

    # Xenomai library dirs for each skin
    set( Xenomai_NATIVE_LIBRARY_DIRS ${Xenomai_ROOT_DIR}/lib )
    set( Xenomai_POSIX_LIBRARY_DIRS ${Xenomai_NATIVE_LIBRARY_DIRS} )
    set( Xenomai_RTDM_LIBRARY_DIRS ${Xenomai_NATIVE_LIBRARY_DIRS} )

    # Xenomai library dirs for each skin
    set( Xenomai_NATIVE_INCLUDE_DIRS ${Xenomai_INCLUDE_DIR} )
    set( Xenomai_POSIX_INCLUDE_DIRS ${Xenomai_INCLUDE_DIR} ${Xenomai_INCLUDE_POSIX_DIR} )
    set( Xenomai_RTDM_INCLUDE_DIRS ${Xenomai_INCLUDE_DIR} )

    # Compatibility
    set( Xenomai_LIBRARIES 
      ${Xenomai_LIBRARY_XENOMAI}
      ${Xenomai_LIBRARY_NATIVE}
      ${Xenomai_LIBRARY_PTHREAD_RT}
      ${Xenomai_LIBRARY_RTDM}
      ${Xenomai_LIBRARY_RTDK}
      )

  else( )
    MESSAGE(STATUS "Xenomai NOT found in paths: (${Xenomai_SEARCH_PATH})")
  endif( )

endif( UNIX )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Xenomai DEFAULT_MSG
  Xenomai_ROOT_DIR
  Xenomai_INCLUDE_DIR
  Xenomai_LIBRARY_XENOMAI
  Xenomai_LIBRARY_RTDK
  )
  
set(Xenomai_FOUND ${XENOMAI_FOUND}) # Set appropriately cased variable

if(Xenomai_LIBRARY_XENOMAI AND Xenomai_LIBRARY_NATIVE) 
  message(STATUS "Xenomai Native skin found")
  set(Xenomai_NATIVE_FOUND True)
endif()
  
if(Xenomai_LIBRARY_XENOMAI AND Xenomai_LIBRARY_PTHREAD_RT) 
  message(STATUS "Xenomai POSIX skin found")
  set(Xenomai_POSIX_FOUND True)
endif()
  
if(Xenomai_LIBRARY_XENOMAI AND Xenomai_LIBRARY_RTDM) 
  message(STATUS "Xenomai RTDM skin found")
  set(Xenomai_RTDM_FOUND True)
endif()
  
