# Locate the include paths and libraries for the UUID libraries. On
# Windows this locates the Rpcrt4 library
#
# UUID_FOUND - was libuuid (Linux or OSX) or Rpcrt4 (Windows) found
# UUID_INCLUDE_DIRS - path to the UUID include files. On Windows this variable
# is left empty, but you should still add it to your CMakeLists.txt to ensure
# portability
# UUID_LIBRARIES - full path to the libraries
if(WIN32)
  find_library(UUID_LIBRARIES NAMES Rpcrt4 PATH)

  if(UUID_LIBRARIES)
    set(UUID_FOUND true)
  endif(UUID_LIBRARIES)

else()
  find_path(UUID_INCLUDE_DIRS uuid/uuid.h)
  find_library(UUID_LIBRARIES NAMES uuid PATH)

  if(UUID_INCLUDE_DIRS)
    set(UUID_FOUND true)
  endif(UUID_INCLUDE_DIRS)

  if(NOT UUID_LIBRARIES)
    set(UUID_LIBRARIES "")
  endif(NOT UUID_LIBRARIES)

endif(WIN32)

if(NOT UUID_FOUND)
  if(UUID_FIND_REQUIRED)
    if(WIN32)
      message(FATAL_ERROR "Could not find Rpcrt4")
    else()
      message(FATAL_ERROR "Could not find UUID")
    endif(WIN32)
  endif(UUID_FIND_REQUIRED)
endif(NOT UUID_FOUND)
