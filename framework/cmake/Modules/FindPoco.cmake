# - Find the Poco includes and libraries.
# The following variables are set if Poco is found.  If Poco is not
# found, Poco_FOUND is set to false.
#  Poco_FOUND        - True when the Poco include directory is found.
#  Poco_INCLUDE_DIRS - the path to where the poco include files are.
#  Poco_LIBRARY_DIR - The path to where the poco library files are.
#  Poco_BINARY_DIRS - The path to where the poco dlls are.
#  Poco_LIBRARIES - list of all libs from requested components.

# ----------------------------------------------------------------------------
# If you have installed Poco in a non-standard location.
# Then you have three options.
# In the following comments, it is assumed that <Your Path>
# points to the root directory of the include directory of Poco. e.g
# If you have put poco in C:\development\Poco then <Your Path> is
# "C:/development/Poco" and in this directory there will be two
# directories called "include" and "lib".
# 1) After CMake runs, set Poco_INCLUDE_DIR to <Your Path>/poco<-version>
# 2) Use CMAKE_INCLUDE_PATH to set a path to <Your Path>/poco<-version>. This will allow FIND_PATH()
#    to locate Poco_INCLUDE_DIR by utilizing the PATH_SUFFIXES option. e.g.
#    SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "<Your Path>/include")
# 3) Set an environment variable called ${POCO_ROOT} that points to the root of where you have
#    installed Poco, e.g. <Your Path>. It is assumed that there is at least a subdirectory called
#    Foundation/include/Poco in this path.
#
# Note:
#  1) If you are just using the poco headers, then you do not need to use
#     Poco_LIBRARY_DIR in your CMakeLists.txt file.
#  2) If Poco has not been installed, then when setting Poco_LIBRARY_DIR
#     the script will look for /lib first and, if this fails, then for /stage/lib.
#
# Usage:
# In your CMakeLists.txt file do something like this:
# ...
# # Poco
# FIND_PACKAGE(Poco COMPONENTS XML Net Data...)
# ...
# INCLUDE_DIRECTORIES(${Poco_INCLUDE_DIRS})
# LINK_DIRECTORIES(${Poco_LIBRARY_DIR})
#
# In Windows, we make the assumption that, if the Poco files are installed, the default directory
# will be C:\poco or C:\Program Files\Poco or C:\Programme\Poco.

MESSAGE(STATUS "Searching for Poco library...")

SET(POCO_INCLUDE_PATH_DESCRIPTION "top-level directory containing the poco include directories. E.g /usr/local/include/ or c:\\poco\\include\\poco-1.3.2")
SET(POCO_INCLUDE_DIR_MESSAGE "Set the Poco_INCLUDE_DIR cmake cache entry to the ${POCO_INCLUDE_PATH_DESCRIPTION}")
SET(POCO_LIBRARY_PATH_DESCRIPTION "top-level directory containing the poco libraries.")
SET(POCO_LIBRARY_DIR_MESSAGE "Set the Poco_LIBRARY_DIR cmake cache entry to the ${POCO_LIBRARY_PATH_DESCRIPTION}")


SET(POCO_DIR_SEARCH $ENV{POCO_ROOT})
IF(POCO_DIR_SEARCH)
  FILE(TO_CMAKE_PATH ${POCO_DIR_SEARCH} POCO_DIR_SEARCH)
ENDIF(POCO_DIR_SEARCH)


IF(WIN32)
  SET(POCO_DIR_SEARCH
    ${POCO_DIR_SEARCH}
    C:/poco
    D:/poco
    "C:/Program Files/poco"
    "C:/Programme/poco"
    "D:/Program Files/poco"
    "D:/Programme/poco"
  )
ENDIF(WIN32)

# Add in some path suffixes. These will have to be updated whenever a new Poco version comes out.
SET(SUFFIX_FOR_INCLUDE_PATH
 poco-1.3.2
 poco-1.3.3
 poco-1.3.4
 poco-1.3.5
 poco-1.3.6
)

SET(SUFFIX_FOR_LIBRARY_PATH
 poco-1.3.2/lib
 poco-1.3.2/lib/Linux/i686
 poco-1.3.2/lib/Linux/x86_64
 poco-1.3.3/lib
 poco-1.3.3/lib/Linux/i686
 poco-1.3.3/lib/Linux/x86_64
 poco-1.3.4/lib
 poco-1.3.4/lib/Linux/i686
 poco-1.3.4/lib/Linux/x86_64
 poco-1.3.5/lib
 poco-1.3.5/lib/Linux/i686
 poco-1.3.5/lib/Linux/x86_64
 poco-1.3.6/lib
 poco-1.3.6/lib/Linux/i686
 poco-1.3.6/lib/Linux/x86_64
 lib
 lib/Linux/i686
 lib/Linux/x86_64
)

#
# Look for an installation.
#
FIND_PATH(Poco_INCLUDE_DIR NAMES Foundation/include/Poco/SharedLibrary.h PATH_SUFFIXES ${SUFFIX_FOR_INCLUDE_PATH} PATHS

# Look in other places.
  ${POCO_DIR_SEARCH}

# Help the user find it if we cannot.
  DOC "The ${POCO_INCLUDE_PATH_DESCRIPTION}"
)

IF(NOT Poco_INCLUDE_DIR)

# Look for standard unix include paths
  FIND_PATH(Poco_INCLUDE_DIR Poco/Poco.h DOC "The ${POCO_INCLUDE_PATH_DESCRIPTION}")

ENDIF(NOT Poco_INCLUDE_DIR)

# Assume we didn't find it.
SET(Poco_FOUND 0)

# Now try to get the include and library path.
IF(Poco_INCLUDE_DIR)
  IF(EXISTS "${Poco_INCLUDE_DIR}/Foundation/include/Poco/SharedLibrary.h")
    SET(Poco_INCLUDE_DIRS
      ${Poco_INCLUDE_DIR}/CppUnit/include
      ${Poco_INCLUDE_DIR}/Foundation/include
      ${Poco_INCLUDE_DIR}/Net/include
      ${Poco_INCLUDE_DIR}/Util/include
      ${Poco_INCLUDE_DIR}/XML/include
    )
    SET(Poco_FOUND 1)
  ELSEIF(EXISTS "${Poco_INCLUDE_DIR}/Poco/Poco.h")
    SET(Poco_INCLUDE_DIRS
      ${Poco_INCLUDE_DIR}
    )
    SET(Poco_FOUND 1)
  ENDIF()

  IF(NOT Poco_LIBRARY_DIR)

    FIND_LIBRARY(Poco_FOUNDATION_LIB NAMES PocoFoundation PocoFoundationd  PATH_SUFFIXES ${SUFFIX_FOR_LIBRARY_PATH} PATHS

# Look in other places.
      ${Poco_INCLUDE_DIR}
      ${POCO_DIR_SEARCH}

# Help the user find it if we cannot.
      DOC "The ${POCO_LIBRARY_PATH_DESCRIPTION}"
    )
    SET(Poco_LIBRARY_DIR "" CACHE PATH POCO_LIBARARY_PATH_DESCRIPTION)
    GET_FILENAME_COMPONENT(Poco_LIBRARY_DIR ${Poco_FOUNDATION_LIB} PATH)
    SET(Poco_LIBRARIES "")
    SET(Comp_List "")
    IF(Poco_LIBRARY_DIR AND Poco_FOUNDATION_LIB)
# Look for the poco binary path.
      SET(Poco_BINARY_DIR ${Poco_INCLUDE_DIR})
      IF(Poco_BINARY_DIR AND EXISTS "${Poco_BINARY_DIR}/bin")
        SET(Poco_BINARY_DIRS ${Poco_BINARY_DIR}/bin)
      ENDIF(Poco_BINARY_DIR AND EXISTS "${Poco_BINARY_DIR}/bin")
    ENDIF(Poco_LIBRARY_DIR AND Poco_FOUNDATION_LIB)
    IF(Poco_FOUNDATION_LIB)
      IF ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	SET(DBG "d")
      ELSE ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	SET(DBG "")
      ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
      SET(Comp_List "Foundation${DBG}")
      FOREACH(COMPONENT ${Poco_FIND_COMPONENTS})
	FIND_LIBRARY(LIB${COMPONENT} "Poco${COMPONENT}${DBG}" Poco_LIBRARY_DIR)
	IF (LIB${COMPONENT})
	  LIST(APPEND Poco_LIBRARIES "${LIB${COMPONENT}}")
	  LIST(APPEND Comp_List "${COMPONENT}${DBG}")
	ENDIF(LIB${COMPONENT})
      ENDFOREACH(COMPONENT)
      LIST(REMOVE_DUPLICATES Comp_List)
    ENDIF(Poco_FOUNDATION_LIB)
    ENDIF(NOT Poco_LIBRARY_DIR)
ENDIF(Poco_INCLUDE_DIR)

IF(NOT Poco_FOUND)
  IF(Poco_FIND_QUIETLY)
    MESSAGE(STATUS "Poco was not found. ${POCO_INCLUDE_DIR_MESSAGE}")
  ELSE(Poco_FIND_QUIETLY)
    IF(Poco_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Poco was not found. ${POCO_INCLUDE_DIR_MESSAGE}")
    ENDIF(Poco_FIND_REQUIRED)
  ENDIF(Poco_FIND_QUIETLY)
ELSE(NOT Poco_FOUND)
  MESSAGE(STATUS "  Found Poco!")
  SET(COMPONENT_STR "components found:")
  FOREACH(comp ${Comp_List})
   SET(COMPONENT_STR "${COMPONENT_STR}, ${comp}")

  ENDFOREACH(comp ${Comp_List})
  STRING(REPLACE ":," ":" COMPONENT_LSTR ${COMPONENT_STR})
  MESSAGE(STATUS "${COMPONENT_LSTR}.")
ENDIF(NOT Poco_FOUND)

#I added this in to add "libdl" on non-Windows systems. Technically dl is only neded if the "Foundation" component is used,
#but i doesn't hurt to add it in anyway - mas
if(Poco_FOUND AND NOT WIN32)
  LIST(APPEND Poco_LIBRARIES "dl")
endif(Poco_FOUND AND NOT WIN32)
