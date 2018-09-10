# Try to find gnu scientific library GSL
# See
# http://www.gnu.org/software/gsl/  and
# http://gnuwin32.sourceforge.net/packages/gsl.htm
#
# Once run this will define:
#
# GSL_FOUND       = system has GSL lib
#
# GSL_LIBRARIES   = full path to the libraries
#    on Unix/Linux with additional linker flags from "gsl-config --libs"
#
# CMAKE_GSL_CXX_FLAGS  = Unix compiler flags for GSL, essentially "`gsl-config --cxxflags`"
#
# GSL_INCLUDE_DIRS      = where to find headers
#
# GSL_LINK_DIRECTORIES = link directories, useful for rpath on Unix
# GSL_EXE_LINKER_FLAGS = rpath on Unix
#
# Felix Woelk 07/2004
# Jan Woetzel
#
# www.mip.informatik.uni-kiel.de
# --------------------------------

IF(WIN32)
# JW tested with gsl-1.8, Windows XP, MSVS 7.1, MSVS 8.0
SET(GSL_POSSIBLE_ROOT_DIRS
${GSL_ROOT_DIR}
$ENV{GSL_ROOT_DIR}
${GSL_DIR}
${GSL_HOME}
$ENV{GSL_DIR}
$ENV{GSL_HOME}
$ENV{EXTERN_LIBS_DIR}/gsl
$ENV{EXTRA}
# "C:/home/jw/source2/gsl-1.8"
)
FIND_PATH(GSL_INCLUDE_DIRS
NAMES gsl/gsl_cdf.h gsl/gsl_randist.h
PATHS ${GSL_POSSIBLE_ROOT_DIRS}
PATH_SUFFIXES include
DOC "GSL header include dir"
)

FIND_LIBRARY(GSL_GSL_LIBRARY
NAMES gsl libgsl
PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
PATH_SUFFIXES lib
DOC "GSL library dir" )

FIND_LIBRARY(GSL_GSLCBLAS_LIBRARY
NAMES gslcblas libgslcblas
PATHS  ${GSL_POSSIBLE_ROOT_DIRS}
PATH_SUFFIXES lib
DOC "GSL cblas library dir" )

SET(GSL_LIBRARIES ${GSL_GSL_LIBRARY})

#MESSAGE("DBG\n"
#  "GSL_GSL_LIBRARY=${GSL_GSL_LIBRARY}\n"
#  "GSL_GSLCBLAS_LIBRARY=${GSL_GSLCBLAS_LIBRARY}\n"
#  "GSL_LIBRARIES=${GSL_LIBRARIES}")


ELSE(WIN32)

IF(UNIX)
SET(GSL_CONFIG_PREFER_PATH
 "$ENV{GSL_DIR}/bin"
 "$ENV{GSL_DIR}"
 "$ENV{GSL_HOME}/bin"
 "$ENV{GSL_HOME}"
 CACHE STRING "preferred path to GSL (gsl-config)")
FIND_PROGRAM(GSL_CONFIG gsl-config
 ${GSL_CONFIG_PREFER_PATH}
 /usr/bin/
 )
# MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")

IF (GSL_CONFIG)

 MESSAGE(STATUS "GSL using gsl-config ${GSL_CONFIG}")
 # set CXXFLAGS to be fed into CXX_FLAGS by the user:
 EXEC_PROGRAM(${GSL_CONFIG}
   ARGS --cflags
   OUTPUT_VARIABLE  GSL_CXX_FLAGS )
 #SET(GSL_CXX_FLAGS "`${GSL_CONFIG} --cflags`")

 # set INCLUDE_DIRS to prefix+include
 EXEC_PROGRAM(${GSL_CONFIG}
   ARGS --prefix
   OUTPUT_VARIABLE GSL_PREFIX)
 SET(GSL_INCLUDE_DIRS ${GSL_PREFIX}/include CACHE STRING INTERNAL)

 # set link libraries and link flags

 #SET(GSL_LIBRARIES "`${GSL_CONFIG} --libs`")

 # extract link dirs for rpath
 EXEC_PROGRAM(${GSL_CONFIG}
   ARGS --libs
   OUTPUT_VARIABLE  GSL_CONFIG_LIBS )
 SET(GSL_LIBRARIES "${GSL_CONFIG_LIBS}")

 # split off the link dirs (for rpath)
 # use regular expression to match wildcard equivalent "-L*"
 # with  is a space or a semicolon
 STRING(REGEX MATCHALL "[-][L]([^ ;])+"
   GSL_LINK_DIRECTORIES_WITH_PREFIX
   "${GSL_CONFIG_LIBS}" )
 #      MESSAGE("DBG  GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")

 # remove prefix -L because we need the pure directory for LINK_DIRECTORIES

 IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
   STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
 ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
 SET(GSL_EXE_LINKER_FLAGS "-Wl,-rpath,${GSL_LINK_DIRECTORIES}" CACHE STRING INTERNAL)
 #      MESSAGE("DBG  GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
 #      MESSAGE("DBG  GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")

 #      ADD_DEFINITIONS("-DHAVE_GSL")
 #      SET(GSL_DEFINITIONS "-DHAVE_GSL")
 MARK_AS_ADVANCED(
   GSL_CXX_FLAGS
   GSL_INCLUDE_DIRS
   GSL_LIBRARIES
   GSL_LINK_DIRECTORIES
   GSL_DEFINITIONS
)
 MESSAGE(STATUS "Using GSL from ${GSL_PREFIX}")

ELSE(GSL_CONFIG)

 INCLUDE(UsePkgConfig) #needed for PKGCONFIG(...)

 MESSAGE(STATUS "GSL using pkgconfig")
 #      PKGCONFIG(gsl includedir libdir linkflags cflags)
 PKGCONFIG(gsl GSL_INCLUDE_DIRS GSL_LINK_DIRECTORIES GSL_LIBRARIES GSL_CXX_FLAGS)
 IF(GSL_INCLUDE_DIRS)
MARK_AS_ADVANCED(
     GSL_CXX_FLAGS
     GSL_INCLUDE_DIRS
     GSL_LIBRARIES
     GSL_LINK_DIRECTORIES
)

 ELSE(GSL_INCLUDE_DIRS) 
MESSAGE("FindGSL.cmake: gsl-config/pkg-config gsl not found. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
 ENDIF(GSL_INCLUDE_DIRS)

ENDIF(GSL_CONFIG)

ENDIF(UNIX)
ENDIF(WIN32)


IF(GSL_LIBRARIES)
IF(GSL_INCLUDE_DIRS OR GSL_CXX_FLAGS)

SET(GSL_FOUND 1)

ENDIF(GSL_INCLUDE_DIRS OR GSL_CXX_FLAGS)
ENDIF(GSL_LIBRARIES)


# ==========================================
IF(NOT GSL_FOUND)
# make FIND_PACKAGE friendly
IF(NOT GSL_FIND_QUIETLY)
IF(GSL_FIND_REQUIRED)
 MESSAGE(FATAL_ERROR "GSL required, please specify it's location.")
ELSE(GSL_FIND_REQUIRED)
 MESSAGE(STATUS       "ERROR: GSL was not found.")
ENDIF(GSL_FIND_REQUIRED)
ENDIF(NOT GSL_FIND_QUIETLY)
ENDIF(NOT GSL_FOUND)
