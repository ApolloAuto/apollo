licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# This assumes you have pcl 1.7 pre-installed in your system.
cc_library(
    name = "vtk",    
    includes = ["."],
    linkopts = [
	"-lvtkCommon",
	"-lvtkCharts",
	"-lvtkCommon",
	"-lvtkDICOMParser",
	"-lvtkFiltering",
	"-lvtkGenericFiltering",
	"-lvtkGeovis",
	"-lvtkGraphics",
	"-lvtkHybrid",
	"-lvtkIO",
	"-lvtkImaging",
	"-lvtkInfovis",
	"-lvtkParallel",
	"-lvtkQtChart",
	"-lvtkRendering",
	"-lvtkViews",
	"-lvtkVolumeRendering",
	"-lvtkWidgets",
	"-lvtkalglib",
	"-lvtkexoIIc",
	"-lvtkftgl",
	"-lvtkmetaio",
	"-lvtkproj4",
	"-lvtksys",
	"-lvtkverdict",
    ],
    copts = [
        "-Wno-deprecated",
    ],
)
