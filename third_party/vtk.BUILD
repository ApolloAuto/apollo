licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# This assumes you have vtk pre-installed in your system.
cc_library(
    name = "vtk",
    copts = [
        "-Wno-deprecated",
    ],
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
)
