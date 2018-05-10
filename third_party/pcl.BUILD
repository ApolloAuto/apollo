licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# This assumes you have pcl 1.7 pre-installed in your system.
cc_library(
    name = "pcl",
    defines = ["PCL_NO_PRECOMPILE"],
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lboost_system",
        "-lpcl_common",
        "-lpcl_features",
        "-lpcl_filters",
        "-lpcl_io_ply",
        "-lpcl_io",
        "-lpcl_kdtree",
        "-lpcl_keypoints",
        "-lpcl_octree",
        "-lpcl_outofcore",
        "-lpcl_people",
        "-lpcl_recognition",
        "-lpcl_registration",
        "-lpcl_sample_consensus",
        "-lpcl_search",
        "-lpcl_segmentation",
        "-lpcl_surface",
        "-lpcl_tracking",
        "-lpcl_visualization",
    ],
)
