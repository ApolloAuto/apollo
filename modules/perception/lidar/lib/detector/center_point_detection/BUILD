load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "center_point_detection",
    srcs = [
        "center_point_detection.cc",
    ],
    hdrs = [
        "center_point_detection.h",
    ],
    deps = [
        #":iou3d_cpu",
        #":iou3d_nms_api",
        #":iou3d_nms_kernel",
        #":iou3d_nms",
        ":params",
        "//cyber/common",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lib/thread",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
        "@paddleinference//:paddleinference_lib",
        "libpd_infer_custom_op",
        #"@boost"
    ],
    alwayslink = True,
)


cc_library(
    name = "common",
    hdrs = ["common.h"],
)

cc_library(
    name = "params",
    hdrs = ["params.h"],
)


#cc_binary(
#    name = "libpd_infer_custom_op.so",
#    linkshared = True,
#    linkstatic = False,
#    visibility = ["//visibility:private"],
#    srcs = ["iou3d_cpu.cc", "iou3d_nms_api.cc", "iou3d_nms.cc", "iou3d_nms_kernel.cu"],
#    deps = ["@paddleinference//:paddleinference_lib","@local_config_cuda//cuda:cudart"],
#)


cc_import(
    name = "libpd_infer_custom_op",
    shared_library = "libpd_infer_custom_op.so",
)

#cc_library(
#    name = "libpd_infer_custom_op.so",
#    srcs = ["iou3d_cpu.cc", "iou3d_nms_api.cc", "iou3d_nms.cc","iou3d_cpu.h", "iou3d_nms.h"],
#    hdrs = ["iou3d_cpu.h", "iou3d_nms.h"],
#    deps = [
#        ":iou3d_nms_kernel",
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#        "@paddleinference//:paddleinference_lib",
#    ],
#)



#cc_library(
#    name = "iou3d_cpu",
#    srcs = [
#        "iou3d_cpu.cc",
#    ],
#    hdrs = [
#        "iou3d_cpu.h",
#    ],
#    includes = ["paddle/include"],
#    deps = [
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#        "@paddleinference//:paddleinference_lib",
#        "@boost"
#    ],
#)

#cc_library(
#    name = "iou3d_nms_api",
#    srcs = [
#        "iou3d_nms_api.cc",
#    ],
#    includes = ["paddle/include/"],
#    deps = [
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#        "@paddleinference//:paddleinference_lib",
#        "@boost"
#    ],
#)

cuda_library(
    name = "iou3d_nms_kernel",
    srcs = [
        "iou3d_nms_kernel.cu",
    ],
)


#cc_library(
#    name = "iou3d_nms",
#    srcs = [
#        ":iou3d_cpu",
#    ],
#    hdrs = [
#        "iou3d_nms.h",
#    ],
#    includes = ["paddle/include"],
#    deps = [
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#        "@paddleinference//:paddleinference_lib",
#        "@boost"
#    ],
#)

#cc_binary(
#    name = "pd_infer_custom_opt.so",
#    srcs = [
#        "iou3d_cpu",
#        "iou3d_nms",
#        "iou3d_nms_api",
#        "iou3d_nms_kernel",
#    ],
#    includes = ["paddle/include"],
#    deps = [
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#        "@paddleinference//:paddleinference_lib",
#        "@boost"
#    ],
#)




#cc_library(
#    name = "common",
#    hdrs = ["common.h"],
#)

#cuda_library(
#    name = "pfe_cuda",
#    srcs = ["pfe_cuda.cu"],
#    hdrs = ["pfe_cuda.h"],
#    deps = [
#        ":common",
#        "@local_config_cuda//cuda:cudart",
#    ],
#)


#cc_test(
#    name = "point_pillars_test",
#    size = "large",
#    srcs = ["point_pillars_test.cc"],
#    deps = [
#        ":point_pillars",
#        ":preprocess_points",
#        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
#        "@boost",
#        "@com_google_googletest//:gtest_main",
#        "@eigen",
#        "@local_config_pcl//:pcl",
#    ],
#)

cpplint()
