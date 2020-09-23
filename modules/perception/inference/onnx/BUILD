load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
	name = "onnx_obstacle_detector",
	srcs = ["onnx_obstacle_detector.cc"],
	hdrs = [
        "onnx_obstacle_detector.h",
    ],
    deps = [
    	"//modules/perception/common:perception_gflags",
    	"//modules/perception/inference:inference_lib",
    	"@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
	name = "libtorch_obstacle_detector",
	srcs = ["libtorch_obstacle_detector.cc"],
	hdrs = [
        "libtorch_obstacle_detector.h",
    ],
    deps = [
    	"//modules/perception/common:perception_gflags",
    	"//modules/perception/inference:inference_lib",
    	"//cyber/common:log",
    	"//third_party:libtorch",
    ]
)

cc_test(
	name = "libtorch_obstacle_detector_test",
	size = "small",
	srcs = ["libtorch_obstacle_detector_test.cc"],
	deps = [
		"//modules/perception/common:perception_gflags",
		":libtorch_obstacle_detector",
		"@com_google_googletest//:gtest_main",
        "//third_party:libtorch",
	],
)