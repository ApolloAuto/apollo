load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_service_lib",
    srcs = [
        "motion_service.cc",
    ],
    hdrs = [
        "motion_service.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/camera/common",
        "//modules/perception/lib/registerer",
        "//modules/perception/camera/lib/motion",
        "//modules/perception/lib/utils",
	    "//modules/perception/proto:perception_proto",
        "//modules/perception/onboard/proto:motion_service_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

cc_binary(
    name = "libmotion_service.so",
    linkshared = True,
    linkstatic = False,
    deps = [":motion_service_lib"],
)

cpplint()
