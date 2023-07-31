load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_manager",
    srcs = ["sensor_manager.cc"],
    hdrs = ["sensor_manager.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/io:io_util",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/base:camera",
        "//modules/perception/common/base:omnidirectional_model",
        "//modules/perception/common/base:polynomial",
        "//modules/perception/common/proto:sensor_meta_schema_cc_proto",
    ],
)

#cc_test(
 #   name = "sensor_manager_test",
  #  size = "small",
  #  srcs = ["sensor_manager_test.cc"],
   # data = [
    #    "//modules/perception/data:perception_testdata",
    #],
    #deps = [
     #   ":sensor_manager",
      #  "@com_google_googletest//:gtest_main",
    #],
#)

apollo_package()
cpplint()
