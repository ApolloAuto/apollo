package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "caffe",
    srcs = [
        "build/lib/libcaffe.so",
        "build/lib/libproto.a",
        #"build/include/caffe/proto/caffe.ph.cc",
    ],
    hdrs = glob([
        "include/caffe/*.hpp",
        "build/include/caffe/proto/*.ph.h",
    ]),
    defines = [
        "CPU_ONLY"
    ],
    includes = [
        "include",
        "build/include"
    ],
    #linkopts = [
    #    "-lrt",
    #    "-lboost_system",
    #],
)