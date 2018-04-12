load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

genrule(
    name = "cuda_util_genrule",
    srcs = [
        "block_uf.h",
        "connected_component_gpu.h",
        "network.h",
        "region_output.h",
        "texture.h",
        "undistortion.h",
        "util.h",
        "CMakeLists.txt",
        "block_uf.cu",
        "connected_component_gpu.cu",
        "network.cu",
        "region_output.cc",
        "region_output.cu",
        "undistortion.cu",
        "util.cu",
    ],
    outs = ["cmake_build/libcuda_util.so"],
    cmd = """mkdir -p modules/perception/cuda_util/cmake_build
             cd modules/perception/cuda_util/cmake_build
             cmake .. && make -j8
             cd -
             cp modules/perception/cuda_util/cmake_build/libcuda_util.so $@
          """,
)

cc_library(
    name = "cuda_util",
    srcs = [
        "cmake_build/libcuda_util.so",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//modules/perception/obstacle/camera/lane_post_process/common:connected_component",
        "@caffe//:lib",
        "@cuda",
    ],
)

cpplint()
