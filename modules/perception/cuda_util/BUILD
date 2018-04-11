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
        "network.cc",
        "network.cu",
        "region_output.cc",
        "region_output.cu",
        "undistortion.cc",
        "undistortion.cu",
        "util.cc",
        "util.cu",
    ],
    outs = ["build/libcuda_util.so"],
    cmd = """if [ ! -e modules/perception/cuda_util/build ]; then
            mkdir -p modules/perception/cuda_util/build;
            fi
            cd modules/perception/cuda_util/build
            cmake .. && make
            cd -
            cp modules/perception/cuda_util/build/libcuda_util.so $@
          """,
)

cc_library(
    name = "cuda_util",
    srcs = [
        "build/libcuda_util.so",
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
