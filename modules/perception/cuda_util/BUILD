package(default_visibility = ["//visibility:public"])

licenses(["notice"])

genrule(
    name = "cuda_util_genrule",
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
       "@caffe//:lib",
       "@cuda",
   ],
)
