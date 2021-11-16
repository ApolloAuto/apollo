# CUDA Support for Bazel

## Introduction

Since there is no native CUDA support in Bazel (Ref:
[Bazel Issue #6578: Native CUDA Support](https://github.com/bazelbuild/bazel/issues/6578)),
the Authors of Apollo borrowed CUDA support for Bazel from
[TensorFlow](https://https://github.com/tensorflow/tensorflow) project.

## Status

Support upto CUDA Toolkit 11.1 and cuDNN8

## How to generate `find_cuda_config.py.gz.base64`

```
cd ${APOLLO_ROOT_DIR}/third_party/gpus/
python3 compress_find_cuda_config.py
```

## Notes

Please note that CUDA support for Bazel in Apollo was tailored to run on Linux
ONLY. Windows and MacOS support were stripped off which are available in the
original TensorFlow project.

As for CPU architecture, only `x86_64` and `aarch64(arm64)` support was
reserved.

