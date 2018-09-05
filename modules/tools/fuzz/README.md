# Fuzzing Test Cases

## Introduction

Fuzz test feeds random and unexpected data to the target functions, and observe the target program for memory corruptions or undefined behaviors. The folder contains test cases that invokes the self-driing logic to be fuzzed. We are continuously adding new fuzz test cases to cover more functionalities.

Got questions regarding how to run it, contribute new test cases or report bugs? Please refer to this [tutorial](https://github.com/BaiduXLab/apollo/blob/master/docs/howto/how_to_write_and_run_fuzz_test.md)

## Build
```
apollo.sh build_fuzz_test
```

## Run
```
./bazel-bin/modules/tools/fuzz/[fuzz_test_case] [seed_file]
```

## Seeds
Seed file is a valid input file for the target program, it helps fuzzer to reach the code blocks that are of interest faster, and explore more paths. Fuzzing performs best when high quality seeds are provided. You can download the seeds consumed by these fuzzing test cases [here](https://github.com/BaiduXLab/libprotobuf-mutator/releases/download/v1.0/apollo_fuzz_seeds.tgz)