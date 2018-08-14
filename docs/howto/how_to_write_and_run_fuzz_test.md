# How to Write and Run Fuzz Test

## Introduction

Fuzz test is an automated software testing technique that involves providing unexpected or random data to target program, and monitor for exceptions such as crashes, memory leaks, or failing build-in assertions. This [commit](https://github.com/ApolloAuto/apollo/commit/28e55b367b630ecdb540f46d9a14c393cb584532), adds fuzzing support to the apollo platform, which includes:

* A new command `apllo.sh build_fuzz_test` to build all the fuzz test cases, each of which tests one of the Apollo modules or functions by constantly exploring internal program states with mutated input. 
* A new `CROSSTOOL` for `clang-6.0` which is required to build these fuzz tests with [sanitizer](https://github.com/google/sanitizers) support. 
* An external repository [BaiduXLab/libprotobuf-mutator](https://github.com/BaiduXLab/libprotobuf-mutator), that provides structure(protobuf)-aware mutations, which greatly improve the efficiency of fuzzing. 
* Several fuzz test cases that are ready to be compiled and run, covered part of the functionalities in `/control` and `prediction` modules. 

This tutorial will walk you through the creation and execution of fuzz test. 

## Getting Started 

Source code of the fuzz test cases are in `/apollo/modules/tools/fuzz/`, and I will use the `control_fuzz.cc` resides in the `control` subfolder as an example to show how fuzz works and how to write it. 

To test the functionality of `control` module, ideally we want to provide mutated `Localization`, `ADCTrajectory`, `Chassis`, and `PadMessage` for the `control` module to `ProduceControlCommand`, and see if any input message will trigger unexpected memory corruptions or leaks. If you haven't heard about [libfuzzer](https://llvm.org/docs/LibFuzzer.html) before, it's recommended to try out some simple [examples](https://github.com/google/fuzzer-test-suite/blob/master/tutorial/libFuzzerTutorial.md), which will help you better understand the code we are about to look at. 

We first create a target function to fuzz, which implements the logics mentioned above. As shown in `control_fuzz.cc`, from line `77` to `101`. The `FuzzTarget()` function takes four types of message, and feed them to the control module, before calling the `ProduceControlCommand()`. 

```
bool ControlFuzz::FuzzTarget(
  PadMessage pad_message,
  LocalizationEstimate localization_message,
  ADCTrajectory planning_message,
  Chassis chassis_message) {
    ...
  control_.OnPad(pad_message);
  AdapterManager::FeedLocalizationData(localization_message); 
  AdapterManager::FeedPlanningData(planning_message);
  AdapterManager::FeedChassisData(chassis_message);
    ...
  auto err = control_.ProduceControlCommand(&control_command_);
}
```
Writing a good `FuzzTarget()` is the key to effective fuzzing, a good `FuzzTarget()` should cover the main workflow of target program, but strip irrelevant code to improve performance. It requires some understanding of the target program. In the Apollo case, reading the unit test code can be great help.

Now, we are going to write the entry point for the fuzzer. An entry point is the function that accepts mutated input from fuzzer and calls the `FuzzTarget()`, and sometimes do the initialization job before running the target. 

Differing from the function prototype used in libfuzzer, `int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size)`, which takes raw `uint8_t` buffer. We integrated protobuf-aware fuzzing, whose entry point looks like `DEFINE_PROTO_FUZZER(const apollo::tools::fuzz::control_message& message)`. By providing it with a `.proto` file, the fuzzer infers the data structure, and ensures that only valid protobuf message will be produced, which greatly improves the efficiency. The `libprotobuf-mutator` is hosted by Baidu X-Lab [GitHub](https://github.com/BaiduXLab/libprotobuf-mutator), and credit to the awesome [job](https://github.com/google/libprotobuf-mutator) done by Google.

```
DEFINE_PROTO_FUZZER(const apollo::tools::fuzz::control_message& message) {
  apollo::control::control_fuzzer.SetUp();
  if (message.has_planning() &&
    message.planning().header().module_name() == "planning"
    && message.planning().trajectory_point().size() > 0) {
    apollo::control::control_fuzzer.FuzzTarget(
      message.pad(),
      message.localization(),
      message.planning(),
      message.chassis());
  }
}
```

The definition of `apollo::tools::fuzz::control_message` can be found at `/apollo/modules/tools/fuzz/control/proto/control_message.proto`, which includes all the four different types of messages that the `control` module consumes to make a control decision. In the entry point function, we will dispatch and feed each type of message to the `FuzzTarget(...)` function. 

You probably have noticed that we add some constraints before calling the target function. It's a little bit weird since fuzzing is intended to discover these corner conditions. It's related to the limitation of in-process fuzzing, such as libfuzzer, which would stop if the target program `exit()`. These constraints ensures that the fuzzing will not take the same path of previously discovered crashes, so the fuzz can continue, and focus on discovering new crashes. 

## Fuzz Test Compilation

The easiest way to build all fuzz test cases is to run `apollo.sh build_fuzz_test`, which will automatically discover all Bazel targets in `/apollo/modules/tools/fuzz` and build them. The build script will also install `clang-6.0` for you in the docker, if it's missing. However, if you want to build only specific fuzz test case, use the following build command:
```
bazel build --crosstool_top=tools/clang-6.0:toolchain modules/tools/fuzz/control:control_fuzz --compilation_mode=dbg
```
The `crosstool_top` points to the `clang-6.0` toolchain, and the second argument is the target defined in the corresponding `BUILD` file. If we take a look at `/apollo/modules/tools/fuzz/control/BUILD`:

```
cc_binary(
    name = "control_fuzz",
    srcs = [
        "control_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/control:control_lib",
        "//modules/tools/fuzz/control/proto:control_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)
```
Special compiler options and link options need to be provided. The `-fsanitize=fuzzer,address,undefined` will build the target as a fuzz test case, with `AddressSanitizer` and `UndefinedBehaviorSanitizer` enabled. Of course, you can try other sanitizers such as `MemorySanitizer` and `ThreadSanitizer`, but just remember that some of them are mutually exclusive. Note that the external repo `"@libprotobuf_mutator//:mutator"` needs to be added to the `deps` list. 

The `--compilation_mode=dbg` is equivalent to the `-g` option of `gcc` and will add debug information to facilitate the root cause diagnosis of potential crashes. 

## Start Fuzzing

The compiled binary is located at `/apollo/bazel-bin/modules/tools/fuzz/[module_name]`. You can run them directly, or explore more options from libfuzzer [page](https://llvm.org/docs/LibFuzzer.html). 
```
./bazel-bin/modules/tools/fuzz/control_fuzz
```
To disable the massive `DEBUG` message from `GLOG` during fuzzing, run this command in your terminal:

```
GLOG_minloglevel=4
```

Fuzzing works **best** if high quality seeds are provided. In the Apollo context, seeds can be valid protobuf messages, and many of them can be found in each modules' `testdata/` folder. Find the sample test data that is the same type with the expected message of your target program, and copy them into `seeds` folder, you can start fuzzing with seeds by:

```
./bazel-bin/modules/tools/fuzz/control_fuzz ./CORPUS ./seeds/ -jobs=20 
```

The first folder `CORPUS` is used to log useful test cases produced during fuzzing, they are usually test cases that helped explore new program paths. You can name the folder whatever name you want. The `-jobs=20` will parallelize the fuzzing, and they will share the `CORPUS`. 

If the fuzzing stops with message like:
```
==2335==ERROR: AddressSanitizer: heap-buffer-overflow on address 0x602000155c13 at pc 0x0000004ee637...
READ of size 1 at 0x602000155c13 thread T0 
    #0 0x4ee636 in QueryNearestPointByPosition control/module/common/trajectory_analyzer.cc:10:7
    #1 0x4ee6aa in LLVMFuzzerTestOneInput control_fuzz.cc:14:3
``` 
Congratulation, a memory corruption is found, and the debug message usually can help you locate the lines where the crash occurs. 

## Crash Diagnosis

If the fuzzing stops on crash, a test case will be saved at `./crash-0x12345678`. You can check if this crash can be reproduced by running:
```
./bazel-bin/modules/tools/fuzz/control_fuzz ./crash-0x12345678
```
`gdb` is prefered to be used for diagnosing complex issues, commonly used commands are:
```
gdb bazel-bin/control/controller/lon_controller_fuzzer
set args ./crash-0x12345678
run # usually will stop on crash
bt  # output backtrace of crash
```
## Reporting Bugs

You are encouraged to report the problems discovered to the Apollo team. Just open an issue and probably with a pull request following these [steps](https://github.com/ApolloAuto/apollo/blob/master/CONTRIBUTING.md).

## Further Reading

libprotobuf-mutator [tutorial](https://github.com/BaiduXLab/libprotobuf-mutator)
libfuzzer command line [options](https://llvm.org/docs/LibFuzzer.html)

