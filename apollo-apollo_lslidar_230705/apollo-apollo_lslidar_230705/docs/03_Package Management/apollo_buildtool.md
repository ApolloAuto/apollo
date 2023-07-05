# Apollo buildtool

## Summary

Apollo buildtool is a command line tool, providing functions such as compiling, testing, installing or running Apollo modules. Based on buildtool, not only can you easily install the binary packages of each module in Apollo, but you can also perform secondary development, compilation, and testing on these source code without downloading the whole Apollo. buildtool allows developers to only focus on the modules that need to be developed to improve overall development efficiency.

## Installation

buildtool currently only supports running in the Apollo container environment. If you haven't started the container, you can use apollo enviromnent tool: **aem** to start or enter an Apollo container environment. Or you can also use the traditional way: Apollo scripts setup.
The Apollo container environment has pre-installed Apollo core and Apollo buildtool. When you enter the container,  you can enter the following command to check whether buildtool has been installed correctly:

```shell
buildtool -h
```

If all goes well, you will see a prompt similar to the image below:

```shell
usage: buildtool [-h]
                 {pack,build,test,init,install,bootstrap,info,reinstall,clean,create}
                 ...

build tools of apollo

positional arguments:
  {pack,build,test,init,install,bootstrap,info,reinstall,clean,create}
                        sub-command
    pack                verify the packaging process
    build               build module
    test                perform the unit test
    init                init a single workspace with cyber example component
    install             install specific package
    bootstrap           start or stop module
    info                list package depends infomation based on a specific
                        compilation parameter
    reinstall           reinstall specific package
    clean               clean build cache and build tool generated files
    create              create specific package

optional arguments:
  -h, --help            show this help message and exit

```

## buildtool action

The buildtool action is a single entry point for all the functions provided by buildtool. The invocation of the buildtool action follows the following form:

```shell
buildtool <action> [action arguments or options]
```

The different functions of buildtool are organized into different action, similar to common command-line tools such as git or apt.
Action covers all functions of buildtool, such as **build** responsible for compiling, **install** responsible for installation, and **clean** responsible for cleaning up compilation cache, etc.
Some parameters may be required after the action, you can enter -h, --help after the action to view the detailed parameters.

## builtin buildtool action

* build - compile the source code in the Apollo workspace
* test - perform the unit test for each package in the Apollo workspace
* install - install the source code of a package to Apollo workspace
* clean - clear the compilation cache of the Apollo workspace
* init - creates a simple Apollo workspace
* info - view the dependency information of the specified package
* bootstrap - bootstrap Apollo specific modules
* reinstall - reinstall the specific package
* create - initialize a user-defined package

The specific usage of each action will be introduced in detail next.

### build

The build action compiles the source code of one or more software packages in the workspace, and build can only be executed in the root directory of the workspace. The build action will throw an error if executed outside the workspace or in a non-root directory within the workspace.

#### Basic usage

##### Compile the source code of all packages under the workspace

```shell
buildtool build
```

When there is no parameter, buildtool will compile all source codes in the workspace. During compilation, buildtool will automatically create dev, modules, third_party, tools folders to save compilation information.

##### Compile the source code of the package in a specified folder under the workspace

When receiving the -p or --packages parameter, buildtool can compile the source code in a specified folder separately:

```shell
$ mkdir demo && cd demo
$ buildtool init
$ tree
|-- WORKSPACE
`-- example_components
    |-- BUILD
    |-- cyberfile.xml
    |-- example-components.BUILD
    |-- example.dag
    |-- example.launch
    |-- proto
    |   |-- BUILD
    |   `-- examples.proto
    `-- src
        |-- BUILD
        |-- common_component_example.cc
        |-- common_component_example.h
        |-- timer_common_component_example.cc
        `-- timer_common_component_example.h
$ buildtool build --packages example_components
```

The above command compiles the source code in example_components. If everything gone well, you will see the following prompt:

```shell
[buildtool] INFO PostProcess example-components-dev
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

#### Advanced usage

##### Specify gpu compilation

```shell
$ buildtool build --gpu
```

##### Pass compilation parameters to bazel

```shell
$ buildtool build --arguments "--linkopts=-lz4"
```

##### Clear the previous compilation cache and compile

```shell
$ buildtool build --expunge
```

##### Specify the number of compilation threads and the percentage of memory used

```shell
$ buildtool build --jobs 2 --memories 0.5
```

##### detailed parameters

```shell
usage: buildtool build [-h] [-p [* [* ...]]] [-a [* [* ...]]] [--gpu] [--cpu]
                       [--dbg] [--opt] [--prof] [--teleop] [--expunge]
                       [-j JOBS] [-m MEMORIES]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --packages [* [* ...]]
                        Specify the package path.
  -a [* [* ...]], --arguments [* [* ...]]
                        Pass arguments to the build system.
  --gpu                 Run build in GPU mode
  --cpu                 Run build in cpu mode
  --dbg                 Build with debugging enabled
  --opt                 Build with optimization enabled
  --prof                Build with profiler enabled
  --teleop              Run build with teleop enabled
  --expunge             Expunge the building cache before build
  -j JOBS, --jobs JOBS  Specifies the number of threads to compile in parallel
  -m MEMORIES, --memories MEMORIES
                        Specifies the percentage of memory used by compilation
```

### test

The test action performs unit testing for the source code of the workspace, and actually executes all cc_test defined in the source code BUILD file.

#### Basic usage

Perform the unit-test of module:

```shell
$ buildtool init
$ buildtool install planning-dev
$ buildtool test --package_paths modules/planning
```

The above operation buildtool will download the planning module, copy the source code of the planning module to the workspace, and finally compile and test it.

##### Specify gpu mode to compile and test

```shell
$ buildtool test --gpu --package_paths modules/planning
```

##### Pass the parameters to bazel

```shell
$ buildtool test --arguments "--linkopts=-llz4" --package_paths modules/planning
```

##### detailed parameters

```shell
usage: buildtool test [-h] [-p [* [* ...]]] [--gpu] [--cpu]
                      [--arguments [* [* ...]]]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --package_paths [* [* ...]]
                        Specify the package path.
  --gpu                 Run build in gpu mode"
  --cpu                 Run build in cpu mode"
  --arguments [* [* ...]]
                        Pass arguments to the build system.
```

### install

This action installs the source code of a specified module into the workspace.

#### usage

```shell
$ buildtool init
$ buildtool install planning-dev
```

The above operation buildtool will download the planning module, copy the source code of the planning module to the workspace.

```shell
$ buildtool init
$ buildtool install --legacy planning-dev
```

The above operation buildtool will only download the planning module, and will not copy the source code to workspace.

##### detailed parameters

```shell
usage: buildtool install [-h] [--legacy] [packages [packages ...]]

positional arguments:
  packages    Install the packages

optional arguments:
  -h, --help  show this help message and exit
  --legacy    legacy way to install package
```

### clean

This action is used to clear the compilation cache and output from the workspace source code.

#### usage

The clean action clears all compilation caches of the workspace when it does not accept any parameters, which is equivalent to executing bazel clean --expunge

```shell
$ buildtool clean
```

The packages_path parameter can specify to delete the compiled output of the module source code under the corresponding path:

```shell
$ buildtool clean --packages_path modules/planning
```

The above operation will delete the compilation output of the planning module.

The expunge parameter will delete the compiled output of all module source codes on the machine:

```shell
$ buildtool clean --expunge
```

##### detailed parameters

```shell
usage: buildtool clean [-h] [--packages_path [* [* ...]]] [--expunge]

optional arguments:
  -h, --help            show this help message and exit
  --packages_path [* [* ...]]
                        clean specified module build production.
  --expunge             clean the build cache including production
```

### init

This action is used to initialize a single workspace.

#### usage

The init action creates a basic workspace in the current directory when it accepts no arguments:

```shell
buildtool init
```

Of course, you can also bring example_component in the workspace.

```shell
buildtool init -w
```

The path parameter can specify the creation path in the workspace:

```shell
$ buildtool init --path ～/demo
```

##### detailed parameters

```shell
usage: buildtool init [-h] [-p PATH] [-w]

optional arguments:
  -h, --help            show this help message and exit
  -p PATH, --path PATH  specify workspace path
  -w, --with-examples   init workspace with example component and lib
```

### info

This action is used to list the dependencies of the specified package.

#### usage

List the direct dependencies of the requested package:

```shell
$ buildtool info planning-dev
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO   using gpu: False
[buildtool] INFO   using debug mode: False
[buildtool] INFO According parameters above to analysis depends of planning-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO planning-dev directly depends on the following packages:
[buildtool] INFO   (3rd-gflags-dev|1.0.0.1), (3rd-absl-dev|1.0.0.1), (3rd-osqp-dev|1.0.0.1), (3rd-glog-dev|1.0.0.1)
[buildtool] INFO   (3rd-proj-dev|1.0.0.1), (libtinyxml2-dev|None), (3rd-boost-dev|1.0.0.1), (3rd-opencv-dev|1.0.0.1)
[buildtool] INFO   (3rd-ipopt-dev|1.0.0.1), (3rd-eigen3-dev|1.0.0.1), (libadolc-dev|None), (3rd-ad-rss-lib-dev|1.0.0.1)
[buildtool] INFO   (cyber-dev|1.0.0.1), (common-dev|1.0.0.1), (map-dev|1.0.0.2), (common-msgs-dev|1.0.0.1)
[buildtool] INFO   (bazel-extend-tools-dev|1.0.0.1), (3rd-mkl-dev|1.0.0.1), (3rd-libtorch-cpu-dev|1.0.0.1), (3rd-protobuf-dev|1.0.0.1)
[buildtool] INFO   (3rd-rules-python-dev|1.0.0.1), (3rd-grpc-dev|1.0.0.1), (3rd-bazel-skylib-dev|1.0.0.1), (3rd-rules-proto-dev|1.0.0.1)
[buildtool] INFO   (3rd-py-dev|1.0.0.1), (3rd-gpus-dev|1.0.0.1), (3rd-gtest-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

List all dependencies of the requested package:

```shell
$ buildtool info --with-indirect planning-dev
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO   using gpu: False
[buildtool] INFO   using debug mode: False
[buildtool] INFO According parameters above to analysis depends of planning-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO planning-dev depends on these following packages:
[buildtool] INFO   (3rd-gflags-dev|1.0.0.1), (3rd-absl-dev|1.0.0.1), (3rd-osqp-dev|1.0.0.1), (3rd-glog-dev|1.0.0.1)
[buildtool] INFO   (3rd-proj-dev|1.0.0.1), (libtinyxml2-dev|None), (3rd-boost-dev|1.0.0.1), (3rd-opencv-dev|1.0.0.1)
[buildtool] INFO   (3rd-ipopt-dev|1.0.0.1), (3rd-eigen3-dev|1.0.0.1), (libadolc-dev|None), (3rd-ad-rss-lib-dev|1.0.0.1)
[buildtool] INFO   (cyber-dev|1.0.0.1), (libncurses5-dev|None), (libuuid1|None), (3rd-rules-python-dev|1.0.0.1)
[buildtool] INFO   (3rd-grpc-dev|1.0.0.1), (3rd-rules-proto-dev|1.0.0.1), (3rd-py-dev|1.0.0.1), (3rd-bazel-skylib-dev|1.0.0.1)
[buildtool] INFO   (3rd-protobuf-dev|1.0.0.1), (3rd-fastrtps-dev|1.0.0.1), (common-msgs-dev|1.0.0.1), (bazel-extend-tools-dev|1.0.0.1)
[buildtool] INFO   (common-dev|1.0.0.1), (libsqlite3-dev|None), (3rd-gtest-dev|1.0.0.1), (3rd-nlohmann-json-dev|1.0.0.1)
[buildtool] INFO   (map-dev|1.0.0.2), (3rd-mkl-dev|1.0.0.1), (3rd-libtorch-cpu-dev|1.0.0.1), (3rd-gpus-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

List the dependency information of the package source code in the specified path:

```shell
$ buildtool info --directory example_components/
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO   using gpu: False
[buildtool] INFO   using debug mode: False
[buildtool] INFO According parameters above to analysis depends of example-components-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO example-components-dev directly depends on the following packages:
[buildtool] INFO   (cyber-dev|1.0.0.1), (bazel-extend-tools-dev|1.0.0.1), (3rd-protobuf-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

##### detailed parameters

```shell
usage: buildtool info [-h] [--depends-on] [--with-indirect] [--directory]
                      [--gpu] [--cpu] [--dbg]
                      [query]

positional arguments:
  [query]          the package name or stored path of which package's depends
                   you want to list

optional arguments:
  -h, --help       show this help message and exit
  --depends-on     list those packages information which are directly
                   dependent on the package
  --with-indirect  list the package all depends, direct and indirect
  --directory      list the package infomation which stored in the directory
  --gpu            with compilation in GPU parameter
  --cpu            with compilation in CPU parameter
  --dbg            with compilation in debugging parameter
```

### bootstrap

bootstrap is used to boot the latest version of a specific package. The latest version refers to the version corresponding to the last operation of the software package. If the last operation of the package is compilation, then bootstrap will start the output of the compiled package. If the last operation of the package is installation or reinstallation, for example, if you execute a command such as sudo apt install --reinstall, then bootstrap will start the binary file or dynamic link library of the installed or reinstalled package.

> Note: Currently, the modules that bootstrap can start are: dreamview-dev, monitor-dev.

#### usage

##### boot dreamview：

```shell
buildtool bootstrap start dreamview-dev
```

##### stop dreamview：

```shell
buildtool bootstrap stop dreamview-dev
```

##### detailed parameters

```shell
usage: buildtool bootstrap [-h]{start,stop}...

positional arguments:
  {start,stop}  options
    start       start module
    stop        stop module

optional arguments:
  -h, --help    show this help message and exit
```

### reinstall

This action is used to reinstall the package, which is equivalent to executing apt install --reinstall.

##### detailed parameters

```shell
usage: buildtool reinstall [-h] [packages [packages ...]]

positional arguments:
  packages    Reinstall the packages

optional arguments:
  -h, --help  show this help message and exit
```

### create

This action is used to initialize the cyberfile of the user defined package.

##### detailed parameters

```shell
usage: buildtool create [-h] [--name NAME] --type {src,binary,wrapper}
                        [--author AUTHOR] [--email EMAIL]
                        package_path

positional arguments:
  package_path          specify the path of this package needed to create

optional arguments:
  -h, --help            show this help message and exit
  --name NAME           specify the name of this package
  --type {src,binary,wrapper}
                        specify the type of this package
  --author AUTHOR       specify this 3rd package author
  --email EMAIL         specify the contacted email
```

### pack

This action is used to pack deb file for a package from building output.

#### usage

```shell
$ buildtool pack -d package.json
```

among them, package.json is releasing description file, you can refer to "Introduction to package of Apollo" to see the detailed example.
