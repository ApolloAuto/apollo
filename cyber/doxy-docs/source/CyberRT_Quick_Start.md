# How to Create and Run a new Component in Cyber RT

Apollo Cyber RT framework is built upon the concept of components. As the
building block of Cyber RT, each component is a specific algorithm module which
processes a set of inputs and generates its set of outputs.

To successfully create and launch a new component, there are basically 4 steps:

- Set up directory layout
- Implement the component class
- Configuration setup
- Launch the component

The example below demonstrates how to create, build and run a simple component
named `CommonComponentExample`. To explore more about Cyber RT, you can find a
couple of examples showing different functionalities of Cyber RT under the
`cyber/examples` directory.

> **Note**: The examples need to run after successfully built within Apollo
> Docker container.

## Set up directry layout

Take the sample component under `cyber/examples/common_component_example` for
example:

- Header file: common_component_example.h
- Source file: common_component_example.cc
- BUILD file: BUILD
- DAG file: common.dag
- Launch file: common.launch

## Implement the sample component class

### Header file

In the header file (`common_component_example.h`) for the sample component:

- Inherit the `Component` base class
- Define your own `Init` and `Proc` functions. Please note that for `proc`,
  input data types need to be specified also.
- Register the sample component class to be globally visible using the
  `CYBER_REGISTER_COMPONENT` macro.

```cpp
#include <memory>

#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::proto::Driver;

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};
CYBER_REGISTER_COMPONENT(CommonComponentSample)
```

### Source File

Implement both the `Init` and `Proc` functions in `common_component_example.cc`:

```cpp
#include "cyber/examples/common_component_example/common_component_example.h"

bool CommonComponentSample::Init() {
  AINFO << "Commontest component init";
  return true;
}

bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
```

### BUILD file

```python
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    linkshared = True,
    linkstatic = False,
    deps = [":common_component_example_lib"],
)

cc_library(
    name = "common_component_example_lib",
    srcs = ["common_component_example.cc"],
    hdrs = ["common_component_example.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
```

## Configuration setup

### DAG file

To configure the DAG file (`common.dag` here), specify the following items:

- Channel names: for data input and output
- Library path: library built from component class
- Class name: the class name of the component

```protobuf
# Define all components in DAG streaming.
module_config {
module_library : "/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so"
components {
    class_name : "CommonComponentSample"
    config {
        name : "common"
        readers {
            channel: "/apollo/prediction"
        }
        readers {
            channel: "/apollo/test"
        }
    }
  }
}
```

### Launch file

To configure the launch (`common.launch`) file, specify the following items:

- The name of the component
- The DAG file created in the previous step
- The name of the process to run the component

```xml
<cyber>
    <component>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </component>
</cyber>
```

## Launch the component

### Build

Build the sample component by running the command below:

```bash
cd /apollo
bash apollo.sh build
```

### Environment setup

Then configure the environment:

```bash
source cyber/setup.bash

# To see output from terminal
export GLOG_alsologtostderr=1
```

### Launch the component

You can choose either of the two ways to launch the newly built component:

- Launch with the launch file (recommended)

```bash
cyber_launch start cyber/examples/common_component_example/common.launch
```

- Launch with the DAG file

```bash
mainboard -d cyber/examples/common_component_example/common.dag
```

### _Feed_ channel data for the component to process

Open another terminal:

```bash
source cyber/setup.bash
export GLOG_alsologtostderr=1
/apollo/bazel-bin/cyber/examples/common_component_example/channel_test_writer
```

Open the 3rd terminal and run:

```bash
source cyber/setup.bash
export GLOG_alsologtostderr=1
/apollo/bazel-bin/cyber/examples/common_component_example/channel_prediction_writer
```

And you should see output from terminal #1 like the following:

```
I0331 16:49:34.736016 1774773 common_component_example.cc:25] [mainboard]Start common component Proc [1094] [766]
I0331 16:49:35.069005 1774775 common_component_example.cc:25] [mainboard]Start common component Proc [1095] [767]
I0331 16:49:35.402289 1774783 common_component_example.cc:25] [mainboard]Start common component Proc [1096] [768]
```
