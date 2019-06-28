
Getting started 
=================

Apollo Cyber RT framework is built based on the concept of component. As a basic building block of Apollo Cyber RT framework, each component contains a specific algorithm module which process a set of data inputs and generate a set of outputs.

In order to successfully create and launch a new component, there are four essential steps that need to happen:

- Set up the component file structure
- Implement the component class
- Set up the configuration files
- Launch the component

The example below demonstrates how to create a simple component, then build, run and watch the final output on screen. If you would like to explore more about Apollo Cyber RT, you can find a couple of examples showing how to use different functionalities of the framework under directory `/apollo/cyber/examples/`.

*Note: the example has to be run within apollo docker environment and it's compiled with Bazel.*


## Set up the component file structure
Please create the following files, assumed under the directory of `/apollo/cyber/examples/common_component_example/`:

- Header file: common_component_example.h
- Source file: common_component_example.cc
- Build file: BUILD
- DAG dependency file: common.dag
- Launch file: common.launch

## Implement the component class

### Implement component header file
To implement `common_component_example.h`:

- Inherit the Component class
- Define your own `Init` and `Proc` functions. Proc function needs to specify its input data types
- Register your component classes to be global by using
`CYBER_REGISTER_COMPONENT`

```cpp
#include <memory>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};

CYBER_REGISTER_COMPONENT(CommonComponentSample)
```

### Implement the source file for the example component

For `common_component_example.cc`, both `Init` and `Proc` functions need to be implemented.

```cpp
#include "cyber/examples/common_component_example/common_component_example.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

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

### Create the build file for the example component

Create bazel BUILD file.

```bash
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    deps = [":common_component_example_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "common_component_example_lib",
    srcs = [
        "common_component_example.cc",
    ],
    hdrs = [
        "common_component_example.h",
    ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
```
## Set up the configuration files

### Configure the DAG dependency file

To configure the DAG dependency file (common.dag), specify the following items as below:

 - Channel names: for data input and output
 - Library path: library built from component class
 - Class name: the class name of the component

```bash
# Define all coms in DAG streaming.
    component_config {
    component_library : "/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so"
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

### Configure the launch file

To configure the launch (common.launch) file, specify the following items:

  - The name of the component
  - The dag file you just created in the previous step.
  - The name of the process which the component runs within

```bash
<cyber>
    <component>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </component>
</cyber>
```

## Launch the component

Build the component by running the command below:

```bash
bash /apollo/apollo.sh build
```

Note: make sure the example component builds fine

Then configure the environment:

```bash
cd /apollo/cyber
source setup.bash
```

There are two ways to launch the component:

- Launch with the launch file (recommended)

```bash
cyber_launch start /apollo/cyber/examples/common_component_example/common.launch
```

- Launch with the DAG file

```bash
mainboard -d /apollo/cyber/examples/common_component_example/common.dag
```
