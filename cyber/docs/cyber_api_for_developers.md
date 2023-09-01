# Cyber RT API for Developers

This document provides an extensive technical deep dive into how to create, manipulate and use Cyber RT's API.

## Table of Contents

- [Talker-Listener](#Talker-Listener)
- [Service Creation and Use](#Service-Creation-and-Use)
- [Param parameter service](#Param-parameter-service)
- [Log API](#LOG-API)
- [Building a module based on Component](#Building-a-module-based-on-Component)
- [Launch](#Launch)
- [Timer](#timer)
- [Time API](#use-of-time)
- [Record file: Read and Write](#Record-file-Read-and-Write)
- [C++ API Directory](##API-Directory)
    - [Node](#node-api)
    - [Writer](#writer-api)
    - [Client](#client-api)
    - [Parameter](#parameter-api)
    - [Timer](#timer-api)
    - [Time](#timer-api)
    - [Duration](#duration-api)
    - [Rate](#rate-api)
    - [RecordReader](#recordreader-api)
    - [RecordWriter](#recordwriter-api)

## Talker-Listener

The first part of demonstrating CyberRT API is to understand the Talker/Listener example. Following are three essential concepts: node (basic unit), reader(facility to read message) and writer(facility to write message) of the example.

### Create a node

In the CyberRT framework, the node is the most fundamental unit, similar to the role of a `handle`. When creating a specific functional object (writer, reader, etc.), you need to create it based on an existing node instance.
The node creation interface is as follows:

```cpp
std::unique_ptr<Node> apollo::cyber::CreateNode(const std::string& node_name, const std::string& name_space = "");
```

- Parameters:
    - node_name: name of the node, globally unique identifier
    - name_space: name of the space where the node is located

    > name_space is empty by default. It is the name of the space concatenated with node_name. The format is `/namespace/node_name`

- Return value - An exclusive smart pointer to Node
- Error Conditions - when `cyber::Init()` has not called, the system is in an uninitialized state, unable to create a node, return nullptr

### Create a writer

The writer is the basic facility used in CyberRT to send messages. Every writer corresponds to a channel with a specific data type.
The writer is created by the `CreateWriter` interface in the node class. The interfaces are listed as below:

```cpp
template <typename MessageT>
   auto CreateWriter(const std::string& channel_name)
       -> std::shared_ptr<Writer<MessageT>>;
template <typename MessageT>
   auto CreateWriter(const proto::RoleAttributes& role_attr)
       -> std::shared_ptr<Writer<MessageT>>;

```

- Parameters:
    - channel_name: the name of the channel to write to
    - MessageT: The type of message to be written out
- Return value - Shared pointer to the Writer object

### Create a reader

The reader is the basic facility used in cyber to receive messages. Reader has to be bound to a callback function when it is created. When a new message arrives in the channel, the callback will be called.
The reader is created by the `CreateReader` interface of the node class. The interfaces are listed as below:

```cpp
template <typename MessageT>
auto CreateReader(const std::string& channel_name, const std::function<void(const std::shared_ptr<MessageT>&)>& reader_func)
    -> std::shared_ptr<Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const ReaderConfig& config,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
    -> std::shared_ptr<cyber::Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const proto::RoleAttributes& role_attr,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
-> std::shared_ptr<cyber::Reader<MessageT>>;
```

- Parameters:
    - MessageT: The type of message to read
    - channel_name: the name of the channel to receive from
    - reader_func: callback function to process the messages

- Return value - Shared pointer to the Reader object

### Code Example

#### Talker (cyber/examples/talker.cc)

```cpp
#include "cyber/cyber.h"
#include "cyber/proto/chatter.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::proto::Chatter;
int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create talker node
  std::shared_ptr<apollo::cyber::Node> talker_node(
      apollo::cyber::CreateNode("talker"));
  // create talker
  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  Rate rate(1.0);
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<apollo::cyber::proto::Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content("Hello, apollo!");
    talker->Write(msg);
    AINFO << "talker sent a message!";
    rate.Sleep();
  }
  return 0;
}
```

#### Listener (cyber/examples/listener.cc)

```cpp
#include "cyber/cyber.h"
#include "cyber/proto/chatter.pb.h"
void MessageCallback(
    const std::shared_ptr<apollo::cyber::proto::Chatter>& msg) {
  AINFO << "Received message seq-> " << msg->seq();
  AINFO << "msgcontent->" << msg->content();
}
int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  auto listener =
      listener_node->CreateReader<apollo::cyber::proto::Chatter>(
          "channel/chatter", MessageCallback);
  apollo::cyber::WaitForShutdown();
  return 0;
}
```

#### Bazel BUILD file(cyber/samples/BUILD)

```python
cc_binary(
    name = "talker",
    srcs = [ "talker.cc", ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "listener",
    srcs = [ "listener.cc", ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)
```

#### Build and Run

- Build: bazel build cyber/examples/…
- Run talker/listener in different terminals:
  - ./bazel-bin/cyber/examples/talker
  - ./bazel-bin/cyber/examples/listener
- Examine the results: you should see message printing out on listener.

## Service Creation and Use

### Introduction

In an autonomous driving system, there are many scenarios that require more from module communication than just sending or receiving messages. Service is another way of communication between nodes. Unlike channel, service implements `two-way` communication, e.g. a node obtains a response by sending a request. This section introduces the `service` module in CyberRT API with examples.

### Demo - Example

Problem: create a client-server model that pass Driver.proto back and forth.
When a request is sent in by the client, the server parses/processes the request and returns the response.

The implementation of the demo mainly includes the following steps.

#### Define request and response messages

All messages in cyber are in the `protobuf` format. Any protobuf message with serialize/deserialize interfaces can be used as the service request and response message. `Driver` in examples.proto is used as service request and response in this example:

```protobuf
// filename: examples.proto
syntax = "proto2";
package apollo.cyber.examples.proto;
message Driver {
    optional string content = 1;
    optional uint64 msg_id = 2;
    optional uint64 timestamp = 3;
};
```

#### Create a service and a client

```cpp
// filename: cyber/examples/service.cc
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("start_node"));
  auto server = node->CreateService<Driver, Driver>(
      "test_server", [](const std::shared_ptr<Driver>& request,
                        std::shared_ptr<Driver>& response) {
        AINFO << "server: I am driver server";
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(0);
      });
  auto client = node->CreateClient<Driver, Driver>("test_server");
  auto driver_msg = std::make_shared<Driver>();
  driver_msg->set_msg_id(0);
  driver_msg->set_timestamp(0);
  while (apollo::cyber::OK()) {
    auto res = client->SendRequest(driver_msg);
    if (res != nullptr) {
      AINFO << "client: response: " << res->ShortDebugString();
    } else {
      AINFO << "client: service may not ready.";
    }
    sleep(1);
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}
```

#### Bazel build file

```python
cc_binary(
    name = "service",
    srcs = [ "service.cc", ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)
```

#### Build and run

- Build service/client: bazel build cyber/examples/…
- Run: ./bazel-bin/cyber/examples/service
- Examining result: you should see content below in apollo/data/log/service.INFO

``` txt
I1124 16:36:44.568845 14965 service.cc:30] [service] server: i am driver server
I1124 16:36:44.569031 14949 service.cc:43] [service] client: response: msg_id: 1 timestamp: 0
I1124 16:36:45.569514 14966 service.cc:30] [service] server: i am driver server
I1124 16:36:45.569932 14949 service.cc:43] [service] client: response: msg_id: 2 timestamp: 0
I1124 16:36:46.570627 14967 service.cc:30] [service] server: i am driver server
I1124 16:36:46.571024 14949 service.cc:43] [service] client: response: msg_id: 3 timestamp: 0
I1124 16:36:47.571566 14968 service.cc:30] [service] server: i am driver server
I1124 16:36:47.571962 14949 service.cc:43] [service] client: response: msg_id: 4 timestamp: 0
I1124 16:36:48.572634 14969 service.cc:30] [service] server: i am driver server
I1124 16:36:48.573030 14949 service.cc:43] [service] client: response: msg_id: 5 timestamp: 0
```

### Precautions

- When registering a service, note that duplicate service names are not allowed
- The node name applied when registering the server and client should not be duplicated either

## Parameter Service

The Parameter Service is used for shared data between nodes, and provides basic operations such as `set`, `get`, and `list`. The Parameter Service is based on the `Service` implementation and contains service and client.

### Parameter Object

#### Supported Data types

All parameters passed through cyber are `apollo::cyber::Parameter` objects, the table below lists the 5 supported parameter types.

Parameter type | C++ data type | protobuf data type
:------------- | :------------- | :--------------
apollo::cyber::proto::ParamType::INT    |   int64_t |   int64
apollo::cyber::proto::ParamType::DOUBLE | double | double
apollo::cyber::proto::ParamType::BOOL   | bool |bool
apollo::cyber::proto::ParamType::STRING  | std::string | string
apollo::cyber::proto::ParamType::PROTOBUF  | std::string | string
apollo::cyber::proto::ParamType::NOT_SET | - | -

Besides the 5 types above, Parameter also supports interface with protobuf object as incoming parameter. Post performing serialization processes the object and converts it to the STRING type for transfer.

#### Creating the Parameter Object

Supported constructors:

```cpp
Parameter();  // Name is empty, type is NOT_SET
explicit Parameter(const Parameter& parameter);
explicit Parameter(const std::string& name);  // type为NOT_SET
Parameter(const std::string& name, const bool bool_value);
Parameter(const std::string& name, const int int_value);
Parameter(const std::string& name, const int64_t int_value);
Parameter(const std::string& name, const float double_value);
Parameter(const std::string& name, const double double_value);
Parameter(const std::string& name, const std::string& string_value);
Parameter(const std::string& name, const char* string_value);
Parameter(const std::string& name, const std::string& msg_str,
          const std::string& full_name, const std::string& proto_desc);
Parameter(const std::string& name, const google::protobuf::Message& msg);
```

Sample code of using Parameter object:

```cpp
Parameter a("int", 10);
Parameter b("bool", true);
Parameter c("double", 0.1);
Parameter d("string", "cyber");
Parameter e("string", std::string("cyber"));
// proto message Chatter
Chatter chatter;
Parameter f("chatter", chatter);
std::string msg_str("");
chatter.SerializeToString(&msg_str);
std::string msg_desc("");
ProtobufFactory::GetDescriptorString(chatter, &msg_desc);
Parameter g("chatter", msg_str, Chatter::descriptor()->full_name(), msg_desc);
```

#### Interface and Data Reading

Interface list:

```cpp
inline ParamType type() const;
inline std::string TypeName() const;
inline std::string Descriptor() const;
inline const std::string Name() const;
inline bool AsBool() const;
inline int64_t AsInt64() const;
inline double AsDouble() const;
inline const std::string AsString() const;
std::string DebugString() const;
template <typename Type>
typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_integral<Type>::value && !std::is_same<Type, bool>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_floating_point<Type>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_convertible<Type, std::string>::value, const std::string&>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_same<Type, bool>::value, bool>::type
value() const;
```

An example of how to use those interfaces:

```cpp
Parameter a("int", 10);
a.Name();  // return int
a.Type();  // return apollo::cyber::proto::ParamType::INT
a.TypeName();  // return string: INT
a.DebugString();  // return string: {name: "int", type: "INT", value: 10}
int x = a.AsInt64();  // x = 10
x = a.value<int64_t>();  // x = 10
x = a.AsString();  // Undefined behavior, error log prompt
f.TypeName();  // return string: chatter
auto chatter = f.value<Chatter>();
```

### Parameter Service

If a node wants to provide a Parameter Service to other nodes, then you need to create a `ParameterService`.

```cpp
/**
 * @brief Construct a new ParameterService object
 *
 * @param node shared_ptr of the node handler
 */
explicit ParameterService(const std::shared_ptr<Node>& node);
```

Since all parameters are stored in the parameter service object, the parameters can be manipulated directly in the ParameterService without sending a service request.

**Setting parameters:**

```cpp
/**
 * @brief Set the Parameter object
 *
 * @param parameter parameter to be set
 */
void SetParameter(const Parameter& parameter);
```

**Getting parameters:**

```cpp
/**
 * @brief Get the Parameter object
 *
 * @param param_name
 * @param parameter the pointer to store
 * @return true
 * @return false call service fail or timeout
 */
bool GetParameter(const std::string& param_name, Parameter* parameter);
```

**Getting the list of parameters:**

```cpp
/**
 * @brief Get all the Parameter objects
 *
 * @param parameters pointer of vector to store all the parameters
 * @return true
 * @return false call service fail or timeout
 */
bool ListParameters(std::vector<Parameter>* parameters);
```

### Parameter Client

If a node wants to use parameter services of other nodes, you need to create a `ParameterClient`.

```cpp
/**
 * @brief Construct a new ParameterClient object
 *
 * @param node shared_ptr of the node handler
 * @param service_node_name node name which provide a param services
 */
ParameterClient(const std::shared_ptr<Node>& node, const std::string& service_node_name);
```

You could also perform `SetParameter`, `GetParameter` and `ListParameters` mentioned under [Parameter Service](#Parameter-Service).

### Demo - example

```cpp
#include "cyber/cyber.h"
#include "cyber/parameter/parameter_client.h"
#include "cyber/parameter/parameter_server.h"

using apollo::cyber::Parameter;
using apollo::cyber::ParameterServer;
using apollo::cyber::ParameterClient;

int main(int argc, char** argv) {
  apollo::cyber::Init(*argv);
  std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("parameter");
  auto param_server = std::make_shared<ParameterServer>(node);
  auto param_client = std::make_shared<ParameterClient>(node, "parameter");
  param_server->SetParameter(Parameter("int", 1));
  Parameter parameter;
  param_server->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();
  param_client->SetParameter(Parameter("string", "test"));
  param_client->GetParameter("string", &parameter);
  AINFO << "string: " << parameter.AsString();
  param_client->GetParameter("int", &parameter);
  AINFO << "int: " << parameter.AsInt64();
  return 0;
}
```

#### Build and run

- Build: bazel build cyber/examples/…
- Run: ./bazel-bin/cyber/examples/paramserver

## Log API

### Log library

Cyber log library is built on top of glog. The following header files need to be included:

```cpp
#include "cyber/common/log.h"
#include "cyber/init.h"
```

### Log configuration

Default global config path: cyber/setup.bash

The configs below could be modified by devloper:

```
export GLOG_log_dir=/apollo/data/log
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
```

### Log initialization

Call the Init method at the code entry to initialize the log:

```cpp++
apollo::cyber::cyber::Init(argv[0]) is initialized.
If no macro definition is made in the previous component, the corresponding log is printed to the binary log.
```

### Log output macro

Log library is encapsulated in Log printing macros. The related log macros are used as follows:

```cpp
ADEBUG << "hello cyber.";
AINFO  << "hello cyber.";
AWARN  << "hello cyber.";
AERROR << "hello cyber.";
AFATAL << "hello cyber.";

```

### Log format

The format is `<MODULE_NAME>.log.<LOG_LEVEL>.<datetime>.<process_id>`

### About log files

Currently, the only different output behavior from default glog is that different log levels of a module will be written into the same log file.

## Building a module based on Component

### Key concepts

#### 1. Component

The component is the base class that Cyber RT provides to build application modules. Each specific application module can inherit the Component class and define its own `Init` and `Proc` functions so that it can be loaded into the Cyber framework.

#### 2. Binary vs Component

There are two options to use Cyber RT framework for applications:

- Binary based: the application is compiled separately into a binary, which communicates with other cyber modules by creating its own `Reader` and `Writer`.
- Component based: the application is compiled into a Shared Library. By inheriting the Component class and writing the corresponding dag description file, the Cyber RT framework will load and run the application dynamically.

##### The essential Component interface

- The component's `Init()` function is like the main function that does some initialization of the algorithm.
- Component's `Proc()` function works like Reader's callback function that is called by the framework when a message arrives.

##### Advantages of using Component

- Component can be loaded into different processes through the launch file, and the deployment is flexible.
- Component can change the received channel name by modifying the dag file without recompiling.
- Component supports receiving multiple types of data.
- Component supports providing multiple fusion strategies.

#### 3. Dag file format

An example dag file:

```protobuf
# Define all coms in DAG streaming.
module_config {
    module_library : "lib/libperception_component.so"
    components {
        class_name : "PerceptionComponent"
        config {
            name : "perception"
            readers {
                channel: "perception/channel_name"
            }
        }
    }
    timer_components {
        class_name : "DriverComponent"
        config {
            name : "driver"
            interval : 100
        }
    }
}
```

- **module_library**: If you want to load the .so library the root directory is the working directory of cyber (the same directory of `setup.bash`)
- **components & timer_component**: Select the base component class type that needs to be loaded.
- **class_name**: the name of the component class to load
- **name**: the loaded class_name as the identifier of the loading example
- **readers**: Data received by the current component, supporting 1-3 channels of data.

### Demo - examples

#### Common_component_example(cyber/examples/common_component_example/*)

Header definition(common_component_example.h)

```cpp
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class Commontestcomponent : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};
CYBER_REGISTER_COMPONENT(Commontestcomponent)
```

Cpp file implementation(common_component_example.cc)

```cpp
#include "cyber/examples/common_component_smaple/common_component_example.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

bool Commontestcomponent::Init() {
  AINFO << "Commontest component init";
  return true;
}

bool Commontestcomponent::Proc(const std::shared_ptr<Driver>& msg0,
                               const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start commontest component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
```

#### Timer_component_example(cyber/examples/timer_component_example/*)

Header definition(timer_component_example.h)

```cpp
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::TimerComponent;
using apollo::cyber::Writer;

class TimertestComponent : public TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  std::shared_ptr<Writer<Driver>> driver_writer_ = nullptr;
};
CYBER_REGISTER_COMPONENT(TimertestComponent)
```

Cpp file implementation(timer_component_example.cc)

```cpp
#include "cyber/examples/timer_component_example/timer_component_example.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

bool TimertestComponent::Init() {
  driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
  return true;
}

bool TimertestComponent::Proc() {
  static int i = 0;
  auto out_msg = std::make_shared<Driver>();
  out_msg->set_msg_id(i++);
  driver_writer_->Write(out_msg);
  AINFO << "timertestcomponent: Write drivermsg->"
        << out_msg->ShortDebugString();
  return true;
}
```

#### Build and run

Use timertestcomponent as example:

- Build: bazel build cyber/examples/timer_component_smaple/…
- Run: mainboard -d cyber/examples/timer_component_smaple/timer.dag

### Precautions

- Component needs to be registered to load the class through SharedLibrary. The registration interface looks like:

```cpp
CYBER_REGISTER_COMPONENT(DriverComponent)
```

If you use a namespace when registering, you also need to add a namespace when you define it in the dag file.

- The configuration files of the Component and TimerComponent are different, please be careful not to mix the two up.

## Launch

**cyber_launch** is the launcher of the Cyber RT framework. It starts multiple mainboards according to the launch file, and loads different components into different mainboards according to the dag file.
cyber_launch supports two scenarios for dynamically loading components or starting Binary programs in a child process.

### Launch File Format

```xml
<cyber>
    <module>
        <name>driver</name>
        <dag_conf>driver.dag</dag_conf>
        <process_name></process_name>
        <exception_handler>exit</exception_handler>
    </module>
    <module>
        <name>perception</name>
        <dag_conf>perception.dag</dag_conf>
        <process_name></process_name>
        <exception_handler>respawn</exception_handler>
    </module>
    <module>
        <name>planning</name>
        <dag_conf>planning.dag</dag_conf>
        <process_name></process_name>
    </module>
</cyber>
```

**Module**:
Each loaded component or binary is a module

- **name** is the loaded module name
- **dag_conf** is the name of the corresponding dag file of the component
- **process_name** is the name of the mainboard process once started, and the same component of process_name will be loaded and run in the same process.
- **exception_handler** is the handler method when the exception occurs in the process. The value can be exit or respawn listed below.
    - exit, which means that the entire process needs to stop running when the current process exits abnormally.
    - respawn, the current process needs to be restarted after abnormal exit. Start this process. If there is no such thing as it is empty, it means no treatment. Can be controlled by the user according to the specific conditions of the process

## Timer

Timer can be used to create a timed task to run on a periodic basis, or to run only once

### Timer Interface

```cpp
/**
 * @brief Construct a new Timer object
 *
 * @param period The period of the timer, unit is ms
 * @param callback The tasks that the timer needs to perform
 * @param oneshot True: perform the callback only after the first timing cycle
 *                False: perform the callback every timed period
 */
Timer(uint32_t period, std::function<void()> callback, bool oneshot);
```

Or you could encapsulate the parameters into a timer option as follows:

```cpp
struct TimerOption {
  uint32_t period;                 // The period of the timer, unit is ms
  std::function<void()> callback;  // The tasks that the timer needs to perform
  bool oneshot;  // True: perform the callback only after the first timing cycle
                 // False: perform the callback every timed period
};
/**
 * @brief Construct a new Timer object
 *
 * @param opt Timer option
 */
explicit Timer(TimerOption opt);
```

### Start Timer

After creating a Timer instance, you must call `Timer::Start()` to start the timer.

### Stop Timer

When you need to manually stop a timer that has already started, you can call the `Timer::Stop()` interface.

### Demo - example

```cpp
#include <iostream>
#include "cyber/cyber.h"
int main(int argc, char** argv) {
    cyber::Init(argv[0]);
    // Print current time every 100ms
    cyber::Timer timer(100, [](){
        std::cout << cyber::Time::Now() << std::endl;
    }, false);
    timer.Start()
    sleep(1);
    timer.Stop();
}
```

## Time API

Time is a class used to manage time; it can be used for current time acquisition, time-consuming calculation, time conversion, and so on.

The time interfaces are as follows:

```cpp
// constructor, passing in a different value to construct Time
Time(uint64_t nanoseconds); //uint64_t, in nanoseconds
Time(int nanoseconds); // int type, unit: nanoseconds
Time(double seconds); // double, in seconds
Time(uint32_t seconds, uint32_t nanoseconds);
// seconds seconds + nanoseconds nanoseconds
Static Time Now(); // Get the current time
Double ToSecond() const; // convert to seconds
Uint64_t ToNanosecond() const; // Convert to nanoseconds
Std::string ToString() const; // Convert to a string in the format "2018-07-10 20:21:51.123456789"
Bool IsZero() const; // Determine if the time is 0

```

A code example can be seen below:

```cpp
#include <iostream>
#include "cyber/cyber.h"
#include "cyber/duration.h"
int main(int argc, char** argv) {
    cyber::Init(argv[0]);
    Time t1(1531225311123456789UL);
    std::cout << t1.ToString() << std::endl; // 2018-07-10 20:21:51.123456789
    // Duration time interval
    Time t1(100);
    Duration d(200);
    Time t2(300);
    assert(d == (t1-t2)); // true
}

```

## Record file: Read and Write

### Reading the Reader file

**RecordReader** is the component used to read messages in the cyber framework. Each RecordReader can open an existing record file through the `Open` method, and the thread will asynchronously read the information in the record file. The user only needs to execute ReadMessage to extract the latest message in RecordReader, and then get the message information through GetCurrentMessageChannelName, GetCurrentRawMessage, GetCurrentMessageTime.

**RecordWriter** is the component used to record messages in the cyber framework. Each RecordWriter can create a new record file through the Open method. The user only needs to execute WriteMessage and WriteChannel to write message and channel information, and the writing process is asynchronous.

### Demo - example(cyber/examples/record.cc)

Write 100 RawMessage to`TEST_FILE` through `test_write` method, then read them out through `test_read` method.

```cpp
#include <string>

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordWriter;
using ::apollo::cyber::record::RecordMessage;
using apollo::cyber::message::RawMessage;

const char CHANNEL_NAME_1[] = "/test/channel1";
const char CHANNEL_NAME_2[] = "/test/channel2";
const char MESSAGE_TYPE_1[] = "apollo.cyber.proto.Test";
const char MESSAGE_TYPE_2[] = "apollo.cyber.proto.Channel";
const char PROTO_DESC[] = "1234567890";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";

void test_write(const std::string &writefile) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(writefile);
  writer.WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC);
  for (uint32_t i = 0; i < 100; ++i) {
    auto msg = std::make_shared<RawMessage>("abc" + std::to_string(i));
    writer.WriteMessage(CHANNEL_NAME_1, msg, 888 + i);
  }
  writer.Close();
}

void test_read(const std::string &readfile) {
  RecordReader reader(readfile);
  RecordMessage message;
  uint64_t msg_count = reader.GetMessageNumber(CHANNEL_NAME_1);
  AINFO << "MSGTYPE: " << reader.GetMessageType(CHANNEL_NAME_1);
  AINFO << "MSGDESC: " << reader.GetProtoDesc(CHANNEL_NAME_1);

  // read all message
  uint64_t i = 0;
  uint64_t valid = 0;
  for (i = 0; i < msg_count; ++i) {
    if (reader.ReadMessage(&message)) {
      AINFO << "msg[" << i << "]-> "
            << "channel name: " << message.channel_name
            << "; content: " << message.content
            << "; msg time: " << message.time;
      valid++;
    } else {
      AERROR << "read msg[" << i << "] failed";
    }
  }
  AINFO << "static msg=================";
  AINFO << "MSG validmsg:totalcount: " << valid << ":" << msg_count;
}

int main(int argc, char *argv[]) {
  apollo::cyber::Init(argv[0]);
  test_write(TEST_FILE);
  sleep(1);
  test_read(TEST_FILE);
  return 0;
}
```

#### Build and run

- Build: bazel build cyber/examples/…
- Run: ./bazel-bin/cyber/examples/record
- Examining result:

```
I1124 16:56:27.248200 15118 record.cc:64] [record] msg[0]-> channel name: /test/channel1; content: abc0; msg time: 888
I1124 16:56:27.248227 15118 record.cc:64] [record] msg[1]-> channel name: /test/channel1; content: abc1; msg time: 889
I1124 16:56:27.248239 15118 record.cc:64] [record] msg[2]-> channel name: /test/channel1; content: abc2; msg time: 890
I1124 16:56:27.248252 15118 record.cc:64] [record] msg[3]-> channel name: /test/channel1; content: abc3; msg time: 891
I1124 16:56:27.248297 15118 record.cc:64] [record] msg[4]-> channel name: /test/channel1; content: abc4; msg time: 892
I1124 16:56:27.248378 15118 record.cc:64] [record] msg[5]-> channel name: /test/channel1; content: abc5; msg time: 893
...
I1124 16:56:27.250422 15118 record.cc:73] [record] static msg=================
I1124 16:56:27.250434 15118 record.cc:74] [record] MSG validmsg:totalcount: 100:100
```

## API Directory

### Node API

For additional information and examples, refer to [Node](#node)

### API List

```cpp
//create writer with user-define attr and message type
auto CreateWriter(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<transport::Writer<MessageT>>;
//create reader with user-define attr, callback and message type
auto CreateReader(const proto::RoleAttributes& role_attr,
    const croutine::CRoutineFunc<MessageT>& reader_func)
    -> std::shared_ptr<transport::Reader<MessageT>>;
//create writer with specific channel name and message type
auto CreateWriter(const std::string& channel_name)
    -> std::shared_ptr<transport::Writer<MessageT>>;
//create reader with specific channel name, callback and message type
auto CreateReader(const std::string& channel_name,
    const croutine::CRoutineFunc<MessageT>& reader_func)
    -> std::shared_ptr<transport::Reader<MessageT>>;
//create reader with user-define config, callback and message type
auto CreateReader(const ReaderConfig& config,
                  const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<cybertron::Reader<MessageT>>;
//create service with name and specific callback
auto CreateService(const std::string& service_name,
    const typename service::Service<Request, Response>::ServiceCallback& service_calllback)
    -> std::shared_ptr<service::Service<Request, Response>>;
//create client with name to send request to server
auto CreateClient(const std::string& service_name)
    -> std::shared_ptr<service::Client<Request, Response>>;

```

## Writer API

For additional information and examples, refer to [Writer](#writer)

### API List

```cpp
bool Write(const std::shared_ptr<MessageT>& message);
```

## Client API

For additional information and examples, refer to [Client](#service-creation-and-use)

### API List

```cpp
SharedResponse SendRequest(SharedRequest request,
                           const std::chrono::seconds& timeout_s = std::chrono::seconds(5));
SharedResponse SendRequest(const Request& request,
                           const std::chrono::seconds& timeout_s = std::chrono::seconds(5));
```

## Parameter API

The interface that the user uses to perform parameter related operations:

- Set the parameter related API.
- Read the parameter related API.
- Create a ParameterService to provide parameter service related APIs for other nodes.
- Create a ParameterClient that uses the parameters provided by other nodes to service related APIs.

For additional information and examples, refer to [Parameter](##param-parameter-service)

### API List - Setting parameters

```cpp
Parameter();  // Name is empty, type is NOT_SET
explicit Parameter(const Parameter& parameter);
explicit Parameter(const std::string& name);  // Type is NOT_SET
Parameter(const std::string& name, const bool bool_value);
Parameter(const std::string& name, const int int_value);
Parameter(const std::string& name, const int64_t int_value);
Parameter(const std::string& name, const float double_value);
Parameter(const std::string& name, const double double_value);
Parameter(const std::string& name, const std::string& string_value);
Parameter(const std::string& name, const char* string_value);
Parameter(const std::string& name, const std::string& msg_str,
          const std::string& full_name, const std::string& proto_desc);
Parameter(const std::string& name, const google::protobuf::Message& msg);
```

### API List - Reading parameters

```cpp
inline ParamType type() const;
inline std::string TypeName() const;
inline std::string Descriptor() const;
inline const std::string Name() const;
inline bool AsBool() const;
inline int64_t AsInt64() const;
inline double AsDouble() const;
inline const std::string AsString() const;
std::string DebugString() const;
template <typename Type>
typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_integral<Type>::value && !std::is_same<Type, bool>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_floating_point<Type>::value, Type>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_convertible<Type, std::string>::value, const std::string&>::type
value() const;
template <typename Type>
typename std::enable_if<std::is_same<Type, bool>::value, bool>::type
value() const;
```

### API List - Creating parameter service

```cpp
explicit ParameterService(const std::shared_ptr<Node>& node);
void SetParameter(const Parameter& parameter);
bool GetParameter(const std::string& param_name, Parameter* parameter);
bool ListParameters(std::vector<Parameter>* parameters);
```

### API List - Creating parameter client

```cpp
ParameterClient(const std::shared_ptr<Node>& node, const std::string& service_node_name);
bool SetParameter(const Parameter& parameter);
bool GetParameter(const std::string& param_name, Parameter* parameter);
bool ListParameters(std::vector<Parameter>* parameters);
```

## Timer API

You can set the parameters of the Timer and call the start and stop interfaces to start the timer and stop the timer.
For additional information and examples, refer to [Timer](#timer)

### API List

```cpp
Timer(uint32_t period, std::function<void()> callback, bool oneshot);
Timer(TimerOption opt);
void SetTimerOption(TimerOption opt);
void Start();
void Stop();
```

## Time API

For additional information and examples, refer to [Time](#use-of-time)

### API List

```cpp
static const Time MAX;
static const Time MIN;
Time() {}
explicit Time(uint64_t nanoseconds);
explicit Time(int nanoseconds);
explicit Time(double seconds);
Time(uint32_t seconds, uint32_t nanoseconds);
Time(const Time& other);
static Time Now();
static Time MonoTime();
static void SleepUntil(const Time& time);
double ToSecond() const;
uint64_t ToNanosecond() const;
std::string ToString() const;
bool IsZero() const;
```

## Duration API

Interval-related interface, used to indicate the time interval, can be initialized according to the specified nanosecond or second.

### API List

```cpp
Duration() {}
Duration(int64_t nanoseconds);
Duration(int nanoseconds);
Duration(double seconds);
Duration(uint32_t seconds, uint32_t nanoseconds);
Duration(const Rate& rate);
Duration(const Duration& other);
double ToSecond() const;
int64_t ToNanosecond() const;
bool IsZero() const;
void Sleep() const;
```

## Rate API

The frequency interface is generally used to initialize the time of the sleep frequency after the object is initialized according to the specified frequency.

### API List

```cpp
Rate(double frequency);
Rate(uint64_t nanoseconds);
Rate(const Duration&);
void Sleep();
void Reset();
Duration CycleTime() const;
Duration ExpectedCycleTime() const { return expected_cycle_time_; }
```

## RecordReader API

The interface for reading the record file is used to read the message and channel information in the record file.

### API List

```cpp
RecordReader();
bool Open(const std::string& filename, uint64_t begin_time = 0,
          uint64_t end_time = UINT64_MAX);
void Close();
bool ReadMessage();
bool EndOfFile();
const std::string& CurrentMessageChannelName();
std::shared_ptr<RawMessage> CurrentRawMessage();
uint64_t CurrentMessageTime();
```

## RecordWriter API

The interface for writing the record file, used to record the message and channel information into the record file.

### API List

```cpp
RecordWriter();
bool Open(const std::string& file);
void Close();
bool WriteChannel(const std::string& name, const std::string& type,
                  const std::string& proto_desc);
template <typename MessageT>
bool WriteMessage(const std::string& channel_name, const MessageT& message,
                  const uint64_t time_nanosec,
                  const std::string& proto_desc = "");
bool SetSizeOfFileSegmentation(uint64_t size_kilobytes);
bool SetIntervalOfFileSegmentation(uint64_t time_sec);
```
