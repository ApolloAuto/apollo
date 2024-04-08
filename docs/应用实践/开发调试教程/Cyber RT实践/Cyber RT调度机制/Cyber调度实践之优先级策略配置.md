## 实验内容

Apollo Cyber 是首个专为自动驾驶定制的高性能开源的运行时框架，它主要解决了自动驾驶系统的高并发、低延迟、高吞吐、任务调度等问题，同时还提供了多种通信机制和用户级的协程，在资源有限的情况下会根据任务的优先级来进行调度处理。本实验将讲述任务的优先级配置和调度。

## 体验地址

https://apollo.baidu.com/community/course/41#

## 实验目的

掌握 Apollo 如何设置 Cyber 的调度策略，任务优先级配置。

## 实验流程

本实验涉及到的目录结构如下：

```bash
/apollo
|-- cyber
|    | -- conf
|         | -- example_sched_choreography.conf       -----------task配置文件
|    | -- examples
|         | -- common_component_example
|              | -- common.dag                       -----------启动的dag文件
|              | -- common.launch                    -----------启动的launch文件
|-- data
|   |-- log
|       |-- mainboard.INFO                           -----------log文件
|-- modules
|-- scripts
```

1. 修改`/apollo/cyber/conf/example_sched_choreography.conf`，通过更改配置可以设置任务的优先级。这里面设置了 common0，common1，common2，common3 四个任务，以及对应的优先级，tasks：这里是对 task 任务进行配置，name 表示 task 的名字，prio 表示任务的优先级，值越大表明优先级越高，这四个任务的优先级顺序，从高到低依次为 common3，common2，common1，common0。

   ```bash
   scheduler_conf {
    policy: "choreography"
    process_level_cpuset: "0"  # all threads in the process are on the cpuset

    choreography_conf {
        choreography_processor_num: 4
        choreography_affinity: "range"
        choreography_cpuset: "0"
        choreography_processor_policy: "SCHED_FIFO" # policy: SCHED_OTHER,SCHED_RR,SCHED_FIFO
        choreography_processor_prio: 10

        pool_processor_num: 4
        pool_affinity: "range"
        pool_cpuset: "0"
        pool_processor_policy: "SCHED_OTHER"
        pool_processor_prio: 0

        tasks: [
            {
                name: "common0"
                processor: 0
                prio: 1
            },
            {
                name: "common1"
                processor: 0
                prio: 2
            },
            {
                name: "common2"
                processor: 0
                prio: 3
            },
            {
                name: "common3"
                processor: 0
                prio: 4
            }
        ]
    }
   }
   ```

2. 在`/apollo/cyber/examples/common_component_example`路径下新建 common0.dag，common1.dag，common2.dag，common3.dag，修改common.launch。

   命令为：

   ```bash
   touch common0.dag common1.dag common2.dag common3.dag
   ```

   四个文件的内容分别如下：

   ```bash
    # Define all coms in DAG streaming.
    module_config {
    module_library : "cyber/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common0"
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

   ```bash
    # Define all coms in DAG streaming.
    module_config {
    module_library : "cyber/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common1"
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

   ```bash
   # Define all coms in DAG streaming.
    module_config {
    module_library : "cyber/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common2"
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

   ```bash
   # Define all coms in DAG streaming.
    module_config {
    module_library : "cyber/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common3"
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

   common.launch 配置如下：

   ```bash
   <cyber>
    <module>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common0.dag</dag_conf>
        <dag_conf>/apollo/cyber/examples/common_component_example/common1.dag</dag_conf>
        <dag_conf>/apollo/cyber/examples/common_component_example/common2.dag</dag_conf>
        <dag_conf>/apollo/cyber/examples/common_component_example/common3.dag</dag_conf>
        <process_name>example_sched_choreography</process_name>
    </module>
   </cyber>
   ```

3. 执行：

   ```bash
   channel_prediction_writer &
   channel_test_writer &
   ```

   往 channel /apollo/prediction 和 /apollo/test 发送消息。

4. 启动

   ```bash
   cyber_launch start /apollo/cyber/examples/common_component_example/common.launch
   ```

   用于消费 channel 中的消息。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7874cef.png)

5. 新打开一个 Terminal，进入`/apollo/data/log`目录中，查看 mainboard.INFO 日志，可以看到任务的执行顺序，依次是 common3，common2，common1，common0，符合预期。

   ```bash
   cd /apollo/data/log
   ll
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_08e9f94.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4632854.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_37aa354.png)

   至此，实验结束。
