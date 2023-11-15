# 红绿灯场景仿真调试

当 apollo 的现有功能和业务逻辑完全适用于您的使用场景时。例如，您只希望点到点行驶调整巡航速度，或者转弯是期望速度更低，您可以不需要开发代码，只通过调整规划模块配置参数实现场景的功能需求。

## 配置参数综述

在 Apollo 的 Planning2.0 中，我们对配置参数进行了重要的更新，以提高系统的灵活性和适用性。以下是更新的主要内容：

- 配置参数拆分：我们对 1.0 版本的配置参数进行了细分，将原本的配置参数拆分成了两大类：全局参数（planning_base）和局部参数（scenarios、tasks、traffic_rules）。全局参数包含了规划模块的基本参数设置，而局部参数则针对不同的场景和任务进行了特定的配置。
- 多份配置参数切换：为了适应不同的应用需求，我们引入了多份配置参数切换方式。用户都可以定义一到多份配置参数，以满足不同的场景和任务要求。

Planning 参数配置分为全局参数和局部参数，全局参数是在 planning 基本流程和基础算法中使用的参数，在`planning_base`文件夹中定义和配置，包含`gflag`和`protobuf`两种文件配置方式；局部参数是在 planning 的插件中定义和配置的，它只作用于所属的插件。

配置参数分类如下：

经过修改后的配置参数文件目录可以分为：全局配置参数和局部配置参数。无论是全局还局部配置参数，它们的配置目录都可以分为两类：一类是采用`proto`文件进行定义，用于定义配置参数的数据结构；另一类是采用`conf`文件进行配置，用于包含实际的配置参数。

![下载.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/%E4%B8%8B%E8%BD%BD_114d038.png)

planning 参数配置的结构如下：

### 全局配置参数

<table><thead><tr><th>全局配置参数</th><th>文件目录结构</th><th>文件</th><th></th></tr></thead><tbody><tr><td rowspan="9">planning_base</td><td rowspan="3">proto</td><td>planning_config.proto</td><td>planning全局配置参数定义</td></tr><tr><td>traffic_rule_config.proto</td><td>traffic_rule生效列表配置定义</td></tr><tr><td>其他基础算法配置参数.proto</td><td>其他算法配置参数定义</td></tr><tr><td rowspan="4">conf</td><td>planning.conf</td><td>全局flag变量</td></tr><tr><td>planning_config.pb.txt</td><td>planning全局配置参数</td></tr><tr><td>traffic_rule_config.pb.txt</td><td>选择traffic_rule生成列表</td></tr><tr><td>其他基础算法配置参数.pb.txt</td><td>其他算法配置参数定义</td></tr><tr><td>common</td><td>planning_gflags.h</td><td>全局flag变量定义</td></tr><tr><td>scenario_base</td><td>proto/scenario_pipeline.proto</td><td>scenaro的pipeline定义</td></tr></tbody></table>

### 局部配置参数 scenarios

<table><thead><tr><th>局部配置参数</th><th>场景</th><th>文件目录结构</th><th></th><th></th><th></th></tr></thead><tbody><tr><td rowspan="11">scenarios</td><td rowspan="6">park_and_go</td><td>proto</td><td>park_and_go.proto</td><td>Scenario:park_and_go scenario的配置参数定义</td><td></td></tr><tr><td rowspan="5">conf</td><td>pipeline.pb.txt</td><td>Scenario Pipeline :park_and_go scenario的PiPeline参数配置</td><td></td></tr><tr><td>scenario_conf.pb.txt</td><td>Scenario :park_and_go scenario的配置参数</td><td></td></tr><tr><td rowspan="2">park_and_go_adjust<br>Stage目录：<br>park_and_go just&nbsp;&nbsp;stage的task配置参数</td><td>open_space_roi_decider.pb.txt</td><td>Task：open_space_roi_decider&nbsp;&nbsp;task的配置参数</td></tr><tr><td>open_space_trajectory_provider.pb.txt</td><td>Task：open_space_trajectory_provider的配置参数</td></tr><tr><td>park_and_go_check<br>Stage目录：park_and_go_check stage的task配置参数</td><td>open_space_roi_decider.pb.txt</td><td>Task：open_space_roi_decider task的配置参数</td></tr><tr><td rowspan="2">lane_follow</td><td>proto</td><td>Scenario :lane_follow的配置参数定义</td><td></td><td></td></tr><tr><td>conf</td><td>Scenario :lane_follow的配置参数</td><td></td><td></td></tr><tr><td rowspan="2">bare_intersection_unprotected</td><td>proto</td><td>Scenario :bare_intersection_unprotected的配置参数定义</td><td></td><td></td></tr><tr><td>conf</td><td>Scenario :bare_intersection_unprotected的配置参数</td><td></td><td></td></tr><tr><td>…</td><td></td><td></td><td></td><td></td></tr></tbody></table>

### 局部配置参数 tasks

<table><thead><tr><th>局部配置参数</th><th>场景</th><th>文件目录结构</th><th></th><th></th></tr></thead><tbody><tr><td rowspan="7">tasks</td><td rowspan="2">lane_borrow_path<br>task默认配置</td><td>proto</td><td>lane_borrow_path.proto</td><td>lane_borrow_path task的配置参数定义</td></tr><tr><td>conf</td><td>default_conf.pb.txt</td><td>lane_borrow_path task的配置参数</td></tr><tr><td rowspan="2">open_space_roi_decider<br>task默认配置</td><td>proto</td><td>open_space_roi_decider.proto</td><td>open_space_roi_decider的配置参数定义</td></tr><tr><td>conf</td><td>default_conf.pb.txt</td><td>open_space_roi_decider的配置参数</td></tr><tr><td rowspan="2">path_decider<br>task默认配置</td><td>proto</td><td>path_decider.proto</td><td>path_decider的配置参数定义</td></tr><tr><td>conf</td><td>default_conf.pb.txt</td><td>path_decider的配置参数</td></tr><tr><td>…</td><td></td><td></td><td></td></tr></tbody></table>

### 局部配置参数 traffic_rules

<table><thead><tr><th>局部配置参数</th><th>场景</th><th>文件目录结构</th><th></th><th></th></tr></thead><table><thead><tr><th>局部配置参数</th><th>场景</th><th>文件目录结构</th><th></th><th></th></tr></thead><tbody><tr><td rowspan="7">traffic_rules</td><td rowspan="2">baskside_vehicle<br>task默认配置</td><td>proto</td><td>backside_vihicle.proto</td><td>backside_vihicle的配置参数定义</td></tr><tr><td>conf</td><td>default_conf.pb.txt</td><td>backside_vihicle的配置参数</td></tr><tr><td rowspan="2">crosswalk<br>traffic_rules默认配置</td><td>proto</td><td>crosswalk.proto</td><td>crosswalk的配置参数定义</td></tr><tr><td>conf</td><td>default_conf.pb.txt</td><td>crosswalk的配置参数</td></tr><tr><td rowspan="2">keepclear<br>traffic_rules默认配置</td><td>proto</td><td>backside_vihicle.proto</td><td>keepclear的配置参数定义</td></tr><tr><td>conf</td><td>keepclearpb.txt</td><td>keepclear的配置参数</td></tr><tr><td>…</td><td></td><td></td><td></td></tr></tbody></table>

## Planning_base 参数配置

Planning_base：全局配置参数。

### 概述

全局配置参数，是指会参与规划上所有内容的参数。这些参数对整个规划过程起着全局性的影响，它们决定了规划模块的基本行为和策略。全局配置参数在整个规划系统中是通用的，不会因为不同的场景或任务而变化，因此它们适用于所有的规划任务。

对应的配置文件在`modules/planning/planning_base/conf/planning.conf`中，
navi模式对应的配置文件在`modules/planning/planning_base/conf/planning_navi.conf`中。

### 配置流程

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_2474d30.png)

开发者如果对配置参数有特定的修改要求，只需输入同步指令，系统将自动将全局配置参数复制到 profile 的 default 目录中，然后就可以在 profile 目录上轻松修改配置参数。

```bash
buildtool profile config init --package planning --profile=default
```

使用 default 目录这份配置

```bash
# 使用名字叫default的这份配置
aem profile use default
```

profile 插件目录结构

```bash
profiles/default/modules/planning/planning_base/
|-- conf
|   |-- discrete_points_smoother_config.pb.txt
|   |-- planner_open_space_config.pb.txt
|   |-- planning.conf
|   |-- planning_config.pb.txt
|   |-- planning_config_navi.pb.txt
|   |-- planning_navi.conf
|   |-- planning_semantic_map_config.pb.txt
|   |-- qp_spline_smoother_config.pb.txt
|   |-- spiral_smoother_config.pb.txt
|   `-- traffic_rule_config.pb.txt
|-- dag
|-- launch
`-- testdata
```

### 实践

全局配置参数对车辆规划的影响：

在这个实践案例中，我们将通过调整不同的配置参数，探索全局配置参数对车辆速度规划、减速带处理以及绕行距离规划的影响。我们将使用 planning2.0 来进行参数配置和规划，并通过 Apollo 的 dreamview+ 来观察车辆规划的效果。

**实践内容**：

- 全局速度配置实践：
  - 目标：调整全局配置参数，使得车辆的最高行驶速度不超过 10m/s。
  - 方法：在 planning2.0 中修改全局速度配置参数。
  - 结果：在 dreamview+ 上实时查看车辆规划的速度情况，验证是否符合要求。
- 减速带配置实践：
  - 目标：调整全局配置参数，使得车辆通过减速带时的最高行驶速度不超过 4m/s。
  - 方法：在 planning2.0 中设定不同的减速带配置参数。
  - 结果：通过 dreamview+ 实时观察车辆的规划路径和速度情况，验证是否满足要求。
- 绕行距离配置实践：
  - 目标：调整全局配置参数，使得车辆在遇到障碍物时与障碍物保持 1.0m 的距离。
  - 方法：在 planning2.0 中改变绕行距离配置参数。
  - 结果：在 dreamview+ 上实时查看车辆绕行的路径和与障碍物的距离，验证是否符合要求。

**实践目的**：

探索全局配置参数对车辆规划的影响。将关注全局速度配置、减速带配置以及绕行距离配置，分别观察车辆规划速度、减速带处理和绕行路径规划的效果。

**实践流程**：

1. 默认配置参数规划效果。

   启动 dreamview 并观察默认配置参数下车辆速度规划、减速带处理以及绕行距离规划的仿真效果。

2. 局速度配置实践：

   a. 找到 planning2.0 中全局速度配置的相关文件。

   b. 根据要求，调整全局速度配置参数，确保车辆的最高行驶速度不超过10m/s。

   c. 保存修改后的配置参数，并重新启动 planning 模块。

   d. 运行车辆规划模块，并在 dreamview+ 上观察车辆规划速度的变化。

3. 减速带配置实践：

   a. 找到 planning2.0 中减速带配置的相关文件。

   b. 设定不同的减速带配置参数，确保车辆通过减速带时的最高行驶速度不超过 4m/s。

   c. 保存修改后的配置参数，并重新编译 planning2.0 模块。

   d. 运行车辆规划模块，并在 dreamview 上实时观察车辆遇到减速带时的通行速度和规划路径。

4. 绕行距离配置实践：

   a. 寻找 planning2.0 中绕行距离配置的相关文件。

   b. 改变绕行距离配置参数，确保车辆在遇到障碍物时与障碍物保持 1.0m 的距离。

   c. 保存修改后的配置参数，并重新编译 planning2.0 模块。

   d. 运行车辆规划模块，并在 dreamview 上实时查看车辆绕行的路径和与障碍物的距离。

#### 默认配置参数规划效果

启动 dreamview 后，选取这三个场景进行仿真依次进行仿真。

![image (11).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2811%29_a2466e7.png)

#### 全局速度配置实践

在全局配置参数中`profiles/default/modules/planning/planning_base/conf/planning.conf`找到该全局速度的配置参数：

```bash
--default_cruise_speed=1.5
```

将配置参数`profiles/default/modules/planning/planning_base/conf/planning.conf`的值修改为：

```bash
--default_cruise_speed=10.0
```

修改之后保存代码后进行仿真。

**实验现象**：

|                  | 原始                                                                                                                    | 修改后                                                                                                                  |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| 全局速度配置实践 | ![image (13).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2813%29_f54be06.png) | ![image (14).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2814%29_537512d.png) |

#### 减速带配置实践

全局配置参数`profiles/default/modules/planning/planning_base/conf/planning.conf`中找到减速带配置参数：

```bash
--speed_bump_speed_limit=1.5
```

将配置参数`profiles/default/modules/planning/planning_base/conf/planning.conf`的值修改为：

```bash
--speed_bump_speed_limit=4.0
```

修改之后保存代码后进行仿真。

**实验现象**：

|                | 原始                                                                                                                    | 修改后                                                                                                                  |
| -------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| 减速带配置实践 | ![image (15).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2815%29_972db7f.png) | ![image (16).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2816%29_7498852.png) |

#### 绕行距离配置实践

全局配置参数`profiles/default/modules/planning/planning_base/planning.conf`中找到减速带配置参数

```bash
--obstacle_lat_buffer=0.3
```

将配置参数`profiles/default/modules/planning/planning_base/planning.conf`的值修改为：

```bash
--obstacle_lat_buffer=1.0
```

修改之后保存代码后进行仿真：

|                | 原始                                                                                                                    | 修改后                                                                                                                  |
| -------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| 减速带配置实践 | ![image (29).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2829%29_96827fd.png) | ![image (30).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2830%29_3ebcb65.png) |

## traffic rules 参数配置

traffic rules：局部配置参数

### 概述

traffic rules 实现了在所有场景下都会生效的规则，用户可以根据自己的需求，决定启用哪些 traffic rules，对应的配置文件在`modules/planning/planning_base/conf/traffic_rule_config.pb.txt`中：

```bash
rule {
  name: "BACKSIDE_VEHICLE"
  type: "BacksideVehicle"
}
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
rule {
  name: "DESTINATION"
  type: "Destination"
}
rule {
  name: "KEEP_CLEAR"
  type: "KeepClear"
}
rule {
  name: "STOP_SIGN"
  type: "StopSign"
}
```

> 注意：rule 中的“name”为场景的别名，用户可以自定义，使用大写字母命名；“type” 是对应场景插件类的类名，省略`namespace apollo::planning`。

对于每一个交通规则，还有其自己的交通规则参数，位于交通规则插件的`conf`目录内。比如人行道规则参数位于`modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt`。

```bash
stop_distance: 1.0
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 2.0
stop_strict_l_distance: 4.0
stop_loose_l_distance: 5.0
start_watch_timer_distance:10
stop_timeout: 10.0
```

### 配置流程

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_4607852.png)

开发者如果对配置参数有特定的修改要求，只需输入同步指令，系统将自动将全局配置参数复制到`profile`的`default`目录中，然后就可以在`profile`目录上修改配置参数。

```bash
buildtool profile config init --package planning-traffic-rules-traffic-light --profile=default
```

查看 profile 插件目录结构：

```bash
tree profiles/default/modules/planning/traffic_rules/
```

目录结构：

```bash
profiles/default/modules/planning/traffic_rules/
`-- traffic_light
    `-- conf
        `-- default_conf.pb.txt
```

### 实践

Traffic Rules 配置参数对车辆规划的影响：

在这个实践案例中，我们将通过调整不同的配置参数，探索 Traffic Rules 配置参数对车辆遇到红绿灯时的停车距离的影响。我们将使用 planning2.0 来进行参数配置和规划，并通过 Apollo 的 Dreamview 来观察车辆规划的效果。

**实践内容：**

局部配置参数交通灯场景配置实践

- 目标：通过调整 Traffic Rules 配置参数，使得主车在监测到前方为红灯时，能够准确停车在距离停止线 2 米至 2.5 米的位置，不得超过停止线。当交通灯变为绿色后，主车能够顺利通过红绿灯路口。
- 方法：在 planning2.0 中找到 Traffic Rules 配置的相关文件，并调整配置参数，以满足红绿灯场景的要求。
- 结果：在 Apollo 的 Dreamview 仿真环境中，观察主车在红绿灯场景下的行驶情况，验证是否能够在红灯时停车在合适位置，并在绿灯后通过红绿灯路口。

**实践目的：**

探索 Traffic Rules 配置参数对车辆规划的影响。观察车辆在不同情况下的停车行为，并探索如何优化交通规则处理模块，以实现更精确和安全的车辆规划。

**实践流程：**

- 默认配置参数规划效果。

  启动 dreamview 并观察默认配置参数下车辆遇到交通灯的仿真效果。

- 红绿灯场景实践。

  a. 找到 planning2.0 中 Traffic Rules 配置的相关文件。

  b. 根据要求，调整 Traffic Rules 配置参数，确保车辆停车在距离停止线 2 米至 2.5 米的位置停车。

  c. 保存修改后的配置参数，并重新启动 planning 模块。

  d. 运行车辆规划模块，并在 dreamview 上观察车辆遇到红绿灯场景的停车变化。

#### 红绿灯场景实践配置参数修改

在 Traffic Rules 参数`profiles/default/modules/planning/planning_base/conf/planning.conf`中找到交通灯停止距离的配置参数：

```bash
stop_distance: 1.0
```

将默认配置参数`profiles/default/modules/planning/planning_base/conf/planning.conf`修改成

```
bash
stop_distance: 2.0
```

**仿真对比：**

|                  | 原始                                                                                                                    | 修改后                                                                                                                  |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| 绕行距离配置实践 | ![image (18).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2818%29_2c431fa.png) | ![image (19).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2819%29_f34c981.png) |

## planning2.0 配置文件新功能

在 Apollo 的 planning2.0 中，为了解决不同场景下可能需要对同一模块使用不同配置参数的问题，我们引入了 profiles 目录，并提供了使用 aem 工具来查看和使用 profile 配置文件的功能。

在 profiles 目录下，用户可以定义一到多份配置参数，以适应不同的应用场景。通过使用 aem 命令，我们可以轻松地查看已有的 profile，并在不同场景下切换使用不同的配置参数。这为我们快速切换配置提供了便利，以验证不同参数对系统性能和效果的影响，从而优化规划模块的表现。

**profile工作原理如下：**

在 planning2.0 中，我们将配置参数分为了两大类，分别为全局配置参数（组件）和局部配置参数（插件）。

在读取配置参数时，其优先级如下：

- 组件：profile > 默认，后续会改为工作空间源码 > profile > 默认
- 插件：工作空间源码 > profile > 默认

**关于profile：**

在默认情况下，profile 目录是空的，不包含任何配置参数，profile 软链会优先使用编译产出后的配置参数。想将其他配置参数添加到 profile 目录中，需要输入同步指令，将配置参数复制到 profile 目录下。
通过执行同步指令，将在 profile 目录中看到与默认配置参数相同的文件。另外，如果希望切换不同的配置参数，可以通过输入切换指令来实现，这会改变软链的指向，从而选择不同的配置文件。

**关于工作空间源码：**

这里所谓的“工作空间源码”指的是工作目录下的 modules 文件夹中包含了该份代码的配置参数。这意味着系统会优先读取工作目录中的源码配置参数，以确保在配置参数的选择过程中具有更高的灵活性和个性化定制性。通过这一机制，我们可以更加灵活地适应不同的应用需求，从而提升系统的适应性和性能。

以下是 aem 指令用于查看和切换配置参数：

```bash
buildtool profile config init
# 查看已有的profile
aem profile list

# 使用名字叫default的这份配置
aem profile use default
```

在 profiles 目录下，用户可以定义一到多份配置参数。这样的好处在于：

- 场景适配性：通过根据不同的场景设置不同的配置参数，我们可以轻松地适应各种应用场景。不同的任务、环境或需求可能需要不同的配置，这样的灵活性能够提高系统的适应性和性能表现。
- 参数保存和复用：在调参过程中，我们可以将认为比较好的参数保存在一个文件夹中。这样，我们不仅能够方便地复用之前优化过的参数，还能够在之后的调参过程中避免遗忘或丢失重要的设置，提高了工作效率。
- 快速回滚：通过保存各个版本的配置文件，如果在后续调整参数时发现新的配置不如之前的好，我们可以轻松地将代码切换回之前保存的较为完美的配置文件。这样能够迅速回滚到之前的状态，避免不必要的错误或性能下降。
- 团队协作：当多个团队成员共同参与项目时，可以为每个成员创建独立的配置文件，便于个人调试和优化，而不会干扰其他人的工作。这种方式有助于提高团队协作效率，各成员可以专注于自己的任务而不用担心影响他人。

### 多份配置参数切换方式

在 Apollo 的 planning2.0 中，我们引入了多份配置参数切换方式，使得根据不同的场景使用不同的配置成为可能，以适应各种应用需求。下面将详细介绍如何设置参数目录：

#### 参数设置目录设置

首先，进入 docker 工作空间，在`/apollo_workspace`路径下输入以下配置参数指令，将配置参数复制到`profile`插件目录中。

注意：默认情况下，配置参数使用的是示例指令，查看已有的 profile 配置指令：

```bash
# 查看已有的profile
aem profile list
```

profile 插件的配置文件目录

```bash
default demo_1 demo_2
```

在这个例子中，通过执行以下指令，我们可以查看当前使用的配置为`default`这份配置文件，并查看`/apollo_workspace` 路径下 profiles 目录的结构：

```bash
tree profiles
```

`/apollo_workspace`路径下 profile 目录结构：

```bash
profiles/
├── current -> default #软链
├── default
├── demo_1
└── demo_2
```

通过使用以下命令，我们可以轻松切换到名为`demo_1`的配置参数：

```bash
# 使用名字叫default的这份配置
aem profile use demo_1
```

> 说明：这样的做法十分便捷，不需要手动处理所有配置文件，只需关注需要修改的部分，系统会智能地处理其余配置，确保我们能够快速调整参数并保持系统正常运行。这样的配置管理方式有效地提高了工作效率和代码的可维护性，让调参过程更加简洁和灵活。
