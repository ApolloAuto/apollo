## 模块概述

### 功能概述

Planning 模块在自动驾驶系统中扮演着至关重要的角色，它在遵守交通规则的前提下，将车辆从当前位置沿着路由线路导航到终点位置。

Planning 进行轨迹规划时，需要满足的基础要求有三个：

- 遵守道路交通规则，比如在人行道前停止让行，在交通灯变红时停车，变绿时通行等，

- 对路上遇到的其他交通参与者，如行人，车辆，路边固定障碍物等，不能产生碰撞，

- 规划输出的轨迹，包括路径曲率，速度，加速度等不能超出车辆执行能力边界。

同时，Planning 模块还要考虑在满足基础要求的前提下，尽量提高车辆的舒适性，避免加减速变化过于突兀，方向盘角度变化过快等不舒适的情况。

### 运行流程

planning 模块根据上游模块输入的感 知周围环境信息，地图定位导航信息，以及全局路径信息，为自动驾驶车辆规划出一条运动轨迹（包含坐标，速度，加速度，jerk加加速 度，时间等信息），然后将这些信息传递给控制模块。

![模块综述.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/%E6%A8%A1%E5%9D%97%E7%BB%BC%E8%BF%B0_509d61d.png)

### 模块原理

Apollo 规划模块功能的实现是基于场景（scenario-based）实现的，针对不同的场景，规划模块通过一系列独立的 任务（task） 组合来完成轨迹的规划。开发者可以根据自己的使用需求，调整支持运行的场景列表，或者场景中支持的任务类型来满足自己的需求。

![模块综述-模块原理.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/%E6%A8%A1%E5%9D%97%E7%BB%BC%E8%BF%B0-%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86_2cf19d6.png)

Apollo 规划架构示意图如上，其中部分重要模块如下：

- Public Road Planner：实现了一个有限状态机，结合导航、环境等信息确定自动驾驶车辆的驾驶场景

- Scenario Manager：根据状态机与车辆相关信息，调用合适当前场景的Scenario

- Scenario：结合上游模块信息，通过一系列的任务组合，完成自动驾驶车辆的轨迹规划

- Task组合：一组实现决策和优化任务的 task 集合。优化器用于优化车辆的轨迹和速度。决策器则基于规则，确定自动驾驶车辆何时换车道、何时停车、何时蠕行（慢速行进）或蠕行何时完成等驾驶行为。

### 功能列表

Planning 模块目前支持的功能列表如下：

<table style="undefined;table-layout: fixed; width: 1070px"><thead><tr><th>功能名称</th><th>功能描述</th><th>功能相关代码包</th><th style="width:400px">功能图示</th></tr></thead><tbody><tr><td>lane follow</td><td>车辆沿指令中的路由线路行驶，从地图中查询路由中的车道信息，规划沿车道线行驶的轨迹。</td><td>LaneFollowScenario<br>LaneFollowPath</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%289%29_8433881.png" width="400"></td></tr><tr><td>nudge</td><td>如果道路前方有静止或低速障碍物占据车道，但当前车道内还有足够空间，车辆可以在当前车道内绕过障碍物行驶。</td><td>LaneFollowScenario<br>LaneFollowPath</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2810%29_e312a17.png" alt="image (10).png" width="400"></td></tr><tr><td>lane change</td><td>车辆沿RoutingResponse中的路由线路行驶的过程中，从一个车道切换到相邻车道。</td><td>LaneFollowScenario<br>LaneChangePath</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2811%29_0f2bf4b.png" width="400"></td></tr><tr><td>lane borrow</td><td>如果道路前方有障碍物长时间停留阻塞道路，车辆无法通过在当前车道内绕过，需要往相邻车道借道，绕过当前障碍物。当车辆经过障碍物之后，车辆会立即回到原车道行驶。</td><td>LaneFollowScenario<br>LaneBorrowPath</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2812%29_0efd34a.png" alt="image (12).png" width="400"></td></tr><tr><td>pull over</td><td>当车辆接近终点时，可以通过配置选择是否在终点处靠边停车。如果使能终点靠边停车，车辆在终点附近查找一个可以停车的位置，并将车辆停在这个位置上。如果这个位置前后有其他障碍物，车辆可以通过 OpenSpace 的泊车算法，将车辆停在这个位置。</td><td>PullOverScenario<br>PullOverPath</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_5600a09.png" alt="image.png" width="400"></td></tr><tr><td>park and go</td><td>如果车辆停车位置不在道路上，再次启动的时候，车辆会先从当前位置使用OpenSpace规划算法（如有必要）先行驶到车道线上，然后再正常沿道路行驶。</td><td>ParkAndGoScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%281%29_21cbe23.png" alt="image (1).png" width="400"></td></tr><tr><td>crosswalk</td><td>当车辆行驶到人行道前时，如果有行人通过，车辆会停车等待行人通过后再通行。</td><td>Crosswalk</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%282%29_e65d05f.png" alt="image (2).png" width="400"></td></tr><tr><td>bare intersection</td><td>车辆行驶到无交通灯和停止标志的交通路口，因为对向车辆没有明确通行指示，所以需要车辆根据路口交通情况决定是否通行。</td><td>BareIntersectionUnprotectedScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%283%29_9d8b36e.png" width="400"></td></tr><tr><td>traffic light protected/unprotected</td><td>车辆经过有红绿灯的交通灯路口时，如果交通灯有左转/右转通行箭头，车辆在红灯亮起时停止，绿灯亮起时通行；如果交通灯不是箭头指示灯，车辆在通过路口时可能还有对向车辆经过，这时就需要在通过路口前减速慢行，没有冲突时再通过路口。</td><td>TrafficLight<br>TrafficLightProtectedScenario<br>TrafficLightUnprotectedLeftTurnScenario<br>TrafficLightUnprotectedRightTurnScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%284%29_ad738e7.png" width="400"></td></tr><tr><td>stop sign</td><td>当车辆前方有停止标志时，先停车观察，没有其他行人或车辆冲突时再通行。</td><td>StopSign<br>StopSignUnprotectedScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%285%29_3c11f15.png" alt="image (5).png" width="400"></td></tr><tr><td>yield sign</td><td>当车辆在没有交通灯的路口，有让行标志时，优先让其他对向车辆通行后自车再通行。</td><td>YieldSign<br>YieldSignScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%286%29_68c4693.png" width="400"></td></tr><tr><td>keep clear area</td><td>车辆经过Keep Clear Area区域时，不能在这个区域内停车。</td><td>KeepClear</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%287%29_3e50c6b.png" alt="image (7).png" width="400"></td></tr><tr><td>rerouting</td><td>如果车辆在道路上被阻塞超出一段时间后，planning发出重新路由的请求以便脱困。</td><td>Rerouting</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2833%29_a6d944d.png" alt="image (33).png" width="400"></td></tr><tr><td>valet parking</td><td>给定地图上某一个停车位的id，车辆从当前位置导航到停车位，并泊车入库。</td><td>ValetParkingScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2832%29_d4854af.png" alt="image (32).png" width="400"></td></tr><tr><td>emergency pull over</td><td>在车辆行驶过程中，可以接收外部命令紧急靠边停车。</td><td>EmergencyPullOverScenario</td><td><img src="https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%289%29_9e1c4b9.png" alt="image (9).png" width="400"></td></tr></tbody></table>
