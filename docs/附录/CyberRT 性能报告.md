## 介绍

本性能报告旨在对比新旧版本 CyberRT 在不同传输条件下的跨进程 / 跨机传输时各性能指标的结果，并且列举出在使用不同模块 / 不同传感器（业务模块、普通规格传感器、高规格传感器、超高规格传感器）时有关传输的各个指标的详细数据。本次测试的所有结果均由 cyber_benchmark 基准测试工具生成。

## 测试环境

### 平台 1:

<table>
  <tbody>
    <tr>
      <td rowspan="3">硬件环境</td>
      <td>cpu</td>
      <td>Intel(R) Core(TM) i9-9900K</td>
    </tr>
    <tr>
      <td>内存</td>
      <td>2 x Innodisk M4S0-AGS1OCIK DDR4 16GiB 2667 MHz</td>
    </tr>
    <tr>
      <td>硬盘</td>
      <td>Samsung SSD 980 500GB</td>
    </tr>
    <tr>
      <td rowspan="3">软件环境</td>
      <td>系统版本</td>
      <td>Ubuntu 18.04.5 2021.09.12 LTS</td>
    </tr>
    <tr>
      <td>内核版本</td>
      <td>5.4.0-150-generic</td>
    </tr>
    <tr>
      <td>CyberRT 版本</td>
      <td>Apollo 9.0 / Apollo 10.0</td>
    </tr>
  </tbody>
</table>

### 平台 2:

<table>
  <tbody>
    <tr>
      <td rowspan="3">硬件环境</td>
      <td>cpu</td>
      <td>8 core Arm® Cortex®-A78AE v8.2</td>
    </tr>
    <tr>
      <td>内存</td>
      <td>32GiB 256 bit LPDDR5 Onboard Memory 204.8GB/s</td>
    </tr>
    <tr>
      <td>硬盘</td>
      <td>KINGSTON OM8PGP41024Q-A0</td>
    </tr>
    <tr>
      <td rowspan="3">软件环境</td>
      <td>系统版本</td>
      <td>Ubuntu 20.04.4 LTS</td>
    </tr>
    <tr>
      <td>内核版本</td>
      <td>5.10.104-tegra</td>
    </tr>
    <tr>
      <td>CyberRT 版本</td>
      <td>Apollo 9.0 / Apollo 10.0</td>
    </tr>
  </tbody>
</table>

## 跨进程传输性能测试结果

本测试将会测试新旧版本的 CyberRT 在不同消息大小以及不同发送频率下的各项性能指标

### 不同消息大小

所有测试都是在消息大小为 16B、1KB、64KB......5MB、10MB，以100 hz的频率进行的

下列图片展示了在平台1上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 开启了 arena 零拷贝通信，arena 共享内存配置了 1GB 大小：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_1.png" alt="cyber_performance_report_1" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_2.png" alt="cyber_performance_report_2" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_3.png" alt="cyber_performance_report_3" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_4.png" alt="cyber_performance_report_4" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

下列图片展示了在平台2上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 开启了 arena 零拷贝通信，arena 共享内存配置了 1GB 大小：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_5.png" alt="cyber_performance_report_5" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_6.png" alt="cyber_performance_report_6" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_7.png" alt="cyber_performance_report_7" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_8.png" alt="cyber_performance_report_8" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

### 不同发送频率

所有测试都是在消息频率为 10 hz、20 hz、50 hz、100 hz，以1MB的消息大小进行的

下列图片展示了在平台1上传输不同发送频率的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 开启了 arena 零拷贝通信，arena 共享内存配置了 1GB 大小：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_9.png" alt="cyber_performance_report_9" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_10.png" alt="cyber_performance_report_10" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_11.png" alt="cyber_performance_report_11" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_12.png" alt="cyber_performance_report_12" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

下列图片展示了在平台2上传输不同发送频率的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 开启了 arena 零拷贝通信，arena 共享内存配置了 1GB 大小：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_13.png" alt="cyber_performance_report_13" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_14.png" alt="cyber_performance_report_14" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_15.png" alt="cyber_performance_report_15" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_16.png" alt="cyber_performance_report_16" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

### Apollo 10.0 CyberRT 在不同场景下在平台1下跨进程传输的具体表现

|                                                  | message size/frequency | cpu usage | latency  | msg loss rate | memory usage                     |
| ------------------------------------------------ | ---------------------- | --------- | -------- | ------------- | -------------------------------- |
| Functional Module(perception, planning etc.)     | 64K/10hz               | 9.14%     | 84.6 us  | 0.0%          | 250MB                            |
| High Frequencies functional Module（localization） | 64k/100hz              | 9.71%     | 69.54 us | 0.0%          | 250M + 1024M arena shared memory |
| Normal Sensor Module                             | 1M/10hz                | 8.47%     | 82.29 us | 0.0%          | 250M + 1024M arena shared memory |
| High-End Sensor Module                           | 10M/10hz               | 5.55%     | 58.95 us | 0.0%          | 250M + 1024M arena shared memory |

## 跨机传输性能测试结果

本测试将会测试新旧版本的 CyberRT 在不同消息大小以及不同发送频率下的各项性能指标

### 不同消息大小

所有测试都是在消息大小为 1KB、64KB......5MB、10MB，以100 hz的频率进行的

下列图片展示了在平台1上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 基于 2.x版本的 FastDDS，9.0 的 CyberRT 基于1.5版本的 fastrtps：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_17.png" alt="cyber_performance_report_17" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_18.png" alt="cyber_performance_report_18" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_19.png" alt="cyber_performance_report_19" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_20.png" alt="cyber_performance_report_20" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

下列图片展示了在平台2上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 基于 2.x版本的 FastDDS，9.0 的 CyberRT 基于1.5版本的 fastrtps：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_21.png" alt="cyber_performance_report_21" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_22.png" alt="cyber_performance_report_22" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_23.png" alt="cyber_performance_report_23" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_24.png" alt="cyber_performance_report_24" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

### 不同发送频率

所有测试都是在消息频率为 10 hz、20 hz、50 hz、100 hz，以1MB的消息大小进行的

下列图片展示了在平台1上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 基于 2.x版本的 FastDDS，9.0 的 CyberRT 基于1.5版本的 fastrtps：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_25.png" alt="cyber_performance_report_25" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_26.png" alt="cyber_performance_report_26" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_27.png" alt="cyber_performance_report_27" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_28.png" alt="cyber_performance_report_28" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

下列图片展示了在平台1上传输不同消息大小的cpu占用、内存占用、消息传输时延、丢包率指标。其中，10.0 的 CyberRT 基于 2.x版本的 FastDDS，9.0 的 CyberRT 基于1.5版本的 fastrtps：

<table>
  <tbody>
    <tr>
      <td><img src="./images/cyber_performance_report_29.png" alt="cyber_performance_report_29" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_30.png" alt="cyber_performance_report_30" style="zoom:50%;" /></td>
    </tr>
    <tr>
      <td><img src="./images/cyber_performance_report_31.png" alt="cyber_performance_report_31" style="zoom:50%;" /></td>
      <td><img src="./images/cyber_performance_report_32.png" alt="cyber_performance_report_32" style="zoom:50%;" /></td>
    </tr>
  </tbody>
</table>

### Apollo 10.0 CyberRT 在不同场景下在平台1下跨机传输的具体表现

|                                                  | message size/frequency | cpu usage | latency  | msg loss rate | memory usage |
| ------------------------------------------------ | ---------------------- | --------- | -------- | ------------- | ------------ |
| Functional Module(perception, planning etc.)     | 64K/10hz               | 7.75%     | 391 us   | 0.0%          | 247MB        |
| High Frequencies functional Module（localization） | 64k/100hz              | 10.3%     | 369 us   | 0.0%          | 249M         |
| Normal Sensor Module                             | 1M/10hz                | 8.84%     | 2124 us  | 0.0%          | 251M         |
| High-End Sensor Module                           | 10M/10hz               | 18.8%     | 18886 us | 0.0%          | 288M         |
