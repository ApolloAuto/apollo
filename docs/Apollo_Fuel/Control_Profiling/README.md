# Control Profiling Service

## Overview

Control Profiling Service is a cloud based service to evaluate the control and planning trajectories from road test or simulation records.


## Prerequisites

- [Apollo](https://github.com/ApolloAuto/apollo) 6.0 or higher version.

- Baidu Cloud BOS service registered according to [document](../apply_fuel_account_cn.md)

- Fuel service account on [Apollo Dreamland](http://bce.apollo.auto/user-manual/fuel-service)

## Main Steps

- Data collection

- Job submission

- Results analysis


## Data Collection

### Data Recording

Finish one autonomous driving scenario with a closed loop test, e.g. RTK or close loop.

### Data Sanity Check

- **Make sure the following channels are included in records before submitting them to cloud service**：

    | Modules | channel | items |
    |---|---|---|
    | Canbus | `/apollo/canbus/chassis` | exits without error message |
    | Control | `/apollo/control` | exits without error message |
    | Planning | `/apollo/planning` | - |
    | Localization | `/apollo/localization/pose` | - |
    | GPS | `apollo/sensor/gnss/best_pose` | `sol_type` to `NARROW_INT` |

-  You can check with `cyber_recorder`：

```
    cyber_recorder info xxxxxx.record.xxxxx
```

![](images/profiling_channel_check.png)


## Job Submission

### Upload data to BOS

Here is the folder structure requirements for job submission:
1. A cyber record file containing the execution of open space planner scenario.

1. A configuration file `vehicle_param.pb.txt`; there is a sample file under `apollo/modules/common/data/vehicle_param.pb.txt`.

### Submit job in Dreamland

Go to [Apollo Dreamland](http://bce.apollo.auto/), login with **Baidu** account, choose `Apollo Fuel --> Jobs`，`New Job`, `Control Profiling`，and input the correct BOS path as in [Upload data to BOS](###Upload-data-to-BOS) section：

![control_profiling_submit_job2_en](images/control_profiling_submit_job2_en.png)

![control_profiling_submit_job_en](images/control_profiling_submit_job_en.png)

## Results Analysis

- After job is done, you should be expecting one email per job including `Grading results` and `Visualization results`.

![profiling_grading_results](images/profiling_grading_results.png)

![profiling_visualization_result](images/profiling_visualization_result.png)
