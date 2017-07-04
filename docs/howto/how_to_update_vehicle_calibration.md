
# How to Update Vehicle Calibration for Throttle and Brakes

## Introduction
The purpose of vehicle calibration is to find the throttle and brake commands that accurately produce the amount of acceleration requested from the control module.

## Preparation

Preparation consists of the following task sequence:

- Gain access to the relevant code.
- Change the driving mode.
- Select the testing site.

### Gain Access to the Relevant Code
* Canbus, which includes modules for:
  * GPS Driver
  * Localization

### Change the Driving Mode
  Set the driving mode in `modules/canbus/conf/canbus_conf.pb.txt` to `AUTO_SPEED_ONLY`.

### Select the Testing Site
  The preferred testing site is a long flat road.

## Update the Vehicle Calibration

After preparation, complete the following task sequence from `modules/tools/calibration`:

- Collect data.
- Process data.
- Plot results.
- Convert results to `protobuf`.

### Collect Data

 1. Run `python data_collector.py {dir}/{command}.txt` for different commands. Run each command multiple times.
 2. Adjust the command script based on the vehicle response.
 3. Run `python plot_data.py {dir}/{command}.txt_recorded.csv>` to visualize collected data.

### Process Data
Run `process_data.sh` on each recorded log individually. Each data log is processed and appended to `result.csv`.

### Plot Results
Run `python plot_results.py result.csv` to visualize final results. Check for any abnormality.

### Convert Results to `Protobuf`
If everything looks good, run `result2pb.sh` to move calibration results to `protobuf` defined for the control module.
