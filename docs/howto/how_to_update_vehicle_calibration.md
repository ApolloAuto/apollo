
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

After preparation, complete the following task sequence from `modules/tools/vehicle_calibration`:

- Collect data.
- Process data.
- Plot results.
- Convert results to `protobuf`.

### Collect Data

 1. Run `python data_collector.py` for different commands, commands like x y z, where x is acceleration command, y is speed limit(mps), z is decceleration command,Positive number for throttle and negative number for brake.Run each command multiple times.
 2. Adjust the command script based on the vehicle response.
 3. Run `python plot_data.py ` to open recorded data and visualize collected data.

like `15 5.2 -10`, will create and record a file named `t15b-10r0.csv`.

### Process Data
Run `process_data.sh` on each recorded log individually. data log is processed to `t15b-10r0.csv.result`.

### Plot Results
Run `python plot_results.py t15b-10r0.csv.result` to visualize final results. Check for any abnormality.

### Convert Results to `Protobuf`
If everything looks good, run `result2pb.sh` to move calibration results to `protobuf` defined for the control module.
