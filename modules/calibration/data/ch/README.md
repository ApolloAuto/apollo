# we Calibration Configs

In current stage, we only applied automatic GPS waypoint following test(Close Loop) upon WEY, so we only need the following config files, and the others in current folder are used as placeholder.

For a well functioning vehicle, generally you need the following config files:

```text
<vehicle_dir>
  - vehicle_param.pb.txt      # Instance of apollo.common.VehicleParam
  - control_conf.pb.txt       # Instance of apollo.control.ControlConf
  - vehicle_info.pb.txt       # Used for OTA
  - cancard_params            # Params for CAN card
      - canbus_conf.pb.txt
  - gnss_params               # Params for GNSS
      - ant_imu_leverarm.yaml # Config file for leverarm params
  - gnss_conf                 # Params for GNSS
      - gnss_conf.pb.txt      # GNSS config file
```
