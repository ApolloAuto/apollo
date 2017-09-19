###  How to Check the Sensor Output?

Use the `rostopic` command. For example, type the following command to check the
output of HDL-64ES3:

```bash
 rostopic echo /apollo/sensor/velodyne64/VelodyneScanUnified
```

If the topic data is displayed on the terminal, the LiDAR works normally.