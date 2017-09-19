### How to Check INS Status?

Using Novatel INS as an example, type the following command to check the INS status:  

```bash 
rostopic echo /apollo/sensor/gnss/ins_stat
```

Find the `pos_type` field:  If the value is 56, it has entered a good positioning status (RTK_FIXED) and can be used for calibration. If it is not 56, reliable calibration results cannot be obtained.