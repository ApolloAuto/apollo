## How to troubleshoot ESD CAN device?


### Problem

Can’t communicate through ESD CAN card.

### Troubleshooting Steps:
1. Make sure CAN driver (kernel module) is loaded, run: ```lsmod |grep can```; you should see information regarding the kernel driver such as version number if CAN driver has been loaded.
2. Make sure CAN device is present and has the right permission set, run: ```ls -l /dev/can0```.
3. Check kernel log (run ```dmesg |grep -i can```) and syslog (run ```grep -i can /var/log/syslog```), see if there are error messages related to CAN.
4. Run the Apollo program ```esdcan_test_app``` (under ```monitor/hwmonitor/hw/tools/```), which will print out detailed stats and status information.
    * To learn about this tool, run ```esdcan_test_app --help```.
    * To print additional detailed stats, run ```sudo esdcan_test_app --details=true``.
5. [Optional] Save the kernel log, syslog (Step 4) and output from Step 5 for offline analysis.
6. If necessary, check system ambient temperature. ESD CAN card (CAN-PCIe/402-FD) has a working temperature range of 0-75 degree Celsius; it may not work outside of this temperature range. You may also attach a temperature sensor to the surface of the main IC chip on ESD CAN (an Altera FPGA chip) to monitor the surface temperature of the chip to make sure it is not overheating.
