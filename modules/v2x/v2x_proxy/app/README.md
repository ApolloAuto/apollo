# How to use HIK OBU device
1. make HDmap. 
the signal id should be named as format (node_id)_(lightid) like 1_16, which means that 1 is junction node_id and 16 is light_id(value 1~16)
node_id and light_id is configured in RSU
node id describes which junction
light id from 1 to 16 describes light phase :
   1 East LEFT TURN
   2 East STRAIGHT FORWARD
   3 East RIGHT TURN
   4 East PAVEMENT
   5 South LEFT TURN
   6 South STRAIGHT FORWARD
   7 South RIGHT TURN
   8 South PAVEMENT
   9 West LEFT TURN
   10 West STRAIGHT FORWARD
   11 West RIGHT TURN
   12 West PAVEMENT
   13 North LEFT TURN
   14 North STRAIGHT FORWARD
   15 North RIGHT TURN
   16 North PAVEMENT
2.  configure ip and port 
you need to confiure 
        `--obu_host_ip` as obu ip
        `--obu_host_port` as obu listening udp message port
        `--local_host_port` as obu sending message to IPC port
        `--local_utm_zone_id` as your location UTM zone id.
3. configure proxy
you need to modify v2x_config.pb.txt to 
`v2x_device: HIK`
4. start v2x
In Dreamview choose hmi mode to Dev Kit Debug, click v2x to start.