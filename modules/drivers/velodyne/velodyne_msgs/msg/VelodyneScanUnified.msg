#velodyne lidar scan packets

Header           header         #standard ros message header
uint64           basetime       #time to the top of hour, only for VLP16 & HDL32E
VelodynePacket[] packets        #vector of raw packets
