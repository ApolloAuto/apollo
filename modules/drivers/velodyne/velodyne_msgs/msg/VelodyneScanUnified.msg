#velodyne lidar scan packets

Header           header         #standard ros message header
uint64          basetime       #time to the top of hour
VelodynePacket[] packets        #vector of raw packets
