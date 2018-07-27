# -*- coding:utf-8 -*- 
################################################################
# notice:
#   (1) this tool need scapy, to install : sudo apt-get install scapy
#   (2) replay data need root, so run it in sudo mode
#   (3) send pkt from diff eth or diff machine
#   (4) modify velody poll timeout param 
###############################################################
import time
import os
from scapy.all import *

def get_local_ip():
    c = 'hostname -I'
    r = os.popen(c)
    for i in r:
        return i.split(" ")[0]

def replay_pcap(file, dest_ip):
    count = 0
    pkt = rdpcap(file)
    for i in pkt:
        ip_s = get_local_ip()
        ip_d = dest_ip
        port_s = i[UDP].sport
        port_d = i[UDP].dport
        data = i[Raw].load
            
        p = IP(src = ip_s, dst = ip_d)/UDP(sport = port_s, dport = port_d)/data
        send(p)

        count += 1
        print 'replay packet index: %d' % count
    return count

if __name__ == '__main__': 
    file = "data.pcap"
    dest_ip = "192.168.10.255"
    c = replay_pcap(file, dest_ip)
    print 'total replay packet : %d' % c
