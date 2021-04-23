import numpy as np
import sys, os, time
from routing_pub import talker as rtalker

for i in range(10):
    print('hello, %d' % i)
    time.sleep(1.0)
    if (i == 5):
        os._exit(1)
