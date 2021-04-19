import numpy as np
import sys, time

for i in range(10):
    sys.stdout.write("\rTime remaining: {} Seconds".format(i))
    sys.stdout.flush()
    time.sleep(1)
print '\n'