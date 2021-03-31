import numpy as np

msg = 'Hello World_0'
print(msg)
length = len(msg)
position = msg.find('_')

msg = msg[:position]
print(msg)
