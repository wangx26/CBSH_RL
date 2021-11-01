import csv
import random

with open('map.map','r') as f:
    lists = f.readlines()
    lists = lists[4:]
with open('map.scen','w') as wf:
    i = 0
    while i < 100:
        sx = random.randint(0,78)
        sy = random.randint(0,30)
        gx = random.randint(0,78)
        gy = random.randint(0,30)
        if lists[sy][sx] == '0' and lists[gy][gx] == '1':
            print(sx, sy, gx, gy)
            i += 1
            wf.writelines(['m m m m ', str(sx), ' ', str(sy), ' ', str(gx), ' ', str(gy), '\n'])