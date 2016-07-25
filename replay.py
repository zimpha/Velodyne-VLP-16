#!/usr/bin/python

"""
    Replay Visualize Velodyne VLP-16 Point Cloud
    usage:
        ./replay.py <timestamp>
"""

import sys
import re
import json
import time
import numpy as np
from datetime import datetime


SCHEDULE_HOST = "http://localhost:10002/"

se = requests.session()

def load_file(file_name):
    global time_temp
    time_temp = dict()
    with open(file_name, 'r') as f:
        line = f.readline()
        while line:
            date_time = re.findall('\d{4}-\d{2}-\d{2}\ \d{2}:\d{2}:\d{2}', line)[0]
            value = re.findall('\[[^\[\]]*\]', line)[3][1:-1]
            timestamp = int(float(re.findall('[0-9]*\.[0-9]*', line)[0]) * 10)
            time_temp[timestamp] = value
            line = f.readline()

def load_data(file_name):
    fp = open(path, 'rb')
    data = np.fromfile(fp, dtype = np.float32)
    fo.close()
    len = x.size // 10
    data = np.reshape(data, [len, 10])
    return data

def send_data(data, timestamp):
    pass
    s.post(SCHEDULE_HOST, data='')

def replay(ts):
    ts = int(ts * 10)
    cnt = 0
    while True:
        if ts in time_temp.keys():
            send_data(load_data(time_temp[ts]), ts)
            cnt = 0
        else:
            cnt += 1
        ts += 1
        if cnt == 10:
            break

if __name__ == '__main__':
    load_file('logs/lidar.log')
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    replay(float(sys.argv[1]))
