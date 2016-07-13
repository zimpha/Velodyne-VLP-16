#!/usr/bin/python

import os
import csv
import sys
import socket
from datetime import datetime, timedelta
import struct
import time
import numpy as np
from multiprocessing import Process, Queue, Pool

import gevent.monkey

gevent.monkey.patch_socket()
import gevent

from apyros.metalog import MetaLog, disableAsserts

HOST = "192.168.1.201"
PORT = 2368

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000
CSV_HEADER = ["Points_m_XYZ:0","Points_m_XYZ:1","Points_m_XYZ:2","intensity","laser_id","azimuth","distance_m","adjustedtime","timestamp"]

MSG_QUEUE = Queue(-1)

def save_data(path, data):
    if not data:
        return
    with open(path, 'w') as fp:
        wr = csv.writer(fp, delimiter=',')
        wr.writerows([CSV_HEADER])
        wr.writerows(data)

def save_process(msg_queue):
    while True:
        if msg_queue.empty():
            pass
        else:
            msg = msg_queue.get()
            save_data(msg['path'], msg['data'])
            print msg['path'], 'queue size: %d' % (msg_queue.qsize())
def calc(dis, azimuth, intensity, laser_id, timestamp):
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z, int(intensity), laser_id, azimuth, R, time.time(), timestamp]

def capture(port, dirs, msg_queue):
    metalog = MetaLog()
    soc = metalog.createLoggedSocket("velodyne", headerFormat="<BBBI")
    soc.bind(('', port))
    points = []
    scan_index = 0
    prev_azimuth = None

    try:
        while True:
            try:
                data = soc.recv(2000)
                if len(data) > 0:
                    assert len(data) == 1206, len(data)
                    timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
                    assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
                    for offset in xrange(0, 1200, 100):
                        flag, azimuth = struct.unpack_from("<HH", data, offset)
                        assert flag == 0xEEFF, hex(flag)
                        for step in xrange(2):
                            azimuth += step
                            azimuth %= ROTATION_MAX_UNITS
                            if prev_azimuth is not None and azimuth < prev_azimuth:
                                file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
                                path = datetime.now().strftime(file_fmt)
                                try:
                                    if os.path.exists(path) is False:
                                        os.makedirs(path)
                                except Exception, e:
                                    print e
                                if not points:
                                    timestamp = '%.6f' % time.time()
                                else:
                                    timestamp = '%.6f' % points[0][7]
                                csv_index = '%08d' % scan_index
                                msg = {'path': "{}/i{}_{}.csv".format(path, csv_index, timestamp), 'data': points}
                                msg_queue.put(msg)
                                scan_index += 1
                                points = []
                            prev_azimuth = azimuth
                            # H-distance (2mm step), B-reflectivity (0
                            arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
                            for i in xrange(NUM_LASERS):
                                if arr[i * 2] != 0:
                                    points.append(calc(arr[i * 2], azimuth, arr[i * 2 + 1], i, timestamp))
            except Exception, e:
                print e
    except KeyboardInterrupt, e:
        print e

if __name__ == "__main__":
    processA = Process(target = capture, args = (PORT, './data', MSG_QUEUE))
    processA.start()
    threads = []
    for i in xrange(4):
        threads.append(gevent.spawn(save_process, MSG_QUEUE))
    gevent.joinall(threads)
