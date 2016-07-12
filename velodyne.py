#!/usr/bin/python

import os
import csv
import sys
import socket
from datetime import datetime, timedelta
import struct
import time
import numpy as np

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

class Velodyne:
    def __init__(self, metalog=None):
        if metalog is None:
            metalog = MetaLog()
        self.soc = metalog.createLoggedSocket("velodyne", headerFormat="<BBBI")
        self.soc.bind(('', PORT))
        self.metalog = metalog
        self.buf = ""
        self.time = None
        self.last_blocked = None
        self.points = [["Points_m_XYZ:0","Points_m_XYZ:1","Points_m_XYZ:2","intensity","laser_id","azimuth","distance_m","adjustedtime","timestamp","dual_distance","dual_intensity"]]
        self.scan_index = 0
        self.prev_azimuth = None

    def calc(self, dis, azimuth, intensity, laser_id, timestamp):
        R = dis * DISTANCE_RESOLUTION
        omega = LASER_ANGLES[laser_id] * np.pi / 180.0
        alpha = azimuth / 100.0 * np.pi / 180.0
        X = R * np.cos(omega) * np.sin(alpha)
        Y = R * np.cos(omega) * np.cos(alpha)
        Z = R * np.sin(omega)
        return [X, Y, Z, int(intensity), laser_id, azimuth, R, time.time(), timestamp, 0.0, 0.0]

    def write_data(self):
        dirs = 'data'
        file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
        path = datetime.now().strftime(file_fmt)
        try:
            if os.path.exists(path) is False:
                os.makedirs(path)
        except Exception, e:
            print e
        timestamp = '%.6f' % time.time()
        csv_index = '%08d' % self.scan_index
        file_path = "{}/i{}_{}.csv".format(path, csv_index, timestamp)
        with open(file_path, 'w') as fp:
            wr = csv.writer(fp, delimiter=',')
            wr.writerows(self.points)

    def parse(self, data):
        assert len(data) == 1206, len(data)
        timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
        assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
        time = timestamp/1000000.0
        if self.time is not None:
            lost_packets = int(round((time - self.time)/EXPECTED_PACKET_TIME)) - 1
        else:
            lost_packets = 0
        self.time = time
        if lost_packets > 0 and (self.last_blocked is None or self.time > self.last_blocked + EXPECTED_SCAN_DURATION):
            self.last_blocked = self.time + EXPECTED_SCAN_DURATION
            self.scan_index += 1
            print "DROPPED index", self.scan_index
        if self.last_blocked is not None and self.time < self.last_blocked:
            return  # to catch up-to-date packets again ...

        for offset in xrange(0, 1200, 100):
            # flag | azimuth | 0-15 data | 0-15 data
            flag, azimuth = struct.unpack_from("<HH", data, offset)
            assert flag == 0xEEFF, hex(flag)
            for step in xrange(2):
                azimuth += step
                azimuth %= ROTATION_MAX_UNITS
                if self.prev_azimuth is not None and azimuth < self.prev_azimuth:
                    self.write_data();
                    self.scan_index += 1
                    self.points = [["Points_m_XYZ:0","Points_m_XYZ:1","Points_m_XYZ:2","intensity","laser_id","azimuth","distance_m","adjustedtime","timestamp","dual_distance","dual_intensity"]]
                self.prev_azimuth = azimuth
                # H-distance (2mm step), B-reflectivity (0
                arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
                for i in xrange(NUM_LASERS):
                    if arr[i * 2] != 0:
                        self.points.append(self.calc(arr[i * 2], azimuth, arr[i * 2 + 1], i, timestamp))

    def update(self):
        while True:
            data = self.soc.recv(2000)
            if len(data) > 0:
                assert len(data) == 1206, len(data)
                break
        self.parse(data)


if __name__ == "__main__":
    try:
        sensor = Velodyne()
        while True:
            try:
                sensor.update()
            except Exception, e:
                print e
    except KeyboardInterrupt, e:
        print e

# vim: expandtab sw=4 ts=4
