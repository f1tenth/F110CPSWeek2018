#!/usr/bin/env python

import rospy
import math
import numpy as np
from dino_nav.msg import Stat
import matplotlib.pyplot as plt
import pickle
import sys
import time

seq = 0
start_t = time.time()

def callback(data):
    global seq

    points = []
    angle = data.scan.angle_max + math.pi*3/2
    for d in data.scan.ranges:
        x = math.cos(angle)*d*10
        y = math.sin(angle)*d*10
        points.append((x,y))
        angle -= data.scan.angle_increment

    dim = 100
    vl = dim

    grid = np.zeros([dim, dim], np.bool)
    for p in points:
        x = dim/2 + p[0]
        y = (dim - dim/6) + p[1]
        if(x >= 0 and x <dim and y >=0 and y<dim):
            grid[int(y)][int(x)] = 1

    grd = np.array(grid, np.bool)
    act = np.array( [data.steer, data.throttle] , np.float32)

    fl = open("dataset/data" + str(seq), "w")
    pickle.dump({"bitmap": grd, "actuators": act, "speed": data.speed}, fl)
    fl.close()

    sys.stdout.write("\rDataset entries: %d\t\tdrive time: %d minutes" %  (seq, int( (time.time() - start_t)/60 )) )
    sys.stdout.flush()
    seq += 1

if __name__ == '__main__':
    rospy.init_node('dataset')
    rospy.Subscriber("dinonav/stat", Stat, callback)
    rospy.spin()

