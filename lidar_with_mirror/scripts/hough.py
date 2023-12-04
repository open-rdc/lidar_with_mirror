#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import math

max_bin = 100

sin_l, cos_l = [], []
for t in np.arange(2*math.pi/max_bin/2, 2*math.pi, 2*math.pi/max_bin):
    sin_l.append(math.sin(t))
    cos_l.append(math.cos(t))
sin_t, cos_t = np.array(sin_l), np.array(cos_l)

def hough(list_x, list_y):
    max_rho = 10
    bin = np.zeros((max_bin, max_bin))
    for x,y in zip(list_x, list_y):
        rho = x * cos_t + y * sin_t
        for t in range(max_bin):
            bin[t][int((rho[t]/max_rho*max_bin+max_bin)/2)] += 1
    print(bin)
    max_angle = bin.argmax() // max_bin
    max_dist = bin.argmax() % max_bin
    print(max_angle, max_dist, bin[max_angle][max_dist])
    print("y="+str(-cos_t[max_angle]/sin_t[max_angle])+"x+"+str(1/sin_t[max_angle]*(max_dist-max_bin/2)/(max_bin/2)*max_rho))

if __name__ == '__main__':
    list_x = np.arange(0, 2, 0.1)
    list_y = 2 * list_x + 2
    print(list_x, list_y)
    hough(list_x, list_y)
