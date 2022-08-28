#! /usr/bin/python3

import csv
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
from scipy.spatial.transform import Rotation

data = [[]]
fig, ax = plt.subplots()

with open("scan.csv", 'r') as fr:
    reader = csv.reader(fr)
    data = [row for row in reader]
    #data = data0[0:1] + data0[240:260]
    index_angle_min = data[0].index("field.angle_min")
    index_angle_increment = data[0].index("field.angle_increment")
    index_ranges0 = data[0].index("field.ranges0")
    num = len(data[0]) - index_ranges0

def angle_to_index(angle_rad):
    row = data[1]
    angle_min = float(row[index_angle_min])
    angle_increment = float(row[index_angle_increment])
    return int((angle_rad - angle_min) / angle_increment)

def dist_to_line(x, y, a, b):
    return abs(a * x - y + b) / math.sqrt(a ** 2 + 1)

def obstacle_detect(list_x, list_y, a, b, threshold):
    res_x, res_y = [], []
    for x, y in zip(list_x, list_y):
        if dist_to_line(x, y, a, b) > threshold and math.sqrt(x**2 + y**2) > 0.1:
            res_x.append(x)
            res_y.append(y)
    if len(res_x) >= 4:
        del res_x[0:1]
        del res_y[0:1]
        del res_x[-1]
        del res_x[-1]
        del res_y[-1]
        del res_y[-1]
    return res_x, res_y

def line_detect(list_x, list_y, a, b, threshold):
    res_x, res_y = [], []
    for x, y in zip(list_x, list_y):
        if dist_to_line(x, y, a, b) < threshold:
            res_x.append(x)
            res_y.append(y)
    return res_x, res_y

def update(num):
    row = data[num+1]
    angle = float(row[index_angle_min])
    angle_increment = float(row[index_angle_increment])
    list_x, list_y, list_angle = [], [], []
    length = row[index_ranges0:]

    for value in length:
        x = float(value) * math.cos(angle)
        y = float(value) * math.sin(angle)
        angle += angle_increment
        list_x.append(x)
        list_y.append(y)
        list_angle.append(angle)

    front_x = list_x[angle_to_index(-0.7):angle_to_index(0.7)]
    front_y = list_y[angle_to_index(-0.7):angle_to_index(0.7)]
    n = angle_to_index(0.7) - angle_to_index(-0.7)
    na = int(0.1/angle_increment)
    front_a, front_b = np.polyfit(front_x[:na]+front_x[n-na:n], front_y[:na]+front_y[n-na:n], 1)
    front_fit_x = np.array([-3, 3])
    front_fit_y = front_a * front_fit_x + front_b
    
    front_obs_x, front_obs_y = obstacle_detect(front_x, front_y, front_a, front_b, 0.1)
    front_obs_ave_x, front_obs_ave_y = np.mean(front_obs_x), np.mean(front_obs_y)
    front_obs_b = front_obs_ave_y - front_a * front_obs_ave_x
    front_obs_fit_y = front_a * front_fit_x + front_obs_b

    pitch_angle = math.asin(-0.102*front_a/(front_b-front_obs_b))
    roll_angle = math.asin(math.tan(pitch_angle)/front_a)
    z = front_b*math.sin(roll_angle)*math.cos(pitch_angle)

    euler = np.array([pitch_angle, roll_angle, 0])
    rot = Rotation.from_euler('YXZ', euler)
    #obs_height = []
    #for x, y in zip(front_obs_x, front_obs_y):
    #    vec = np.array([[x],[y],[0]])
    #    pos = rot.as_dcm().dot(vec)
    #    obs_height.append(pos[2][0]-z)
    height = []
    #for x, y in zip(front_x[:na]+front_x[n-na:n], front_y[:na]+front_y[n-na:n]):
    #    vec = np.array([[x],[y],[0]])
    #    pos = rot.as_dcm().dot(vec)
    #    height.append(pos[2][0]-z)

    for x, y in zip(front_x, front_y):
        vec = np.array([[x],[y],[0]])
        pos = rot.as_dcm().dot(vec)
        height.append(pos[2][0]-z)

    #print(str(num+1)+", "+str(z)+", "+str(np.mean(obs_height))+", "+str(np.mean(height))+", "+str(np.mean(obs_height)-np.mean(height)))

    #ax.clear()
    #ax.grid(True)
    #ax.set_xlim(0, 300)
    #ax.set_ylim(-0.2, 0.2)
    #ax.plot(height, color='black')
    #ax.plot(obs_height, color='red')

    right_x = list_x[angle_to_index(-1.8):angle_to_index(-1.1)]
    right_y = list_y[angle_to_index(-1.8):angle_to_index(-1.1)]

    n = angle_to_index(-1.1) - angle_to_index(-1.8)
    na = int(0.1/angle_increment)
    right_a0, right_b0 = np.polyfit(right_x[:na]+right_x[n-2:n], right_y[:na]+right_y[n-2:n], 1)
    right_line_x, right_line_y = line_detect(right_x, right_y, right_a0, right_b0, 0.01)
    right_a, right_b = np.polyfit(right_line_x, right_line_y, 1)
    right_fit_x = np.array([-3, 3])
    right_fit_y = right_a * right_fit_x + right_b

    right_pitch_angle = math.asin(right_a*z/right_b)
    right_roll_angle = math.asin(math.tan(right_pitch_angle)/right_a)
    right_euler = np.array([right_pitch_angle, right_roll_angle, 0])
    right_rot = Rotation.from_euler('YXZ', right_euler)

    right_height = []
    for x, y in zip(right_line_x, right_line_y):
        vec = np.array([[x],[y],[0]])
        pos = right_rot.as_dcm().dot(vec)
        right_height.append(pos[2][0] - z)

    #ax.clear()
    #ax.grid(True)
    #ax.set_xlim(0, 150)
    #ax.set_ylim(-0.2, 0.2)
    #ax.plot(right_height, color='red')

    left_x = list_x[angle_to_index(1.2):angle_to_index(2.0)]
    left_y = list_y[angle_to_index(1.2):angle_to_index(2.0)]

    n = angle_to_index(2.0) - angle_to_index(1.2)
    na = int(0.1/angle_increment)
    left_a0, left_b0 = np.polyfit(left_x[:2]+left_x[n-na:n], left_y[:2]+left_y[n-na:n], 1)
    left_line_x, left_line_y = line_detect(left_x, left_y, left_a0, left_b0, 0.01)
    left_a, left_b = np.polyfit(left_line_x, left_line_y, 1)
    left_fit_x = np.array([-3, 3])
    left_fit_y = left_a * left_fit_x + left_b

    left_pitch_angle = math.asin(left_a*z/left_b)
    left_roll_angle = math.asin(math.tan(left_pitch_angle)/left_a)
    #print(str(num+1)+", "+str(z)+", "+str(left_a)+", "+str(left_b)+", "+str(right_a)+", "+str(right_b)+", "+str(pitch_angle)+", "+str(roll_angle))
    left_euler = np.array([left_pitch_angle, left_roll_angle, 0])
    left_rot = Rotation.from_euler('YXZ', left_euler)
    left_height = []
    for x, y in zip(left_line_x, left_line_y):
        vec = np.array([[x],[y],[0]])
        pos = left_rot.as_dcm().dot(vec)
        left_height.append(z - pos[2][0])

    #ax.clear()
    #ax.grid(True)
    #ax.set_xlim(0, 100)
    #ax.set_ylim(-0.2, 0.2)
    #ax.plot(left_height, color='red')

    ax.clear()
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.plot(list_x, list_y, ".", color='lightgray')
    ax.plot(front_x, front_y, ".", color='red')
    ax.plot(front_fit_x, front_fit_y, color='red')
    ax.plot(front_obs_x, front_obs_y, ".", color='black')
    ax.plot(front_fit_x, front_obs_fit_y, color='black')
    ax.plot(right_line_x, right_line_y, ".", color='green')
    ax.plot(right_fit_x, right_fit_y, color='green')
    ax.plot(left_line_x, left_line_y, ".", color='blue')
    ax.plot(left_fit_x, left_fit_y, color='blue')

ani = animation.FuncAnimation(fig, update, len(data)-1, interval=10)
plt.show()
