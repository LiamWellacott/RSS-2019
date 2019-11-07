#!/usr/bin/env python

import rospy
import rospkg

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np
from particle import Map

import json

import matplotlib.pyplot as plt
from matplotlib import collections as mc


def main():

    with open('pos_est_values.json', 'r') as file:
        data = json.load(file)

        fig, ax = plt.subplots(2,3)

        xs_est = []
        xs_true = []

        ys_est = []
        ys_true = []

        ts_est = []
        ts_true = []

        xs_errs = []
        ys_errs = []
        thetas_errs = []

        for i in range(len(data['Est'])):
            xs_errs.append(data['Est'][i][0] - data['True'][i][0])
            ys_errs.append(data['Est'][i][1] - data['True'][i][1])
            a = data['Est'][i][2] - data['True'][i][2]
            theta = ((a + np.pi) % (2*np.pi)) - np.pi
            thetas_errs.append(theta)

            xs_est.append(data['Est'][i][0])
            xs_true.append(data['True'][i][0])

            ys_est.append(data['Est'][i][1])
            ys_true.append(data['True'][i][1])

            ts_est.append(data['Est'][i][2])
            ts_true.append(data['True'][i][2])

        iterations = range(len(data['Est']))

        ax[0, 0].plot(iterations, xs_errs)
        ax[0, 1].plot(iterations, ys_errs)
        ax[0, 2].plot(iterations, thetas_errs)

        ax[1, 0].plot(iterations, xs_est, c='r')
        ax[1, 1].plot(iterations, ys_est, c='r')
        ax[1, 2].plot(iterations, ts_est, c='r')
        ax[1, 0].plot(iterations, xs_true, c='b')
        ax[1, 1].plot(iterations, ys_true, c='b')
        ax[1, 2].plot(iterations, ts_true, c='b')
        plt.show()




if __name__ == "__main__":
    main()
