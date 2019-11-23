#!/usr/bin/env python

import matplotlib.pyplot as plt
from utile import Map
import json
import sys
import numpy as np

MAP_FILE = "maps/rss_offset_box1.json"
MAX_LOG = 500
map = Map(MAP_FILE)


def parseFileParticle(file):
    fig, ax = plt.subplots()
    plt.show(block=False)
    with open(file) as json_file:
        data = json.load(json_file)
        r = []
        t = []
        s = []
        m = []
        for i in range(MAX_LOG):
            r.append(data["{}rPose".format(i)])
            t.append(data["{}tPose".format(i)])
            s.append(data["{}tScan".format(i)])
            ray = data["{}rRay".format(i)]
            m_ = np.array(ray).reshape(-1,1)
            tmp = np.copy(m_)
            m_[0] = tmp[0]
            m_[1] = tmp[2]
            m_[2] = tmp[4]
            m_[3] = tmp[6]
            m_[4] = tmp[1]
            m_[5] = tmp[3]
            m_[6] = tmp[5]
            m_[7] = tmp[7]
            m_ = m_.tolist()

            m.append(m_)

            particles = data["{}pPose".format(i)]
            r_ = data["{}rPose".format(i)]
            t_ = data["{}tPose".format(i)]

            if i % 10 == 0:
                ax.clear()
                fig, ax = map.plotMap(fig, ax)

                # Plotting robot position
                ax.scatter(r_[0], r_[1], c='r', s=200)

                ax.quiver(r_[0], r_[1], .00001, .00001, angles=[np.rad2deg(r_[2])], scale=1./10000., scale_units="xy",
                        units="xy", color="k", pivot="mid",width=0.01, headwidth=0.1, headlength=0.1)

                ax.scatter(t_[0], t_[1], c='g', s=200)

                ax.quiver(t_[0], t_[1], .00001, .00001, angles=[np.rad2deg(t_[2])], scale=1./10000., scale_units="xy",
                       units="xy", color="k", pivot="mid",width=0.01, headwidth=0.1, headlength=0.1)


                for p in particles:
                    ax.scatter(p[0], p[1], c='b')
                    ax.quiver(p[0], p[1], .000005, .000005, angles=[np.rad2deg(p[2])], scale=1./10000., scale_units="xy",
                           units="xy", color="y", pivot="mid",width=0.001, headwidth=0.001, headlength=0.00005)
                plt.draw()
                plt.pause(0.01)

    r = np.array(r)
    t = np.array(t)
    m = np.array(m).reshape(-1, 8)
    s = np.array(s)

    diff = m - s

    fig, ax = plt.subplots(2, 2)
    x = range(MAX_LOG)
    ax[0, 0].plot(x, r[:,0], 'r')
    ax[0, 0].plot(x, t[:,0], 'b')

    ax[0, 1].plot(x, r[:,1], 'r')
    ax[0, 1].plot(x, t[:,1], 'b')

    ax[1, 0].plot(x, r[:,2], 'r')
    ax[1, 0].plot(x, t[:,2], 'b')

    fig2, ax2 = plt.subplots(4, 2)
    ax2[0, 0].plot(x, diff[:, 0], 'g')
    ax2[0, 1].plot(x, diff[:, 1], 'g')
    ax2[1, 0].plot(x, diff[:, 2], 'g')
    ax2[1, 1].plot(x, diff[:, 3], 'g')
    ax2[2, 0].plot(x, diff[:, 4], 'g')
    ax2[2, 1].plot(x, diff[:, 5], 'g')
    ax2[3, 0].plot(x, diff[:, 6], 'g')
    ax2[3, 1].plot(x, diff[:, 7], 'g')

    '''
    ax2[0, 0].plot(x, m[:, 0], 'r')
    ax2[0, 0].plot(x, s[:, 0], 'b')

    ax2[0, 1].plot(x, m[:, 1], 'r')
    ax2[0, 1].plot(x, s[:, 1], 'b')

    ax2[1, 0].plot(x, m[:, 2], 'r')
    ax2[1, 0].plot(x, s[:, 2], 'b')

    ax2[1, 1].plot(x, m[:, 3], 'r')
    ax2[1, 1].plot(x, s[:, 3], 'b')

    ax2[2, 0].plot(x, m[:, 4], 'r')
    ax2[2, 0].plot(x, s[:, 4], 'b')

    ax2[2, 1].plot(x, m[:, 5], 'r')
    ax2[2, 1].plot(x, s[:, 5], 'b')

    ax2[3, 0].plot(x, m[:, 6], 'r')
    ax2[3, 0].plot(x, s[:, 6], 'b')

    ax2[3, 1].plot(x, m[:, 7], 'r')
    ax2[3, 1].plot(x, s[:, 7], 'b')
    '''
    plt.show()

def parseFileOdom(file):
    before = []
    after = []
    with open(file) as json_file:
        data = json.load(json_file)
        for i in range(MAX_LOG):
            before.append(data["{}vel1".format(i)])
            after.append(data["{}vel2".format(i)])
    before = np.array(before)
    after = np.array(after)
    fig, ax = plt.subplots(2)
    x = range(MAX_LOG)
    ax[0].plot(x, before[:, 0], 'r')
    ax[0].plot(x, after[:, 0], 'b')
    ax[1].plot(x, before[:, 1], 'r')
    ax[1].plot(x, after[:, 1], 'b')
    plt.show()

def main():
    json_file = sys.argv[1]
    #parseFileOdom(json_file)
    parseFileParticle(json_file)

if __name__ == '__main__':
    main()
