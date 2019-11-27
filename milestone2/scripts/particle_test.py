#!/usr/bin/env python

import matplotlib.pyplot as plt
from utile import Map
import json
import sys
import numpy as np
import scipy.signal as sig

MAP_FILE = "maps/rss_offset.json"
MAX_LOG = 1000
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
        path_np = None
        if "0path" in data.keys():
            path_np = np.array(data["0path"])
        for i in range(1, 5):
            if "{}path".format(i) in data.keys():
                path_np = np.vstack((path_np, np.array(data["{}path".format(i)])))

        for i in np.arange(1, MAX_LOG):
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


            if i % 20 == 0:

                ax.clear()
                fig, ax = map.plotMap(fig, ax)
                r__ = np.array(r)
                ax.plot(r__[:, 0], r__[:, 1], 'k', label="True position", linewidth=2)
                # Plotting robot position
                ax.scatter(r_[0], r_[1], c='r', s=200)

                ax.quiver(r_[0], r_[1], .00001, .00001, angles=[np.rad2deg(r_[2])], scale=1./10000., scale_units="xy",
                        units="xy", color="k", pivot="mid",width=0.01, headwidth=0.1, headlength=0.1)

                ax.scatter(t_[0], t_[1], c='g', s=200)

                ax.quiver(t_[0], t_[1], .00001, .00001, angles=[np.rad2deg(t_[2])], scale=1./10000., scale_units="xy",
                       units="xy", color="k", pivot="mid",width=0.01, headwidth=0.1, headlength=0.1)

                '''
                for p in particles:
                    ax.scatter(p[0], p[1], c='b')
                    ax.quiver(p[0], p[1], .000005, .000005, angles=[np.rad2deg(p[2])], scale=1./10000., scale_units="xy",
                           units="xy", color="y", pivot="mid",width=0.001, headwidth=0.001, headlength=0.00005)
                '''
                #for p in path:
                    #p = np.array(p)
                if path_np is not None:
                    ax.plot(path_np[:, 0], path_np[:, 1], c='g', label="trajectory", linewidth=2)
                ax.legend()
                plt.title("True position of the robot w.r.t the planned trajectory")
                plt.draw()
                plt.pause(0.01)

    r = np.array(r)
    t = np.array(t)
    #r[:, 2] = ((r[:, 2] + np.pi) % (2*np.pi)) - np.pi
    #t[:, 2] = ((t[:, 2] + np.pi) % (2*np.pi)) - np.pi
    m = np.array(m).reshape(-1, 8)
    s = np.array(s)
    #print(p.shape)

    #fs = 5.
    #cutoff = 0.2
    #order = 5.

    #nyquist = fs / 2.

    #b, a = sig.butter(order, cutoff / nyquist)
    #if not np.all(np.abs(np.roots(a)) < 1):
    #    raise PsolaError('Filter with cutoff at {} Hz is unstable given '
    #                     'sample frequency {} Hz'.format(cutoff, fs))
    #r_x = sig.filtfilt(b, a, r[:, 0], method='gust')
    #r_y = sig.filtfilt(b, a, r[:, 1], method='gust')
    #r_yaw = sig.filtfilt(b, a, r[:, 2], method='gust')

    diff = m - s

    fig, ax = plt.subplots(2, 2)
    x = range(1, MAX_LOG)
    ax[0, 0].plot(x, r[:, 0], 'r', label="estimated")
    #ax[0, 0].plot(x, r_x, 'k')
    ax[0, 0].plot(x, t[:, 0], 'b', label="ground truth")
    #ax[0, 0].plot(x, path_np[:, 0], 'g')
    ax[0, 0].set_xlabel("Step")
    ax[0, 0].set_ylabel("Position (m)")
    ax[0, 0].set_title("X position")
    ax[0, 0].legend()

    ax[0, 1].plot(x, r[:, 1], 'r', label="estimated")
    #ax[0, 1].plot(x, r_y, 'k')
    ax[0, 1].plot(x, t[:, 1], 'b', label="ground truth")
    #ax[0, 0].plot(x, path_np[:, 1], 'g')
    ax[0, 1].set_xlabel("Step")
    ax[0, 1].set_ylabel("Position (m)")
    ax[0, 1].set_title("Y position")
    ax[0, 1].legend()

    ax[1, 0].plot(x, r[:,2], 'r', label="estimated")
    #ax[1, 0].plot(x, r_yaw, 'k')
    ax[1, 0].plot(x, t[:,2], 'b', label="ground truth")
    ax[1, 0].set_xlabel("Step")
    ax[1, 0].set_ylabel("Position (m)")
    ax[1, 0].set_title("Yaw position")
    ax[1, 0].legend()

    ax[1, 1].set_visible(False)

    fig.suptitle("Pose evolution")

    fig2, ax2 = plt.subplots(4, 2)
    '''
    ax2[0, 0].plot(x, diff[:, 0], 'g')
    ax2[0, 1].plot(x, diff[:, 1], 'g')
    ax2[1, 0].plot(x, diff[:, 2], 'g')
    ax2[1, 1].plot(x, diff[:, 3], 'g')
    ax2[2, 0].plot(x, diff[:, 4], 'g')
    ax2[2, 1].plot(x, diff[:, 5], 'g')
    ax2[3, 0].plot(x, diff[:, 6], 'g')
    ax2[3, 1].plot(x, diff[:, 7], 'g')
    '''

    ax2[0, 0].plot(x, m[:, 0], 'r', label="expected")
    ax2[0, 0].plot(x, s[:, 0], 'b', label="real")
    ax2[0, 0].set_xlabel("Step")
    ax2[0, 0].set_ylabel("Reading (m)")
    ax2[0, 0].legend()

    ax2[0, 1].plot(x, m[:, 1], 'r', label="expected")
    ax2[0, 1].plot(x, s[:, 1], 'b', label="real")
    ax2[0, 1].set_xlabel("Step")
    ax2[0, 1].set_ylabel("Reading (m)")
    ax2[0, 1].legend()

    ax2[1, 0].plot(x, m[:, 2], 'r', label="expected")
    ax2[1, 0].plot(x, s[:, 2], 'b', label="real")
    ax2[1, 0].set_xlabel("Step")
    ax2[1, 0].set_ylabel("Reading (m)")
    ax2[1, 0].legend()

    ax2[1, 1].plot(x, m[:, 3], 'r', label="expected")
    ax2[1, 1].plot(x, s[:, 3], 'b', label="real")
    ax2[1, 1].set_xlabel("Step")
    ax2[1, 1].set_ylabel("Reading (m)")
    ax2[1, 1].legend()

    ax2[2, 0].plot(x, m[:, 4], 'r', label="expected")
    ax2[2, 0].plot(x, s[:, 4], 'b', label="real")
    ax2[2, 0].set_xlabel("Step")
    ax2[2, 0].set_ylabel("Reading (m)")
    ax2[2, 0].legend()

    ax2[2, 1].plot(x, m[:, 5], 'r', label="expected")
    ax2[2, 1].plot(x, s[:, 5], 'b', label="real")
    ax2[2, 1].set_xlabel("Step")
    ax2[2, 1].set_ylabel("Reading (m)")
    ax2[2, 1].legend()

    ax2[3, 0].plot(x, m[:, 6], 'r', label="expected")
    ax2[3, 0].plot(x, s[:, 6], 'b', label="real")
    ax2[3, 0].set_xlabel("Step")
    ax2[3, 0].set_ylabel("Reading (m)")
    ax2[3, 0].legend()

    ax2[3, 1].plot(x, m[:, 7], 'r', label="expected")
    ax2[3, 1].plot(x, s[:, 7], 'b', label="real")
    ax2[3, 1].set_xlabel("Step")
    ax2[3, 1].set_ylabel("Reading (m)")
    ax2[3, 1].legend()

    fig2.suptitle("Measurements")
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
