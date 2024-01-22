#!/usr/bin/env python3

import numpy as np
from numpy import arctan2, sign, arctan, cos, sin, pi,arange
from numpy.linalg import norm, det
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin, ax.xmax)
    ax.set_ylim(ax.ymin, ax.ymax)

def init_figure(xmin, xmax, ymin, ymax):
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    ax.xmin = xmin
    ax.xmax = xmax
    ax.ymin = ymin
    ax.ymax = ymax
    clear(ax)
    return ax

def angle(x):
    x = x.flatten()
    return np.arctan2(x[1], x[0])

def add1(M):
    M = np.array(M)
    return np.vstack((M, np.ones(M.shape[1])))

def tran2H(x, y):
    return np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

def rot2H(a):
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])

def draw_arrow(x, y, θ, L, col='darkblue', w=1):
    plot2D(tran2H(x, y) @ rot2H(θ) @ arrow2H(L), col, w)

def plot2D(M, col='black', w=1):
    plt.plot(M[0, :], M[1, :], col, linewidth=w)

def arrow2H(L):
    e = 0.2
    return add1(L * np.array([[0, 1, 1 - e, 1, 1 - e], [0, 0, -e, 0, e]]))

def draw_disk(ax, c, r, col, alph=0.7, w=1):
    e = Ellipse(xy=c, width=2 * r, height=2 * r, angle=0, linewidth=w)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)

def draw_saildrone(x, u, ψ, awind, ψ_ap, a_ap, fr, fs, ff):
    mx, my, θ, v, w, δs = list(x[0:6, 0])
    u1, u2 = list(u[0:2, 0])
    hull = add1(np.array([[-1, 5, 7, 7, 5, -1, -1, -1], [-2, -2, -1, 1, 2, 2, -2, -2]]))
    sail = np.array([[-5, 2.5], [0, 0], [1, 1]])
    rudder = np.array([[-1, 1], [0, 0], [1, 1]])
    R = tran2H(mx, my) @ rot2H(θ)
    Rs = tran2H(3, 0) @ rot2H(δs)
    Rf = tran2H(-6, 0) @ rot2H(-u2)
    Rr = tran2H(-1, 0) @ rot2H(u1)
    draw_arrow(17, 17, ψ, 5 * awind, 'red')
    draw_arrow(17, 17, ψ_ap + θ, 5 * a_ap, 'green')
    plot2D(R @ Rs @ Rf @ rot2H(-np.pi / 2) @ arrow2H(ff), 'blue')
    plot2D(R @ Rs @ rot2H(-np.pi / 2) @ arrow2H(0.001 * fs), 'blue')
    plot2D(R @ Rr @ rot2H(np.pi / 2) @ arrow2H(0.001 * fr), 'blue')
    draw_disk(ax, R @ np.array([[3], [0], [1]]), 0.5, "red")
    plot2D(R @ hull, 'black');
    plot2D(R @ Rs @ sail, 'red', 2);
    plot2D(R @ Rr @ rudder, 'red', 2);
    plot2D(R @ Rs @ Rf @ rudder, 'green', 2)

if __name__ == '__main__':
    dt=10
    for t in arange (0,100,dt) :
        clear(ax)
        ax = init_figure(-150, 150, -80, 80)
        x = np.array([100, -70, -3, 1, 0, 1]).reshape(6, 1)
        u = np.array([0, 0]).reshape(2, 1)
        x=x+dt

        draw_saildrone(x, u, 1, 1, 1, 1, 1, 1, 1)

        
        plt.show()
