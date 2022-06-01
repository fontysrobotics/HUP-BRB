from typing import List
import numpy as np
from math import sqrt
from matplotlib import pyplot as plt

from geometry_msgs.msg import Vector3

def plot_distance(x1 : List[float], y1 : List[float], v1: Vector3, x2 : List[float], y2 : List[float], v2: Vector3) -> None:
    """Plot the distance between two paths over time"""
    getdist = lambda pt1, pt2: sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    dist1 = np.insert(np.cumsum(list(map(getdist, zip(x1[:-1], y1[:-1]), zip(x1[1:], y1[1:])))), 0, 0)
    dist2 = np.insert(np.cumsum(list(map(getdist, zip(x2[:-1], y2[:-1]), zip(x2[1:], y2[1:])))), 0, 0)

    t1 = dist1/sqrt(v1.x**2 + v1.y**2)
    t2 = dist2/sqrt(v2.x**2 + v2.y**2)

    t = np.union1d(t1, t2)
    plt.subplot(2, 2, 1)
    plt.plot(x1, y1, 'o-')
    plt.plot(x2, y2, 'o-')

    plt.subplot(2, 2, 2)
    x1 = np.interp(t, t1, x1)
    x2 = np.interp(t, t2, x2)
    distx = x1-x2
    plt.plot(t, distx, 'o-')
    
    plt.subplot(2, 2, 3)
    y1 = np.interp(t, t1, y1)
    y2 = np.interp(t, t2, y2)
    disty = y1-y2
    plt.plot(t, disty, 'o-')

    plt.subplot(2, 2, 4)
    dist = np.sqrt(np.square(distx)+np.square(disty))
    plt.plot(t, dist, 'o-')

    plt.subplot(2, 2, 1)
    nmin = np.argmin(dist)
    plt.plot([x1[nmin], x2[nmin]], [y1[nmin], y2[nmin]])
    
    plt.show()