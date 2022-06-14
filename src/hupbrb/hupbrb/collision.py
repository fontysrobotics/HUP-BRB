from turtle import Vec2D
from typing import List, Tuple, Union
import numpy as np
from math import sqrt
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from geometry_msgs.msg import Vector3

def setup_test():
    global x1, y1, x2, y2, v1, v2
    x1 = np.arange(0, 10, 0.1)
    y1 = np.square(x1)/10
    x2 = np.arange(0, 10, 0.1)
    y2 = np.arange(10, 0, -0.1)
    v1 = Vector3(x=1.0)
    v2 = Vector3(x=2.0)

def get_collision_coords(x1 : List[float], y1 : List[float], v1: Vector3, x2 : List[float], y2 : List[float], v2: Vector3, thres: float) ->  Union[None, Tuple[float, float, float]]:
    getdist = lambda pt1, pt2: sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
    
    dist1 = np.insert(np.cumsum(list(map(getdist, zip(x1[:-1], y1[:-1]), zip(x1[1:], y1[1:])))), 0, 0)
    dist2 = np.insert(np.cumsum(list(map(getdist, zip(x2[:-1], y2[:-1]), zip(x2[1:], y2[1:])))), 0, 0)

    t1 = dist1/sqrt(v1.x**2 + v1.y**2)
    t2 = dist2/sqrt(v2.x**2 + v2.y**2)

    t = np.arange(0, np.union1d(t1, t2)[-1], 0.01)

    x1 = np.interp(t, t1, x1)
    x2 = np.interp(t, t2, x2)
    distx = x1-x2
    
    y1 = np.interp(t, t1, y1)
    y2 = np.interp(t, t2, y2)
    disty = y1-y2

    dist = np.sqrt(np.square(distx)+np.square(disty))
    # nmin = np.argmin(dist)
    bin_dist = np.diff(dist<thres)
    n_end = np.argmax(bin_dist)

    if n_end == 0:
        return

    n_collision = np.argmin(dist[:n_end])

    # n_collision = np.argwhere(dist < thres)

    if not n_collision.size:
        return


    x = x2[np.min(n_collision)]
    y = y2[np.min(n_collision)]

    return (x, y, dist[np.min(n_collision)])

    self.mindist.set_data([x1[nmin], x2[nmin]], [y1[nmin], y2[nmin]])
    self.minpt.set_data(t[nmin], dist[nmin])



class Graph():
    def __init__(self, name):
        self.fig, self.ax = plt.subplots(2, 2)
        self.plan1 = Line2D([0], [0], color='blue')
        self.plan2 = Line2D([0], [0], color='orange')
        self.dx = Line2D([0], [0])
        self.dy = Line2D([0], [0])
        self.dist = Line2D([0], [0])
        self.mindist = Line2D([0], [0], color='red', marker='o')
        self.minpt = Line2D([0], [0], color='red', marker='o')

        self.ax[0, 0].add_line(self.plan1)
        self.ax[0, 0].add_line(self.plan2)
        self.ax[0, 0].add_line(self.mindist)

        self.ax[1, 0].add_line(self.dx)
        self.ax[0, 1].add_line(self.dy)

        self.ax[1, 1].add_line(self.dist)
        self.ax[1, 1].add_line(self.minpt)

        self.ax[1, 0].set_autoscalex_on(False)
        self.ax[0, 1].set_autoscalex_on(False)
        self.ax[1, 1].set_autoscalex_on(False)

        self.ax[0, 0].set_xlim(3, 5)
        self.ax[0, 0].set_ylim(0, 6)
        self.ax[1, 0].set_xlim(0, 20)
        self.ax[0, 1].set_xlim(0, 20)
        self.ax[1, 1].set_xlim(0, 20)
        self.ax[1, 1].set_ylim(0, 3)

        self.fig.tight_layout()
        self.fig.show()
        self.fig.canvas.set_window_title(name)
        plt.pause(0.01)
        # plt.show(block=False)

    def rescale(self, *axes: Tuple[int, int]):
        for a in axes:
            self.ax[a].relim()
            self.ax[a].autoscale_view()
        

    def update(self, x1 : List[float], y1 : List[float], v1: Vector3, x2 : List[float], y2 : List[float], v2: Vector3) -> None:
      
        getdist = lambda pt1, pt2: sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

        dist1 = np.insert(np.cumsum(list(map(getdist, zip(x1[:-1], y1[:-1]), zip(x1[1:], y1[1:])))), 0, 0)
        dist2 = np.insert(np.cumsum(list(map(getdist, zip(x2[:-1], y2[:-1]), zip(x2[1:], y2[1:])))), 0, 0)

        t1 = dist1/sqrt(v1.x**2 + v1.y**2)
        t2 = dist2/sqrt(v2.x**2 + v2.y**2)

        t = np.arange(0, np.union1d(t1, t2)[-1], 0.01)
        self.plan1.set_data(x1, y1)
        self.plan2.set_data(x2, y2)

        x1 = np.interp(t, t1, x1)
        x2 = np.interp(t, t2, x2)
        distx = x1-x2
        self.dx.set_data(t, distx)
        
        y1 = np.interp(t, t1, y1)
        y2 = np.interp(t, t2, y2)
        disty = y1-y2
        self.dy.set_data(t, disty)

        dist = np.sqrt(np.square(distx)+np.square(disty))
        nmin = np.argmin(dist)
        self.dist.set_data(t, dist)

        self.mindist.set_data([x1[nmin], x2[nmin]], [y1[nmin], y2[nmin]])
        self.minpt.set_data(t[nmin], dist[nmin])

        # plt.draw()

        self.rescale(
            (0, 1),
            (1, 0)
        )

        plt.pause(0.01)

def plot_distance(x1 : List[float], y1 : List[float], v1: Vector3, x2 : List[float], y2 : List[float], v2: Vector3) -> None:
    """Plot the distance between two paths over time"""
    getdist = lambda pt1, pt2: sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    dist1 = np.insert(np.cumsum(list(map(getdist, zip(x1[:-1], y1[:-1]), zip(x1[1:], y1[1:])))), 0, 0)
    dist2 = np.insert(np.cumsum(list(map(getdist, zip(x2[:-1], y2[:-1]), zip(x2[1:], y2[1:])))), 0, 0)

    t1 = dist1/sqrt(v1.x**2 + v1.y**2)
    t2 = dist2/sqrt(v2.x**2 + v2.y**2)

    global fig
    fig, ax = plt.subplots(2, 2)

    # t = np.union1d(t1, t2)
    global t, dist
    t = np.arange(0, np.union1d(t1, t2)[-1], 0.01)
    ax[0, 0].plot(x1, y1, '-')
    ax[0, 0].plot(x2, y2, '-')

    x1 = np.interp(t, t1, x1)
    x2 = np.interp(t, t2, x2)
    distx = x1-x2
    ax[1, 0].plot(t, distx, '-')
    
    y1 = np.interp(t, t1, y1)
    y2 = np.interp(t, t2, y2)
    disty = y1-y2
    ax[0, 1].plot(t, disty, '-')

    dist = np.sqrt(np.square(distx)+np.square(disty))
    nmin = np.argmin(dist)
    ax[1, 1].plot(t, dist, '-')

    global line, pt
    line = Line2D([x1[nmin], x2[nmin]], [y1[nmin], y2[nmin]], 1, '-', 'red', 'o')
    ax[0, 0].add_line(line)
    # line = ax[0, 0].plot([x1[nmin], x2[nmin]], [y1[nmin], y2[nmin]], 'o-')
    pt = ax[1, 1].plot(t[nmin], dist[nmin], 'o')

    # cursor = Cursor(ax[1, 1], False, True, useblit=False, color='red', linewidth=2)

    # cursor.onmove(update)

    # slider = Slider(
    #     ax=ax[1, 1],
    #     label="Time [s]",
    #     valmin=0,
    #     valmax=max(t),
    #     valinit=t[nmin]
    # )
    # slider.on_changed(update)