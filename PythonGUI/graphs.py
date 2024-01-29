import math
import time
from typing import Callable
import matplotlib
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from scipy.interpolate import interp1d
from scipy.interpolate import BSpline, make_interp_spline
import random
import socket_server
import datetime

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

class StatusState:

    def __init__(self) -> None:
        self.status: socket_server.StatusResponse 
        self.start_time = datetime.datetime.now()
        self.status = socket_server.StatusResponse()
        self.socket = socket_server.Server()


    @property
    def time(self) -> float:
        return (datetime.datetime.now() - self.start_time).microseconds / 1E6 + (datetime.datetime.now() - self.start_time).seconds

    def update(self):
        # self.status.gyro_value = 20 * math.sin(self.time * 2)
        self.status = self.socket.wait_for_update()





class Window:

    def __init__(self) -> None:
        
        style.use("seaborn-v0_8")

        self.fig = plt.figure()
        self.fig.tight_layout(pad=5.0)
        self.graphs: list[Graph] = []

        self.ani = animation.FuncAnimation(
            self.fig,
            lambda i: self.animate(i), # type: ignore
            interval=100,
            cache_frame_data=False
        )

        self.status: StatusState = StatusState()

    def add_graph(self, graph: 'Graph'):
        self.graphs.append(graph)

    def show(self):
        plt.show()

    def animate(self, i):
        self.status.update()
        for graph in self.graphs:
            graph.animate(i)
            # print("animating graph", graph.name)

class Graph:

    def __init__(self, window: Window, name: str, poller: Callable[..., float], limits: tuple[int, int], placement: tuple = (1,1,1)) -> None:
        self.name = name 
        self.window = window
        self.fig = self.window.fig
        self.ax = self.fig.add_subplot(*placement)
        self.points: list[tuple[float, float]] = [(0,0)]
        self.poller = poller
        self.limits = limits

    
    def animate(self, i):
        xs = []
        ys = []
        for point in self.points[-50:]:
            x, y = point 
            xs.append(x)
            ys.append(y)

        self.ax.clear()
        self.ax.set_ylim(*self.limits)
        self.ax.set_title(self.name)
        self.points.append((self.window.status.time, self.poller()))

        print(f"{self.name} plotting", self.points[-1])

        # print("time:", self.window.status.time)
        # print(f"{self.name} -- x: {xs}")
        if len(xs) > 5:
            x_new = np.linspace(min(xs), max(xs), 300)
            spl = make_interp_spline(xs, ys, k=3)
            y_smooth = spl(x_new)

            self.ax.plot(x_new, y_smooth, label='Smoothed curve')

        # self.points.append((self.points[-1][0] + 0.1, 10 * (random.random() + 0.5) * math.sin(self.points[-1][0] * 5)))


