"""
GUI Manager

Spawns window with graphs and handles animations.

January, 2024
"""

import socket_server

from scipy.interpolate import make_interp_spline
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib import style
from typing import Callable
import numpy as np
import datetime
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL) # force kill when matplotlib blocks Ctrl+C

class StatusState:
    """Handles telemetry status updates"""

    def __init__(self) -> None:
        self.status: socket_server.StatusResponse
        self.start_time = datetime.datetime.now()
        self.status = socket_server.StatusResponse()
        self.socket = socket_server.Server()

    @property
    def time(self) -> float:
        """Returns the time since the status stream beguin"""

        return (datetime.datetime.now() - self.start_time).microseconds / 1e6 + (
            datetime.datetime.now() - self.start_time
        ).seconds

    def wait_for_update(self):
        """Waits for a socket status update"""
        self.status = self.socket.wait_for_update()


class Window:
    """Handles animation between multiple matplotlib graphs"""

    def __init__(self) -> None:
        style.use("seaborn-v0_8")

        self.fig = plt.figure()
        self.fig.tight_layout(pad=5.0)
        self.graphs: list[Graph] = []

        self.ani = animation.FuncAnimation(
            self.fig,
            lambda i: self._animate(i),  # type: ignore
            interval=100,
            cache_frame_data=False,
        )

        self.status: StatusState = StatusState()

    def add_graph(self, graph: "Graph") -> None:
        """Adds a graph to the window"""

        self.graphs.append(graph)

    def show(self):
        """Shows the window. Should be called after all graphs are initialized"""
        plt.show()

    def _animate(self, i):
        """Dispatches animation to child graphs"""
        self.status.wait_for_update()
        for graph in self.graphs:
            graph._animate(i)


class Graph:
    def __init__(
        self,
        window: Window,
        name: str,
        poller: Callable[..., float],
        limits: tuple[int, int],
        placement: tuple = (1, 1, 1),
    ) -> None:
        """Constructor for a Graph.
        
        Args:
            window: The window to add the graph to
            name: The name of the graph (used in title bar)
            poller: The function to poll during update
            limits: Y-axis limits (min, max)
            placement: The matplotlib placement (#rows, #cols, index)
        """
        self.name = name
        self.window = window
        self.fig = self.window.fig
        self.ax = self.fig.add_subplot(*placement)
        self.points: list[tuple[float, float]] = [(0, 0)]
        self.poller = poller
        self.limits = limits

    def _animate(self, i):
        """Redraws graph for each animation frame"""

        # poll for newest point
        self.points.append((self.window.status.time, self.poller()))

        # load arrays with points
        interval_size = 50 if len(self.points) >= 50 else len(self.points)
        xs = np.empty((interval_size), dtype=float)
        ys = np.empty((interval_size), dtype=float)
        for i, point in enumerate(self.points[-interval_size:]):
            x, y = point
            xs[i] = x
            ys[i] = y

        # prepare graph for draw
        self.ax.clear()
        self.ax.set_ylim(*self.limits)
        self.ax.set_title(self.name)

        # only draw after >= 50 points are accumulated
        if interval_size >= 50:
            x_new = np.linspace(min(xs), max(xs), 300)
            spl = make_interp_spline(xs, ys, k=3)
            y_smooth = spl(x_new)

            self.ax.plot(x_new, y_smooth, label="Smoothed curve")
