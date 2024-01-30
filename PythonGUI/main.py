"""
Entry Point

Starts socket server and opens window when ready.

January 2024
"""

import gui

window = gui.Window()
window.add_graph(
    gui.Graph(
        window=window,
        name="Gyroscope",
        placement=(1, 3, 1),
        poller=lambda: window.status.status.gyro_value,
        limits=(-45, 45),
    )
)
window.add_graph(
    gui.Graph(
        window=window,
        name="Motor Target",
        placement=(1, 3, 2),
        poller=lambda: window.status.status.motor_target,
        limits=(-55, 55),
    )
)
window.add_graph(
    gui.Graph(
        window=window,
        name="Integral Sum",
        placement=(1, 3, 3),
        poller=lambda: window.status.status.integral_sum,
        limits=(-110, 110),
    )
)
window.show()
