import graphs

window = graphs.Window()
window.add_graph(
    graphs.Graph(
        window=window, 
        name="Gyroscope", 
        placement=(1,3,1),
        poller=lambda: window.status.status.gyro_value,
        limits=(-45, 45)
        )
)
window.add_graph(
    graphs.Graph(
        window=window, 
        name="Motor Target", 
        placement=(1,3,2),
        poller=lambda: window.status.status.motor_target,
        limits=(-55, 55)
        )
)
window.add_graph(
    graphs.Graph(
        window=window, 
        name="Integral Sum", 
        placement=(1,3,3),
        poller=lambda: window.status.status.integral_sum,
        limits=(-12, 12)
        )
)
window.show()