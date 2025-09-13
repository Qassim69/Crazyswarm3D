#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import time

def simtime_talker():
    rclpy.init()

    node = rclpy.create_node('talker')
    pub = node.create_publisher(Clock, '/clock', 10)

    # Declare and get parameters
    node.declare_parameter('runtime', 10.0)
    node.declare_parameter('factor', 1.0)
    
    runtime = node.get_parameter('runtime').get_parameter_value().double_value
    sim_multiplier = node.get_parameter('factor').get_parameter_value().double_value

    base_rate = 1.0 / 20.0  # 20 Hz
    increment = base_rate
    publish_rate = base_rate / sim_multiplier

    sim_time = 0.0
    clock_msg = Clock()

    try:
        while rclpy.ok():
            if sim_time >= runtime:
                node.get_logger().info('End of simulation runtime.')
                break

            sim_time += increment
            seconds = int(sim_time)
            nanoseconds = int((sim_time - seconds) * 1e9)
            clock_msg.clock = Time(sec=seconds, nanosec=nanoseconds)

            pub.publish(clock_msg)
            # node.get_logger().info(f"Published time: {clock_msg}")

            time.sleep(publish_rate)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    simtime_talker()
