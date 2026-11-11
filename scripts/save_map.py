#!/usr/bin/env python3
"""Save the occupancy grid that occupancy_grid_mapper.py publishes as a PNG.

Usage: ros2 run estimation_assignment save_map.py <output.png> [topic]

Run it while the mapper is still running, e.g. after the bag has finished
playing. Each grid cell becomes a 4x4 pixel block whose grey level is the
occupancy belief: white = free, black = occupied, mid-grey = unknown. The
image is oriented like rviz2's top-down view, with x to the right and y up.
"""
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

DEFAULT_TOPIC = '/husky_1/occupancy_grid'
PIXELS_PER_CELL = 4
TIMEOUT_S = 10.0


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) not in (2, 3):
        sys.exit(__doc__)
    out_path = args[1]
    topic = args[2] if len(args) == 3 else DEFAULT_TOPIC

    rclpy.init()
    node = Node('save_map')
    maps = []
    # The mapper publishes transient local, so the latest map arrives at once.
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, topic, maps.append, qos)
    deadline = time.monotonic() + TIMEOUT_S
    while not maps and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if not maps:
        sys.exit(f'no map on {topic} within {TIMEOUT_S:.0f} s; is occupancy_grid_mapper.py running?')

    grid = maps[-1]
    belief = np.array(grid.data, dtype=float).reshape(grid.info.height, grid.info.width)
    belief[belief < 0] = 50.0  # -1 means unknown in OccupancyGrid
    image = 1.0 - belief / 100.0
    image = np.kron(image, np.ones((PIXELS_PER_CELL, PIXELS_PER_CELL)))
    plt.imsave(out_path, image, cmap='gray', vmin=0.0, vmax=1.0, origin='lower')
    print(f'saved {grid.info.width}x{grid.info.height} map ({grid.info.resolution:g} m/cell) '
          f'to {out_path}')


if __name__ == '__main__':
    main()
