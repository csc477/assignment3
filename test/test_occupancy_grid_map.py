"""The provided (non-TODO) grid code produces valid nav_msgs/OccupancyGrid messages."""
import sys
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from occupancy_grid_mapper import OccupancyGridMap  # noqa: E402


def make_map():
    # the size, resolution and origin that occupancy_grid_mapper.py uses
    return OccupancyGridMap(250, 250, 0.2, np.array([-20, -10, 0]), 0)


def test_unknown_map_message():
    msg = make_map().get_map_as_ros_msg(Time(sec=21600), 'map')
    assert msg.header.frame_id == 'map'
    assert msg.header.stamp.sec == 21600
    assert (msg.info.width, msg.info.height) == (250, 250)
    assert abs(msg.info.resolution - 0.2) < 1e-6
    assert (msg.info.origin.position.x, msg.info.origin.position.y) == (-20.0, -10.0)
    assert len(msg.data) == 250 * 250
    assert set(msg.data) == {50}  # log odds 0 is p = 0.5


def test_occupied_and_free_cells():
    m = make_map()
    m.update_log_odds_ratio_in_grid_coords(10, 20, np.log(0.999 / 0.001))
    m.update_log_odds_ratio_in_grid_coords(11, 20, np.log(0.001 / 0.999))
    data = m.get_map_as_ros_msg(Time(), 'map').data
    assert data[10 * 250 + 20] == 99
    assert data[11 * 250 + 20] == 0


def test_cell_of_a_point_is_where_rviz_draws_it():
    # OccupancyGrid data is row-major with rows along y: cell (row, col)
    # covers [origin.x + col*res, +res) x [origin.y + row*res, +res).
    m = make_map()
    x, y = 1.1, 3.3
    row, col = m.cartesian_to_grid_coords(x, y)
    m.update_log_odds_ratio_in_grid_coords(row, col, 10.0)
    msg = m.get_map_as_ros_msg(Time(), 'map')
    idx = list(msg.data).index(max(msg.data))
    res = msg.info.resolution
    x0 = msg.info.origin.position.x + (idx % msg.info.width) * res
    y0 = msg.info.origin.position.y + (idx // msg.info.width) * res
    assert x0 <= x < x0 + res and y0 <= y < y0 + res
