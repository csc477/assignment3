"""laser_and_odometry/ is the complete recording, and plays back like the ROS 1 bag did."""
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

BAG = Path(__file__).resolve().parents[1] / 'laser_and_odometry'

EXPECTED_TOPICS = {
    '/husky_1/scan': ('sensor_msgs/msg/LaserScan', 1384),
    '/husky_1/odometry/ground_truth': ('nav_msgs/msg/Odometry', 13842),
    '/tf': ('tf2_msgs/msg/TFMessage', 23533),
    '/tf_static': ('tf2_msgs/msg/TFMessage', 1),
    '/clock': ('rosgraph_msgs/msg/Clock', 13846),
}


def metadata_topics():
    meta = yaml.safe_load((BAG / 'metadata.yaml').read_text())
    return {t['topic_metadata']['name']: t for t in meta['rosbag2_bagfile_information']['topics_with_message_count']}


def test_topics_types_and_counts():
    got = {name: (t['topic_metadata']['type'], t['message_count']) for name, t in metadata_topics().items()}
    assert got == EXPECTED_TOPICS


def test_tf_static_is_latched():
    # Without transient local, tf2 listeners and rviz2 never get the laser mount.
    profiles = yaml.safe_load(metadata_topics()['/tf_static']['topic_metadata']['offered_qos_profiles'])
    assert profiles[0]['durability'] == 1  # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL


def test_first_scan():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(BAG), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/husky_1/scan']))
    _, data, _ = reader.read_next()
    scan = deserialize_message(data, LaserScan)
    assert scan.header.frame_id == 'husky_1/base_laser'
    assert len(scan.ranges) == 720
    assert (round(scan.angle_min, 4), round(scan.angle_max, 4)) == (-2.3562, 2.3562)
    assert (round(scan.range_min, 4), round(scan.range_max, 4)) == (0.1, 8.0)
