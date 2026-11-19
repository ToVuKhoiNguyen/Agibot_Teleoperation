#!/usr/bin/env python3
"""sil/fake_camera.py - synthetic head camera for SIL: rgb8 colour + 16UC1 depth frames on the real robot's topic
names, plus a neck joint state (yaw, pitch), so the data-collection path can be tested end to end.
Run with the SIL ROS environment (isolated domain, localhost only); refuses the real robot's domain 232."""
import os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState

TOPIC = "/aima/hal/rgbd_camera/head_front/color"
DEPTH_TOPIC = "/aima/hal/rgbd_camera/head_front/depth"
NECK_TOPIC = "/motion/control/neck_joint_state"


def main():
    if os.environ.get("ROS_DOMAIN_ID", "0") == "232" or os.environ.get("ROS_LOCALHOST_ONLY", "0") != "1":
        sys.exit("FATAL: fake camera must run on an isolated ROS domain (not 232) with ROS_LOCALHOST_ONLY=1")
    w, h, hz = 640, 480, 15.0
    rclpy.init()
    node = Node("fake_head_camera")
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Image, TOPIC, qos)
    pub_d = node.create_publisher(Image, DEPTH_TOPIC, qos)
    pub_n = node.create_publisher(JointState, NECK_TOPIC, qos)
    yy, xx = np.mgrid[0:h, 0:w]
    k = 0
    try:
        while rclpy.ok():
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[..., 0] = (xx + 4 * k) % 256            # moving gradient: frames differ, so order can be checked
            img[..., 1] = (yy * 255 // h)
            img[..., 2] = (k * 5) % 256
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg(); msg.header.frame_id = "head_front"
            msg.height, msg.width, msg.encoding, msg.is_bigendian, msg.step = h, w, "rgb8", 0, w * 3
            msg.data = img.tobytes()
            pub.publish(msg)
            # depth in millimetres, 16UC1 little-endian: a ramp 500..2999 mm plus the frame counter in the first pixel
            dep = (500 + (xx * 2500 // w)).astype("<u2"); dep[0, 0] = k % 65536
            dm = Image()
            dm.header.stamp = msg.header.stamp; dm.header.frame_id = "head_front"
            dm.height, dm.width, dm.encoding, dm.is_bigendian, dm.step = h, w, "16UC1", 0, w * 2
            dm.data = dep.tobytes()
            pub_d.publish(dm)
            js = JointState(); js.header.stamp = msg.header.stamp
            js.name, js.position = ["idx27_head_joint1", "idx28_head_joint2"], [0.10, -0.05]
            pub_n.publish(js)
            k += 1
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
