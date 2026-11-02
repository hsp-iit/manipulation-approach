# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""Runs the real ObjectDetector node with DINO/SAM stubbed out (never detects anything), on CPU."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import numpy as np
import torch

import detector as det_mod
det_mod.load_model = lambda *a, **k: None
det_mod.SAM = lambda *a, **k: None
det_mod.predict = lambda **k: (torch.zeros((0, 4)), torch.zeros((0,)), [])
det_mod.annotate = lambda image_source, boxes, logits, phrases: np.ascontiguousarray(image_source)

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor


def main():
    rclpy.init(args=['--ros-args',
                     '-p', 'use_camera_info_topic:=false',
                     '-p', 'goal_update_min_interval:=0.5',
                     '-p', 'navigation_start_timeout:=3.0',
                     '-p', 'raise_hands_before_approach:=' + os.environ.get('STUB_RAISE_HANDS', 'false'),
                     '-p', 'hands_motion_duration:=0.5',
                     '-p', 'hands_motion_timeout:=2.0',
                     '-p', 'left_arm_rpc_port:=/fake_arm/left/rpc:i',
                     '-p', 'right_arm_rpc_port:=/fake_arm/right/rpc:i',
                     '-p', 'arm_reference_frame:=fake_root',
                     '-p', 'robot_base_frame:=base'])
    node = det_mod.ObjectDetector()
    node.device = 'cpu'
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
