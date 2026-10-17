#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from ament_index_python.packages import get_package_share_directory


class WorkcellVisualizer(Node):

    def __init__(self):
        super().__init__('workcell_visualizer')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # =====================================================
        # LOAD YAML
        # =====================================================

        pkg = get_package_share_directory('sa2_description')

        yaml_file = os.path.join(
            pkg,
            'config',
            'workcell.yaml'
        )

        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)

        workcell = config['workcell']

        transforms = []

        # =====================================================
        # WEB RING FRAMES
        # =====================================================

        web_ring_frames = workcell.get(
            'web_ring_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(web_ring_frames)
        )

        # =====================================================
        # WEB RING FRAMES 2
        # =====================================================

        web_ring2_frames = workcell.get(
            'web_ring2_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(web_ring2_frames)
        )

        # =====================================================
        # PLACE FRAMES
        # =====================================================

        place_frames = workcell.get(
            'place_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(place_frames)
        )

        # =====================================================
        # tool_dock_frames
        # =====================================================

        tool_dock_frames = workcell.get(
            'tool_dock_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(tool_dock_frames)
        )

        # =====================================================
        # INVENTORY FRAMES
        # =====================================================

        inventory_frames = workcell.get(
            'inventory_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(inventory_frames)
        )


        # =====================================================
        # ARC frames
        # =====================================================

        arc_frames = workcell.get(
            'arc_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(arc_frames)
        )


        # =====================================================
        # INVENTORY FRAMES
        # =====================================================

        tying_frames = workcell.get(
            'tying_frames',
            {}
        )

        transforms.extend(
            self.create_transforms(tying_frames)
        )
        
        # =====================================================
        # JIG CLAMP
        # =====================================================

        jig_clamp = workcell.get(
            'jig_clamp_pf',
            {}
        ).get(
            'jig_clamp'
        )

        if jig_clamp:
            transforms.extend(
                self.create_transforms(
                    {
                        'jig_clamp': jig_clamp
                    }
                )
            )

        # =====================================================
        # PUBLISH ALL TFs
        # =====================================================

        self.tf_broadcaster.sendTransform(transforms)

        self.get_logger().info(
            f'Published {len(transforms)} static transforms'
        )


    def create_transforms(self, frames):

        transforms = []

        for name, data in frames.items():

            transform = TransformStamped()

            transform.header.stamp = (
                self.get_clock().now().to_msg()
            )

            transform.header.frame_id = data['parent']
            transform.child_frame_id = data['child']

            xyz = data['xyz']
            rpy = data['rpy']

            # =================================================
            # POSITION
            # =================================================

            transform.transform.translation.x = float(xyz[0])
            transform.transform.translation.y = float(xyz[1])
            transform.transform.translation.z = float(xyz[2])

            # =================================================
            # ORIENTATION
            # =================================================

            qx, qy, qz, qw = self.rpy_to_quaternion(
                float(rpy[0]),
                float(rpy[1]),
                float(rpy[2])
            )

            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            transforms.append(transform)

            self.get_logger().info(
                f'TF: {data["parent"]} -> {data["child"]} '
                f'xyz={xyz} rpy={rpy}'
            )

        return transforms


    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw


def main(args=None):

    rclpy.init(args=args)

    node = WorkcellVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()