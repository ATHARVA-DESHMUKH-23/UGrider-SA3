#!/usr/bin/env python3

import os

import yaml
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker


class MovableRebarMarker(Node):

    def __init__(self):
        super().__init__('movable_rebar_marker')

        pkg = get_package_share_directory('sa2_description')
        yaml_file = os.path.join(pkg, 'config', 'workcell.yaml')

        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}

        data = (
            config.get('workcell', {})
            .get('movable_rebars', {})
            .get('rebar_1A')
        )

        if not data:
            raise RuntimeError(
                'rebar_1A not found in workcell.movable_rebars'
            )

        self.publisher = self.create_publisher(
            Marker, '/movable_rebar_marker', 10
        )

        self.marker = Marker()
        self.marker.header.frame_id = data['child']  # TF frame: 1A
        self.marker.ns = 'movable_rebar'
        self.marker.id = 0
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD

        self.marker.mesh_resource = data['mesh']
        self.marker.mesh_use_embedded_materials = False

        scale = float(data.get('scale', 0.001))
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale

        color = data.get('color', [0.5, 0.5, 0.5, 1.0])
        self.marker.color.r = float(color[0])
        self.marker.color.g = float(color[1])
        self.marker.color.b = float(color[2])
        self.marker.color.a = float(color[3])

        # Mesh origin coincides with the TF frame origin.
        self.marker.pose.orientation.w = 1.0

        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info(
            f'Publishing mesh {self.marker.mesh_resource} '
            f'in TF frame {self.marker.header.frame_id}'
        )

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = MovableRebarMarker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
