#!/usr/bin/env python3
"""Publish a dynamic TF for the movable rebar 1A.

Reads workcell.movable_rebars.rebar_1A from workcell.yaml.
The planning scene determines whether 1A is in the world or
attached to a robot link.

This node publishes TF only. It does not modify collision objects.
"""

import math
import os

import yaml
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf2_ros import TransformBroadcaster


class MovableRebarTF(Node):

    def __init__(self):
        super().__init__('movable_rebar_tf')

        # This node is the sole TF publisher for child frame "1A".
        self.tf_broadcaster = TransformBroadcaster(self)

        # Load the movable rebar configuration.
        pkg = get_package_share_directory('sa2_description')
        yaml_file = os.path.join(pkg, 'config', 'workcell.yaml')

        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}

        movable = (
            config.get('workcell', {})
            .get('movable_rebars', {})
        )

        self.rebar_id = 'rebar_1A'
        self.data = movable.get(self.rebar_id)

        if not self.data:
            raise RuntimeError(
                f'{self.rebar_id} not found under '
                'workcell.movable_rebars in workcell.yaml'
            )

        self.child_frame = self.data['child']
        self.yaml_parent = self.data['parent']

        # Current parent and pose. Start with the YAML pose.
        self.current_parent = self.yaml_parent
        self.current_position = tuple(
            float(v) for v in self.data['xyz']
        )
        self.current_quaternion = self.rpy_to_quaternion(
            *[float(v) for v in self.data['rpy']]
        )

        self.request_pending = False

        self.scene_client = self.create_client(
            GetPlanningScene, 'get_planning_scene'
        )

        # Poll the planning scene and publish the current TF.
        self.scene_timer = self.create_timer(
            0.1, self.poll_scene
        )
        self.tf_timer = self.create_timer(
            0.05, self.publish_tf
        )

        self.get_logger().info(
            f'Monitoring MoveIt object "{self.rebar_id}" '
            f'and publishing TF child "{self.child_frame}"'
        )

    def poll_scene(self):
        if (
            self.request_pending
            or not self.scene_client.service_is_ready()
        ):
            return

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        self.request_pending = True
        future = self.scene_client.call_async(request)
        future.add_done_callback(self.on_scene)

    def on_scene(self, future):
        self.request_pending = False

        try:
            scene = future.result().scene
        except Exception as error:
            self.get_logger().warning(
                f'Planning scene query failed: {error}'
            )
            return

        # Case 1: 1A is attached to a robot link.
        for attached in scene.robot_state.attached_collision_objects:
            obj = attached.object

            if obj.id != self.rebar_id:
                continue

            self.set_pose_from_message(
                attached.link_name, obj.pose
            )
            return

        # Case 2: 1A is a world collision object.
        for obj in scene.world.collision_objects:
            if obj.id != self.rebar_id:
                continue

            parent = obj.header.frame_id or 'world'
            self.set_pose_from_message(parent, obj.pose)
            return

        # Case 3: object not present in either scene collection.
        # Fall back to its configured YAML pose.
        self.current_parent = self.yaml_parent
        self.current_position = tuple(
            float(v) for v in self.data['xyz']
        )
        self.current_quaternion = self.rpy_to_quaternion(
            *[float(v) for v in self.data['rpy']]
        )

    def set_pose_from_message(self, parent, pose):
        self.current_parent = parent

        self.current_position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )

        self.current_quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

    def publish_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.current_parent
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = self.current_position[0]
        transform.transform.translation.y = self.current_position[1]
        transform.transform.translation.z = self.current_position[2]

        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = self.current_quaternion

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MovableRebarTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
