#!/usr/bin/env python3
"""Publish gripper component TFs and visualize gripper meshes in RViz2.

Reads sa2_description/config/workcell.yaml under workcell.extra_grippers.
Marker poses come from the MoveIt planning scene, so a gripper that is
attached to a robot link is drawn on that link, exactly where its
collision object is. If the scene is unavailable, the marker falls back
to the yaml child frame + mesh offset.
"""
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# small pose math: pose = (position [x, y, z], quaternion [x, y, z, w])
# ---------------------------------------------------------------------------
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def compose(parent, child):
    rotated = quat_rotate(parent[1], child[0])
    position = (parent[0][0] + rotated[0],
                parent[0][1] + rotated[1],
                parent[0][2] + rotated[2])
    return position, quat_mul(parent[1], child[1])


def msg_to_pose(pose_msg):
    p, o = pose_msg.position, pose_msg.orientation
    return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)


IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class GripperVisualizer(Node):
    def __init__(self):
        super().__init__('gripper_visualizer')
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        package_share = get_package_share_directory('sa2_description')
        yaml_file = os.path.join(package_share, 'config', 'workcell.yaml')
        with open(yaml_file, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}

        self.grippers = config.get('workcell', {}).get('extra_grippers', {})
        if not self.grippers:
            self.get_logger().warning(
                "No grippers found in workcell.yaml under 'extra_grippers'.")

        # ---------------- static TFs (needed by the collision spawner) -------
        transforms = []
        for name, data in self.grippers.items():
            required = ('parent', 'child', 'xyz', 'rpy', 'mesh')
            missing = [key for key in required if key not in data]
            if missing:
                self.get_logger().error(
                    f"Gripper component '{name}' missing fields {missing}; skipping TF.")
                continue

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = str(data['parent'])
            transform.child_frame_id = str(data['child'])
            xyz, rpy = data['xyz'], data['rpy']
            transform.transform.translation.x = float(xyz[0])
            transform.transform.translation.y = float(xyz[1])
            transform.transform.translation.z = float(xyz[2])
            q = self.rpy_to_quaternion(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            (transform.transform.rotation.x,
             transform.transform.rotation.y,
             transform.transform.rotation.z,
             transform.transform.rotation.w) = q
            transforms.append(transform)
            self.get_logger().info(
                f"Gripper TF: {transform.header.frame_id} -> {transform.child_frame_id}")

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'Published {len(transforms)} gripper component TFs.')

        # ---------------- planning-scene poses -------------------------------
        # id -> (frame_id, pose) taken from the scene (world or attached link)
        self.scene_poses = {}
        self.scene_ok = False
        self.request_pending = False
        self.scene_client = self.create_client(GetPlanningScene, 'get_planning_scene')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/gripper_markers', qos)

        self.publish_markers()
        self.scene_timer = self.create_timer(0.5, self.poll_scene)
        self.timer = self.create_timer(1.0, self.publish_markers)

    # -------------------------------------------------------------------------
    # Planning scene polling
    # -------------------------------------------------------------------------
    def poll_scene(self):
        if self.request_pending or not self.scene_client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        self.request_pending = True
        future = self.scene_client.call_async(request)
        future.add_done_callback(self.on_scene)

    def on_scene(self, future):
        self.request_pending = False
        try:
            scene = future.result().scene
        except Exception as error:  # service died, keep the last known state
            self.get_logger().warning(f'get_planning_scene failed: {error}')
            return

        poses = {}

        for obj in scene.world.collision_objects:
            if obj.id in self.grippers:
                frame = obj.header.frame_id or 'world'
                poses[obj.id] = (frame, self.object_pose(obj))

        for attached in scene.robot_state.attached_collision_objects:
            obj = attached.object
            if obj.id in self.grippers:
                poses[obj.id] = (attached.link_name, self.object_pose(obj))

        changed = poses != self.scene_poses
        self.scene_poses = poses
        self.scene_ok = True
        if changed:
            self.publish_markers()

    @staticmethod
    def object_pose(obj):
        """object.pose composed with the first mesh pose (what the collision uses)."""
        pose = msg_to_pose(obj.pose)
        if obj.mesh_poses:
            pose = compose(pose, msg_to_pose(obj.mesh_poses[0]))
        return pose

    # -------------------------------------------------------------------------
    # Markers
    # -------------------------------------------------------------------------
    def publish_markers(self):
        markers = []
        for index, (name, data) in enumerate(self.grippers.items()):
            if 'child' not in data or 'mesh' not in data:
                continue

            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'grippers'
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.frame_locked = True

            if name in self.scene_poses:
                # Same pose as the collision object (world or attached link).
                frame, (position, quat) = self.scene_poses[name]
                marker.header.frame_id = frame
            else:
                # Fallback: yaml child frame + mesh offset.
                marker.header.frame_id = str(data['child'])
                xyz = data.get('mesh_xyz', [0.0, 0.0, 0.0])
                rpy = data.get('mesh_rpy', [0.0, 0.0, 0.0])
                position = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
                quat = self.rpy_to_quaternion(float(rpy[0]), float(rpy[1]), float(rpy[2]))

            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            (marker.pose.orientation.x,
             marker.pose.orientation.y,
             marker.pose.orientation.z,
             marker.pose.orientation.w) = quat

            scale = data.get('scale', 0.001)
            if isinstance(scale, (int, float)):
                sx = sy = sz = float(scale)
            else:
                sx, sy, sz = (float(value) for value in scale)
            marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
            marker.mesh_resource = str(data['mesh'])
            marker.mesh_use_embedded_materials = False
            color = data.get('color', [0.72, 0.76, 0.84, 1.0])
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = float(color[3]) if len(color) > 3 else 1.0
            markers.append(marker)

        self.marker_pub.publish(MarkerArray(markers=markers))

    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GripperVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()