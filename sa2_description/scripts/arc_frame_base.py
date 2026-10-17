
#!/usr/bin/env python3

import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


ARC_NAMES = [
    "r3_arc_start",
    "r3_arc_mid",
    "r3_arc_end",
]


def quaternion_from_rpy(roll, pitch, yaw):
    """Convert roll, pitch, yaw to quaternion (x, y, z, w)."""
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


def quaternion_multiply(a, b):
    """Quaternion composition a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b

    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # q * [v, 0] * conjugate(q)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class ArcFramesBase(Node):

    def __init__(self):
        super().__init__("arc_frames_base")

        self.declare_parameter("yaml_file", "")
        yaml_file = self.get_parameter("yaml_file").value

        if not yaml_file:
            raise ValueError(
                "Set the yaml_file parameter to your workcell.yaml path"
            )

        with open(yaml_file, "r") as f:
            config = yaml.safe_load(f)

        workcell = config.get("workcell", config)
        arc_frames = workcell["arc_frames"]

        # Load existing world-relative arc poses.
        self.arcs = {}

        for name in ARC_NAMES:
            entry = arc_frames[name]

            if entry["parent"] != "world":
                raise ValueError(
                    f"{name} must have parent 'world' in the source YAML"
                )

            xyz = entry["xyz"]
            rpy = entry["rpy"]

            self.arcs[name] = (
                tuple(float(v) for v in xyz),
                quaternion_from_rpy(*[float(v) for v in rpy]),
            )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.done = False
        self.timer = self.create_timer(0.2, self.process_arcs)

        self.get_logger().info(
            "Waiting for world -> robot3_base_link TF..."
        )

    def process_arcs(self):
        if self.done:
            return

        try:
            # T_world_base: pose of robot3_base_link in world.
            tf = self.buffer.lookup_transform(
                "world",
                "robot3_base_link",
                Time(),
                timeout=Duration(seconds=0.5),
            )

        except Exception as exc:
            self.get_logger().info(
                f"Waiting for robot base TF: {exc}"
            )
            return

        t = tf.transform.translation
        r = tf.transform.rotation

        world_base_xyz = (t.x, t.y, t.z)
        q_world_base = (r.x, r.y, r.z, r.w)

        # Inverse of T_world_base.
        q_base_world = (
            -q_world_base[0],
            -q_world_base[1],
            -q_world_base[2],
            q_world_base[3],
        )

        transforms = []

        for name, (world_arc_xyz, q_world_arc) in self.arcs.items():

            # Translation:
            # p_base_arc = R_base_world * (p_world_arc - p_world_base)
            delta = tuple(
                world_arc_xyz[i] - world_base_xyz[i]
                for i in range(3)
            )

            base_arc_xyz = rotate_vector(q_base_world, delta)

            # Orientation:
            # q_base_arc = q_base_world * q_world_arc
            q_base_arc = quaternion_multiply(
                q_base_world, q_world_arc
            )

            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "robot3_base_link"
            msg.child_frame_id = name

            msg.transform.translation.x = base_arc_xyz[0]
            msg.transform.translation.y = base_arc_xyz[1]
            msg.transform.translation.z = base_arc_xyz[2]

            msg.transform.rotation.x = q_base_arc[0]
            msg.transform.rotation.y = q_base_arc[1]
            msg.transform.rotation.z = q_base_arc[2]
            msg.transform.rotation.w = q_base_arc[3]

            transforms.append(msg)

            self.get_logger().info(
                f"{name} relative to robot3_base_link:\n"
                f"  xyz = {[round(v, 6) for v in base_arc_xyz]}\n"
                f"  quaternion = "
                f"{[round(v, 6) for v in q_base_arc]}"
            )

        # Publish all three fixed transforms.
        self.static_broadcaster.sendTransform(transforms)

        self.done = True
        self.timer.cancel()

        self.get_logger().info(
            "All three arc frames published successfully."
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArcFramesBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
