import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf_transformations import euler_matrix

from msgs_pkg.msg import Detection, DetectionArray


class DetectionTransformer(Node):
    def __init__(self):
        super().__init__("detection_transformer")

        self.declare_parameter("input_topic", "/cam_detections")
        self.declare_parameter("output_topic", "/detections")
        self.declare_parameter("imu_topic", "/camera/camera/imu")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("mount_roll_offset", math.pi)
        self.declare_parameter("mount_pitch_offset", math.pi)
        self.declare_parameter("mount_yaw_offset", 0.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.mount_roll_offset = self.get_parameter("mount_roll_offset").get_parameter_value().double_value
        self.mount_pitch_offset = self.get_parameter("mount_pitch_offset").get_parameter_value().double_value
        self.mount_yaw_offset = self.get_parameter("mount_yaw_offset").get_parameter_value().double_value

        # Fixed camera translation in meters: x=50cm, y=0, z=10cm
        self.camera_x = 0.50
        self.camera_y = 0.00
        self.camera_z = 0.10

        # Orientation policy: roll/pitch from IMU, yaw fixed to 0.
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.has_imu_orientation = False
        self.last_invalid_imu_warn_time_sec = -1.0

        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(DetectionArray, input_topic, self.detections_callback, 10)
        self.publisher = self.create_publisher(DetectionArray, output_topic, 10)

        self.get_logger().info(
            f"Transform node ready. Input: {input_topic}, IMU: {imu_topic}, Output: {output_topic}, "
            f"Frame: {self.target_frame}, MountRPY: ({self.mount_roll_offset:.3f}, "
            f"{self.mount_pitch_offset:.3f}, {self.mount_yaw_offset:.3f})"
        )

    def quaternion_to_rpy(self, qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def accel_optical_to_roll_pitch(self, ax, ay, az):
        # IMU frame is optical-like; convert to camera-link convention first.
        ax_cam = az
        ay_cam = -ax
        az_cam = -ay

        # Accelerometer direction is opposite of gravity in this stream,
        # so roll/pitch signs are inverted from the naive formula.
        roll = math.atan2(-ay_cam, az_cam)
        pitch = math.atan2(ax_cam, math.sqrt(ay_cam * ay_cam + az_cam * az_cam))
        return roll, pitch

    def warn_invalid_imu_throttled(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_invalid_imu_warn_time_sec < 0.0 or now_sec - self.last_invalid_imu_warn_time_sec >= 2.0:
            self.get_logger().warn(
                "IMU has no valid orientation and invalid acceleration; keeping previous roll/pitch"
            )
            self.last_invalid_imu_warn_time_sec = now_sec

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        has_orientation = (
            len(msg.orientation_covariance) > 0
            and msg.orientation_covariance[0] >= 0.0
            and q_norm >= 1e-9
        )

        if has_orientation:
            qx /= q_norm
            qy /= q_norm
            qz /= q_norm
            qw /= q_norm
            self.roll, self.pitch, _ = self.quaternion_to_rpy(qx, qy, qz, qw)
        else:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            a_norm = math.sqrt(ax * ax + ay * ay + az * az)
            has_accel = (
                len(msg.linear_acceleration_covariance) > 0
                and msg.linear_acceleration_covariance[0] >= 0.0
                and a_norm >= 1e-6
            )
            if not has_accel:
                self.warn_invalid_imu_throttled()
                return

            self.roll, self.pitch = self.accel_optical_to_roll_pitch(ax, ay, az)

        # Requirement: yaw fixed at 0.
        self.yaw = 0.0
        self.has_imu_orientation = True

    def camera_to_base(self, x_cam, y_cam, z_cam):
        total_roll = self.mount_roll_offset + self.roll
        total_pitch = self.mount_pitch_offset + self.pitch
        total_yaw = self.mount_yaw_offset + self.yaw

        transform = euler_matrix(total_roll, total_pitch, total_yaw)
        transform[:3, 3] = [self.camera_x, self.camera_y, self.camera_z]

        point_cam = np.array([x_cam, y_cam, z_cam, 1.0])
        point_base = transform @ point_cam
        return float(point_base[0]), float(point_base[1]), float(point_base[2])

    def detections_callback(self, msg):
        if not self.has_imu_orientation:
            self.get_logger().warn("Waiting for IMU orientation; skipping detection transform")
            return

        out = DetectionArray()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        for det in msg.detections:
            x_base, y_base, z_base = self.camera_to_base(det.x, det.y, det.z)

            out_det = Detection()
            out_det.class_name = det.class_name
            out_det.class_id = det.class_id
            out_det.confidence = det.confidence
            out_det.x = x_base
            out_det.y = y_base
            out_det.z = z_base
            out.detections.append(out_det)

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = DetectionTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()