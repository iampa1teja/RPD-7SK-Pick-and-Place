import math
import rclpy
from rclpy.node import Node

from msgs_pkg.msg import DetectionArray, Detection


class DetectionTransformer(Node):
    def __init__(self):
        super().__init__("detection_transformer")

        self.camera_translation = (0.0, -0.3, 0.35)

        roll = 0.0
        pitch = 0.7854
        yaw = 1.5708

        self.camera_quaternion = self.rpy_to_quaternion(roll, pitch, yaw)

        self.create_subscription(
            DetectionArray,
            "/cam_detections",
            self.detections_callback,
            10,
        )

        self.publisher = self.create_publisher(
            DetectionArray, "/detections", 10
        )
 
    def rpy_to_quaternion(self, r, p, y):
        cr = math.cos(r/2); sr = math.sin(r/2)
        cp = math.cos(p/2); sp = math.sin(p/2)
        cy = math.cos(y/2); sy = math.sin(y/2)

        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy

        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        return (qx/norm, qy/norm, qz/norm, qw/norm)

    def rotate(self, v):
        qx, qy, qz, qw = self.camera_quaternion
        x, y, z = v

        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)

        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)
        r12 = 2*(qy*qz - qx*qw)

        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx*qx + qy*qy)

        return (
            r00*x + r01*y + r02*z,
            r10*x + r11*y + r12*z,
            r20*x + r21*y + r22*z,
        )

    def transform(self, x, y, z):
        rx, ry, rz = self.rotate((x, y, z))
        tx, ty, tz = self.camera_translation
        return rx + tx, ry + ty, rz + tz

    def detections_callback(self, msg):
        out = DetectionArray()
        out.header = msg.header
        out.header.frame_id = "world"

        for d in msg.detections:
            wx, wy, wz = self.transform(d.x, d.y, d.z)

            nd = Detection()
            nd.class_name = d.class_name
            nd.class_id = d.class_id
            nd.confidence = d.confidence
            nd.x = float(wx)
            nd.y = float(wy)
            nd.z = float(wz)

            out.detections.append(nd)

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = DetectionTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()