import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import csv
import os

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # TF buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Open CSV file
        home = os.path.expanduser("~")
        self.filepath = os.path.join(home, "pose_log.csv")
        self.csvfile = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csvfile)

        # CSV header
        self.writer.writerow(["time", "x", "y", "yaw"])

        # Timer
        self.timer = self.create_timer(0.05, self.log_pose)  # 20 Hz

        self.get_logger().info(f"Logging TF poses to {self.filepath}")

    def log_pose(self):
        try:
            # Try to get transform map -> base_link
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            # Extract translation
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Extract yaw from quaternion
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                             1.0 - 2.0 * (q.y*q.y + q.z*q.z))

            # Timestamp (seconds)
            t = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9

            # Write to CSV
            self.writer.writerow([t, x, y, yaw])

        except Exception as e:
            self.get_logger().warn("TF not available yet")

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
