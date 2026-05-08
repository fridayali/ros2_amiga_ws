# odometry.py
import argparse
import asyncio
from pathlib import Path
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf_transformations import quaternion_from_euler

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file


class OdomPublisher(Node):
    def __init__(self, service_config: EventServiceConfig, log_file: Path):
        super().__init__("odometry_publisher")
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.service_config = service_config
        self.log_file = log_file

        # CSV başlığı
        with open(self.log_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_sec", "time_nanosec", "linear_velocity_x", "angular_velocity"])

    async def run(self):
        client = EventClient(self.service_config)
        subscription = self.service_config.subscriptions[0]

        self.get_logger().info(f"Listening to: {subscription.uri}")
        async for event, message in client.subscribe(subscription, decode=True):
            # Mesaj tipi kontrolü
            if not isinstance(message, Twist2d):
                self.get_logger().warn(f"Gelen mesaj tipi beklenenden farklı: {type(message)}")
                continue

            # Twist2d alanlarını oku (snake_case!)
            linear_x = message.linear_velocity_x
            angular_z = message.angular_velocity

            # Odometry mesajı oluştur
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.twist.twist = Twist()
            odom_msg.twist.twist.linear.x = linear_x
            odom_msg.twist.twist.angular.z = angular_z

            # Orientation sabit (yaw = 0)
            q = quaternion_from_euler(0, 0, 0)
            odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # Yayınla
            self.pub.publish(odom_msg)

            # CSV log kaydı
            t = odom_msg.header.stamp
            with open(self.log_file, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([t.sec, t.nanosec, linear_x, angular_z])

            # Konsola bilgi
            self.get_logger().info(
                f"📡 linear_velocity_x: {linear_x:.3f} m/s | angular_velocity: {angular_z:.3f} rad/s"
            )


def main():
    parser = argparse.ArgumentParser(description="Publish odometry from Twist2d messages.")
    parser.add_argument(
        "--service-config",
        type=Path,
        required=True,
        help="Path to the motor service config JSON",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default="velocity_log.csv",
        help="Path to CSV log file",
    )
    args = parser.parse_args()

    rclpy.init()
    config: EventServiceConfig = proto_from_json_file(args.service_config, EventServiceConfig())
    node = OdomPublisher(config, args.log_file)

    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
