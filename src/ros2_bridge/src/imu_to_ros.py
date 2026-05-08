"""Subscribe to Oak IMU messages and publish as ROS2 sensor_msgs/Imu."""
from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file


class ImuPublisher(Node):
    def __init__(self, service_config: EventServiceConfig):
        super().__init__("imu_publisher")
        self.pub = self.create_publisher(Imu, "/imu/data", 10)
        self.service_config = service_config

    async def run(self):
        client = EventClient(self.service_config)

        # IMU subscription
        imu_sub = None
        for sub in self.service_config.subscriptions:
            path = getattr(sub, "topic_uri", getattr(sub, "uri", None))
            if path and "imu" in path.path.lower():
                imu_sub = sub
                break

        if imu_sub is None:
            self.get_logger().error("No IMU subscription found in service config!")
            return

        async for event, message in client.subscribe(imu_sub, decode=True):
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Varsayılan sıfır
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

            # Packets içinden gyro ve accelero değerlerini al
            if hasattr(message, "packets"):
                for packet in message.packets:
                    if hasattr(packet, "gyro_packet") and hasattr(packet.gyro_packet, "gyro"):
                        imu_msg.angular_velocity.x = packet.gyro_packet.gyro.x
                        imu_msg.angular_velocity.y = packet.gyro_packet.gyro.y
                        imu_msg.angular_velocity.z = packet.gyro_packet.gyro.z
                    if hasattr(packet, "accelero_packet") and hasattr(packet.accelero_packet, "accelero"):
                        imu_msg.linear_acceleration.x = packet.accelero_packet.accelero.z
                        imu_msg.linear_acceleration.y = packet.accelero_packet.accelero.y
                        imu_msg.linear_acceleration.z = packet.accelero_packet.accelero.x

            self.pub.publish(imu_msg)


def main():
    parser = argparse.ArgumentParser(description="Publish Oak IMU to ROS2 /imu/data")
    parser.add_argument(
        "--service-config",
        type=Path,
        required=True,
        help="Path to the IMU service config JSON file.",
    )
    args = parser.parse_args()

    rclpy.init()
    config: EventServiceConfig = proto_from_json_file(args.service_config, EventServiceConfig())
    node = ImuPublisher(config)

    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
