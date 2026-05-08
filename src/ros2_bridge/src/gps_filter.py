#!/usr/bin/env python3
import argparse
import asyncio
from pathlib import Path
from math import radians

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.gps import gps_pb2
from farm_ng_core_pybind import Pose3F64


class MultiClientSubscriber(Node):
    """Example of subscribing to events from multiple clients and publishing to ROS2."""

    def __init__(self, service_config: EventServiceConfigList) -> None:
        super().__init__("multi_client_subscriber_ros")

        self.service_config = service_config
        self._clients: dict[str, EventClient] = {}   # <-- Değiştirildi
        self._subscriptions = []                     # <-- Değiştirildi
        self.orientation: float = None
        self.pose: Pose3F64 = None

        # ROS2 publishers
        self.pub_fix = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/gps/pose", 10)

        # GPS cache
        self.last_lat = 0.0
        self.last_lon = 0.0
        self.last_alt = 0.0
        self.last_yaw = 0.0

        # Populate the event clients
        config: EventServiceConfig
        for config in self.service_config.configs:
            if not config.port:
                self._subscriptions = config.subscriptions  # <-- Değiştirildi
                continue
            self._clients[config.name] = EventClient(config)

    async def _subscribe(self, subscription: SubscribeRequest) -> None:
        client_name: str = subscription.uri.query.split("=")[-1]
        client: EventClient = self._clients[client_name]

        async for event, message in client.subscribe(subscription, decode=True):
            if client_name == "gps" and isinstance(message, gps_pb2.GpsFrame):
                self.last_lat = message.latitude
                self.last_lon = message.longitude
                self.last_alt = message.altitude
                # Eğer GPS verisinde headingMotion yoksa, filter orientation'ı kullan
                gps_yaw = getattr(message, "headingMotion", None)
                if gps_yaw is not None and gps_yaw != 0.0:
                    self.last_yaw = gps_yaw
                elif self.orientation is not None:
                    self.last_yaw = self.orientation  # filter'dan gelen yaw
                fix_ok = getattr(message, "gnss_fix_ok", True)

                # NavSatFix publish
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = "gps_link"
                gps_msg.latitude = self.last_lat
                gps_msg.longitude = self.last_lon
                gps_msg.altitude = self.last_alt
                gps_msg.status.status = (
                    NavSatStatus.STATUS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
                )
                gps_msg.status.service = NavSatStatus.SERVICE_GPS
                self.pub_fix.publish(gps_msg)

                # PoseStamped publish
                pose_msg = PoseStamped()
                pose_msg.header = gps_msg.header
                pose_msg.pose.position.x = self.last_lat
                pose_msg.pose.position.y = self.last_lon
                pose_msg.pose.position.z = self.last_alt
                q = quaternion_from_euler(0, 0, radians(self.last_yaw))
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]
                self.pub_pose.publish(pose_msg)

                print(f"[GPS] {self.last_lat:.6f}, {self.last_lon:.6f}, {self.last_alt:.2f}, yaw={self.last_yaw:.2f}")

            elif client_name == "filter":
                if hasattr(message, "pose"):
                    self.pose: Pose3F64 = Pose3F64.from_proto(message.pose)
                if hasattr(message, "heading"):
                    self.orientation = message.heading

                if self.pose is not None and self.orientation is not None:
                    # Odometry mesajı oluştur
                    from nav_msgs.msg import Odometry
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = "map"
                    odom_msg.child_frame_id = "base_link"

                    # Pozisyon (Kartezyen koordinatlar)
                    odom_msg.pose.pose.position.x = self.pose.translation[0]
                    odom_msg.pose.pose.position.y = self.pose.translation[1]
                    odom_msg.pose.pose.position.z = self.pose.translation[2]

                    # Yönelim (Heading -> quaternion)
                    q = quaternion_from_euler(0, 0, self.orientation)
                    print(self.orientation)
                    print(q)
                    odom_msg.pose.pose.orientation.x = q[0]
                    odom_msg.pose.pose.orientation.y = q[1]
                    odom_msg.pose.pose.orientation.z = q[2]
                    odom_msg.pose.pose.orientation.w = q[3]
                    odom_msg.pose.covariance = [
                        0.05, 0, 0, 0, 0, 0,
                        0, 0.05, 0, 0, 0, 0,
                        0, 0, 0.05, 0, 0, 0,
                        0, 0, 0, 0.1, 0, 0,
                        0, 0, 0, 0, 0.1, 0,
                        0, 0, 0, 0, 0, 0.1
                    ]

                    # Yayınla
                    if not hasattr(self, "pub_odom"):
                        self.pub_odom = self.create_publisher(Odometry, "/rtk/odom", 10)
                    self.pub_odom.publish(odom_msg)

                    print(
                        f"[FILTER→ODOM] x={self.pose.translation[0]:.3f}, "
                        f"y={self.pose.translation[1]:.3f}, "
                        f"yaw={self.orientation:.3f}"
        )
    async def run(self) -> None:
        tasks: list[asyncio.Task] = []
        for subscription in self._subscriptions:   # <-- Değiştirildi
            tasks.append(asyncio.create_task(self._subscribe(subscription)))
        await asyncio.gather(*tasks)


async def main_async(config_path: Path):
    service_config = proto_from_json_file(config_path, EventServiceConfigList())
    node = MultiClientSubscriber(service_config)
    await node.run()


def main():
    parser = argparse.ArgumentParser(description="Farm-ng GPS/Filter ROS2 bridge")
    parser.add_argument("--config", type=Path, required=True, help="Path to config.json")
    args = parser.parse_args()

    rclpy.init()

    try:
        asyncio.run(main_async(args.config))
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
