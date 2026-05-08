#!/usr/bin/env python3
"""
Farm-ng Kamera Stream + ROS2 Bridge:
- Farm-ng kamera stream'ini okur
- /camera_state topic'ini dinler
- camera1:on ise görüntüyü /camera1 topic'ine yayınlar
"""

from __future__ import annotations
import argparse
import asyncio
from pathlib import Path
import json
import cv2
import numpy as np

# FARM-NG
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class CameraRosBridge(Node):
    def __init__(self):
        super().__init__("camera_ros_bridge")

        # Kamera durumu
        self.camera1_on = True

        # Subscriber
        self.create_subscription(String, "camera_state", self.camera_state_callback, 10)

        # Publisher
        self.cam1_pub = self.create_publisher(Image, "/camera1", 10)

        # CV → ROS dönüşümü
        self.bridge = CvBridge()

        self.get_logger().info("CameraRosBridge initialized.")

    def camera_state_callback(self, msg: String):
        """Example message: {"RGB_CAMERA1_STREAM":"on", "RGB_CAMERA2_STREAM":"off", "MULTISPECTRAL_CAMERA":"get"}"""
        try:
            state = json.loads(msg.data)
            cam1_state = state.get("camera1", "off")
            self.camera1_on = (cam1_state == "on")

            self.get_logger().info(
                f"Received camera_state: camera1={cam1_state} → {self.camera1_on}"
            )

        except Exception as e:
            self.get_logger().error(f"Invalid camera_state JSON: {msg.data}")


class CameraClient:
    def __init__(self, service_config_path: Path, ros_node: CameraRosBridge):
        # Farm-ng event client'ini başlat
        config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
        self.client = EventClient(config)
        self.subscription = config.subscriptions[0]
        self.ros_node = ros_node

    async def start(self):
        """Farm-ng kamera stream'ini alıp ROS2'ye yayınlamak"""
        async for event, message in self.client.subscribe(self.subscription, decode=True):
            # Eğer gelen mesaj kamera verisi değilse, atla
            if not hasattr(message, 'image_data'):
                continue

            # ---- IMAGE DECODE ----
            image = cv2.imdecode(np.frombuffer(message.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)
            if event.uri.path == "/disparity":
                image = cv2.applyColorMap(image * 3, cv2.COLORMAP_JET)

            # ---- CAMERA1 ON ise ROS2'ye yayınla ----
            if self.ros_node.camera1_on:
                img_msg = self.ros_node.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.ros_node.cam1_pub.publish(img_msg)
                self.ros_node.get_logger().info("Published frame to /camera1")
            # cv2.imshow("oak1",image)
            # cv2.waitKey(1)
            # ROS2'nin diğer işlemleri için event loop'unu çalıştır
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)


async def main(service_config_path: Path) -> None:
    rclpy.init()
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
    ros_node = CameraRosBridge()

    # Kamera client'ini başlat
    camera_client = CameraClient(service_config_path, ros_node)

    try:
        # Kamera client'ini çalıştırmak için asyncio'yu kullanıyoruz
        await camera_client.start()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Farm-ng camera + ROS2 bridge")
    parser.add_argument("--service-config", type=Path, required=True,
                        help="Path to camera service config JSON")
    args = parser.parse_args()

    # Ana fonksiyonu asyncio ile çalıştırıyoruz
    asyncio.run(main(args.service_config))
