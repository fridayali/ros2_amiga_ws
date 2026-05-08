#!/usr/bin/env python3
import argparse
import asyncio
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file


class CmdVelBridge(Node):
    def __init__(self, service_config_path: Path):
        super().__init__("amiga_cmd_vel_bridge")

        # Farm-ng client setup
        self.config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
        self.client: EventClient = EventClient(self.config)

        # Create and run asyncio event loop in separate thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

        self.twist_queue = asyncio.Queue()

        # Subscribe to ROS topic
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_vel_callback, 10)
        self.get_logger().info("✅ Subscribed to /cmd_vel and connected to Farm-ng /twist service")

        # Schedule the async processing task
        asyncio.run_coroutine_threadsafe(self.process_twists(), self.loop)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def cmd_vel_callback(self, msg: Twist):
        twist = Twist2d()
        twist.linear_velocity_x = float(msg.linear.x)
        twist.angular_velocity = float(msg.angular.z)
        self.get_logger().debug(f"Received /cmd_vel_nav: linear={twist.linear_velocity_x}, angular={twist.angular_velocity}")
        asyncio.run_coroutine_threadsafe(self.twist_queue.put(twist), self.loop)

    async def process_twists(self):
        while rclpy.ok():
            twist = await self.twist_queue.get()
            try:
                await self.client.request_reply("/twist", twist)
                self.get_logger().info(
                    f"→ Sent Twist to Amiga: linear={twist.linear_velocity_x:.3f}, angular={twist.angular_velocity:.3f}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send Twist: {e}")
            await asyncio.sleep(0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--service-config", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelBridge(args.service_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
