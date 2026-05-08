#!/usr/bin/env python3
import asyncio
import json
import math
import threading
import time

import requests
import websockets
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String

ROBOT_ID = 6
BASE_URL = "https://backend.agrobrain.com.tr"
REGISTER_URL = f"{BASE_URL}/amiga/register"
ROBOT_PAYLOAD = {
    "robot_id": ROBOT_ID,
    "sensor_states": ["GNSS", "LIDAR", "RGB_CAMERA1", "RGB_CAMERA2", "DEPTH_CAMERA", "SOIL_SENSOR"],
}
RECONNECT_DELAY = 5.0
TELEMETRY_INTERVAL = 2.0


class WebSocketBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

        self._lock = threading.Lock()
        self._telemetry = {
            "robot_id": ROBOT_ID,
            "lat": None,
            "lon": None,
            "heading": None,
            "motor_temps": [None, None, None, None],
            "battery_pct": None,
        }

        # Subscribers
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(Odometry, "/rtk/odom", self._heading_cb, 10)
        self.create_subscription(Float32MultiArray, "/motor_state", self._motor_cb, 10)
        self.create_subscription(BatteryState, "/battery_state", self._battery_cb, 10)

        # Publishers
        self._mission_pub = self.create_publisher(String, "/mission_segments", 10)
        self._goal_name_pub = self.create_publisher(String, "/goal_name", 10)
        self._cancel_pub = self.create_publisher(String, "/task_manager/cancel_mission", 10)

        # Asyncio loop in background thread
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.run(), self._loop)

        self.get_logger().info("WebSocketBridge initialized.")

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    # ------------------------------------------------------------------ #
    #  ROS Callbacks                                                       #
    # ------------------------------------------------------------------ #

    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self._telemetry["lat"] = msg.latitude
            self._telemetry["lon"] = msg.longitude

    def _heading_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        with self._lock:
            self._telemetry["heading"] = math.degrees(math.atan2(siny, cosy))

    def _motor_cb(self, msg: Float32MultiArray):
        with self._lock:
            self._telemetry["motor_temps"] = list(msg.data[:4])

    def _battery_cb(self, msg: BatteryState):
        pct = int((float(msg.percentage) - 40.0) / 10.0 * 100.0)
        with self._lock:
            self._telemetry["battery_pct"] = max(0, min(100, pct))

    # ------------------------------------------------------------------ #
    #  Registration                                                        #
    # ------------------------------------------------------------------ #

    def _register(self, retries: int = 5, delay: float = 2.0):
        for attempt in range(1, retries + 1):
            try:
                r = requests.post(REGISTER_URL, json=ROBOT_PAYLOAD, timeout=5)
                r.raise_for_status()
                ws_url = r.json().get("websocket_url")
                self.get_logger().info(f"Registered. WS URL: {ws_url}")
                return ws_url
            except requests.RequestException as e:
                self.get_logger().warn(f"Register attempt {attempt}/{retries} failed: {e}")
                if attempt < retries:
                    time.sleep(delay)
        return None

    # ------------------------------------------------------------------ #
    #  Async WebSocket                                                     #
    # ------------------------------------------------------------------ #

    async def _send_telemetry(self, ws):
        while True:
            with self._lock:
                payload = self._telemetry.copy()
            await ws.send(json.dumps(payload))
            await asyncio.sleep(TELEMETRY_INTERVAL)

    async def _receive_commands(self, ws):
        async for raw in ws:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"Invalid JSON: {e}")
                continue
            self.get_logger().info(f"Command received: {data}")
            self._route(data)

    def _route(self, data: dict):
        goal = data.get("goal")

        if goal == "execute_farmng_mission":
            msg = String()
            msg.data = json.dumps(data)
            self._mission_pub.publish(msg)
            self.get_logger().info(
                f"Mission {data.get('mission_id')} forwarded to task_manager.")

        elif goal == "cancel_mission":
            msg = String()
            msg.data = "cancel"
            self._cancel_pub.publish(msg)
            self.get_logger().info("Cancel request forwarded.")

        elif goal is not None:
            msg = String()
            msg.data = str(goal)
            self._goal_name_pub.publish(msg)

    async def run(self):
        while rclpy.ok():
            ws_url = await asyncio.get_event_loop().run_in_executor(None, self._register)
            if not ws_url:
                self.get_logger().error(
                    f"Registration failed. Retrying in {RECONNECT_DELAY}s...")
                await asyncio.sleep(RECONNECT_DELAY)
                continue
            try:
                async with websockets.connect(ws_url) as ws:
                    await ws.send(json.dumps(ROBOT_PAYLOAD))
                    self.get_logger().info("WebSocket connected.")
                    await asyncio.gather(
                        self._send_telemetry(ws),
                        self._receive_commands(ws),
                    )
            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                self.get_logger().warn(f"WS disconnected: {e}. Reconnecting in {RECONNECT_DELAY}s...")
            except Exception as e:
                self.get_logger().error(f"WS error: {e}. Reconnecting in {RECONNECT_DELAY}s...")
            await asyncio.sleep(RECONNECT_DELAY)


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
