"""
Task Manager Node — manages mission queues and sends navigation goals to Nav2.

Topics subscribed:
  /mission_segments               (std_msgs/String JSON) — farmng segment mission
  /task_manager/add_waypoint      (geometry_msgs/PoseStamped) — single waypoint (map frame)
  /task_manager/cancel            (std_msgs/Empty)            — cancel queue
  /task_manager/cancel_mission    (std_msgs/String)           — cancel running mission

Topics published:
  /task_manager/status        (std_msgs/String)    — state + queue size
  /task_manager/current_goal  (geometry_msgs/PoseStamped)
  /tool_cmd                   (std_msgs/String)    — plow_down / plow_up

Services:
  /task_manager/clear_queue   (std_srvs/Trigger)
  /task_manager/pause         (std_srvs/SetBool)
"""

import json
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL

from collections import deque
from enum import Enum, auto

# action_msgs/GoalStatus — SUCCEEDED = 4
NAV_SUCCEEDED = 4


class TaskState(Enum):
    IDLE      = auto()
    RUNNING   = auto()
    PAUSED    = auto()
    CANCELING = auto()


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__("task_manager")

        self._cb_group = ReentrantCallbackGroup()

        # Action client — Nav2
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_group)

        # Service client — robot_localization GPS → map
        self._fromll = self.create_client(
            FromLL, "/fromLL",
            callback_group=self._cb_group)

        # Publishers
        self._status_pub = self.create_publisher(String, "/task_manager/status", 10)
        self._goal_pub   = self.create_publisher(PoseStamped, "/task_manager/current_goal", 10)
        self._tool_pub   = self.create_publisher(String, "/tool_cmd", 10)

        # Subscribers
        self.create_subscription(
            String, "/mission_segments", self._cb_mission, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            String, "/task_manager/cancel_mission", self._cb_cancel_mission, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            PoseStamped, "/task_manager/add_waypoint", self._cb_add_waypoint, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Empty, "/task_manager/cancel", self._cb_cancel, 10,
            callback_group=self._cb_group)

        # Services
        self.create_service(Trigger, "/task_manager/clear_queue", self._srv_clear_queue)
        self.create_service(SetBool, "/task_manager/pause",       self._srv_pause)

        self._queue: deque[PoseStamped] = deque()
        self._state = TaskState.IDLE
        self._current_goal_handle = None
        self._nav_done_event: threading.Event | None = None

        self.create_timer(0.5, self._dispatch_timer_cb, callback_group=self._cb_group)

        self.get_logger().info("TaskManagerNode started.")

    # ------------------------------------------------------------------ #
    #  Segment mission                                                     #
    # ------------------------------------------------------------------ #

    def _cb_mission(self, msg: String):
        if self._state != TaskState.IDLE:
            self.get_logger().warn("Mission received but task manager is busy — ignoring.")
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid mission JSON: {e}")
            return

        segments = data.get("segments_json", [])
        if not segments:
            self.get_logger().warn("Mission has no segments.")
            return

        self.get_logger().info(
            f"Mission {data.get('mission_id')} received — "
            f"{len(segments)} segment(s).")

        self._state = TaskState.RUNNING
        self._publish_status()

        threading.Thread(
            target=self._execute_mission,
            args=(segments,),
            daemon=True,
        ).start()

    def _cb_cancel_mission(self, msg: String):
        if self._state == TaskState.RUNNING:
            self.get_logger().info("Mission cancel requested.")
            self._state = TaskState.CANCELING
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()
            if self._nav_done_event:
                self._nav_done_event.set()

    def _execute_mission(self, segments: list):
        for seg in sorted(segments, key=lambda s: s["order_index"]):
            if self._state == TaskState.CANCELING:
                self.get_logger().info("Mission canceled.")
                break

            action = seg.get("action")
            idx = seg.get("order_index")
            self.get_logger().info(f"Segment {idx}: {action}")

            if action == "move":
                lat = seg.get("latitude")
                lon = seg.get("longitude")
                if lat is None or lon is None:
                    self.get_logger().warn(f"Segment {idx}: missing lat/lon — skipping.")
                    continue
                ok = self._execute_move(lat, lon)
                if not ok and self._state != TaskState.CANCELING:
                    self.get_logger().error(f"Segment {idx}: navigation failed — aborting mission.")
                    break

            elif action in ("plow_down", "plow_up"):
                self._execute_tool(action)

            else:
                self.get_logger().warn(f"Segment {idx}: unknown action '{action}' — skipping.")

        self._state = TaskState.IDLE
        self._current_goal_handle = None
        self._publish_status()
        self.get_logger().info("Mission execution finished.")

    # ------------------------------------------------------------------ #
    #  Move: GPS → map → Nav2                                             #
    # ------------------------------------------------------------------ #

    def _execute_move(self, lat: float, lon: float) -> bool:
        pose = self._gps_to_pose(lat, lon)
        if pose is None:
            return False

        self._goal_pub.publish(pose)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return False

        nav_done = threading.Event()
        self._nav_done_event = nav_done
        nav_result = [None]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        def _on_goal(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().warn("Goal rejected by Nav2.")
                nav_done.set()
                return
            self._current_goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda rf: (nav_result.__setitem__(0, rf.result()), nav_done.set()))

        self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        ).add_done_callback(_on_goal)

        nav_done.wait()
        self._nav_done_event = None

        if nav_result[0] is None:
            return False
        return nav_result[0].status == NAV_SUCCEEDED

    def _gps_to_pose(self, lat: float, lon: float) -> PoseStamped | None:
        if not self._fromll.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/fromLL service not available.")
            return None

        req = FromLL.Request()
        req.ll_point.latitude  = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude  = 0.0

        done = threading.Event()
        result = [None]

        self._fromll.call_async(req).add_done_callback(
            lambda f: (result.__setitem__(0, f.result()), done.set()))

        done.wait(timeout=10.0)

        if result[0] is None:
            self.get_logger().error(f"/fromLL timed out for ({lat}, {lon}).")
            return None

        p = result[0].map_point
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    # ------------------------------------------------------------------ #
    #  Tool command                                                        #
    # ------------------------------------------------------------------ #

    def _execute_tool(self, action: str):
        msg = String()
        msg.data = action
        self._tool_pub.publish(msg)
        self.get_logger().info(f"Tool command: {action}")
        time.sleep(2.0)

    # ------------------------------------------------------------------ #
    #  Waypoint queue (existing functionality)                             #
    # ------------------------------------------------------------------ #

    def _cb_add_waypoint(self, msg: PoseStamped):
        self._queue.append(msg)
        self.get_logger().info(f"Waypoint enqueued. Queue: {len(self._queue)}")
        self._publish_status()

    def _cb_cancel(self, _: Empty):
        if self._current_goal_handle is not None:
            self.get_logger().info("Canceling current goal.")
            self._state = TaskState.CANCELING
            self._current_goal_handle.cancel_goal_async()
        self._queue.clear()
        self._publish_status()

    def _srv_clear_queue(self, _req, response):
        self._queue.clear()
        self.get_logger().info("Queue cleared.")
        response.success = True
        response.message = "Queue cleared."
        self._publish_status()
        return response

    def _srv_pause(self, req, response):
        if req.data:
            self._state = TaskState.PAUSED
            self.get_logger().info("Paused.")
        else:
            if self._state == TaskState.PAUSED:
                self._state = TaskState.IDLE
                self.get_logger().info("Resumed.")
        response.success = True
        response.message = f"State: {self._state.name}"
        self._publish_status()
        return response

    def _dispatch_timer_cb(self):
        if self._state != TaskState.IDLE or not self._queue:
            return
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            return
        self._send_goal(self._queue.popleft())

    def _send_goal(self, pose: PoseStamped):
        self._state = TaskState.RUNNING
        self._goal_pub.publish(pose)
        self._publish_status()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        ).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected.")
            self._state = TaskState.IDLE
            self._publish_status()
            return
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._current_goal_handle = None
        self.get_logger().info(f"Goal done. Status: {future.result().status}")
        self._state = TaskState.IDLE
        self._publish_status()

    def _feedback_cb(self, feedback_msg):
        self.get_logger().debug(
            f"Distance remaining: {feedback_msg.feedback.distance_remaining:.2f} m")

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _publish_status(self):
        msg = String()
        msg.data = f"state={self._state.name} queue_size={len(self._queue)}"
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
