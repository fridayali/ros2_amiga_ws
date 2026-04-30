"""
Task Manager Node — manages mission queues and sends navigation goals to Nav2.

Topics subscribed:
  /task_manager/add_waypoint  (geometry_msgs/PoseStamped)  — enqueue a waypoint
  /task_manager/cancel        (std_msgs/Empty)             — cancel current mission

Topics published:
  /task_manager/status        (std_msgs/String)            — current task status
  /task_manager/current_goal  (geometry_msgs/PoseStamped)  — active navigation goal

Services:
  /task_manager/clear_queue   (std_srvs/Trigger)           — clear all queued tasks
  /task_manager/pause         (std_srvs/SetBool)           — pause/resume execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from collections import deque
from enum import Enum, auto


class TaskState(Enum):
    IDLE     = auto()
    RUNNING  = auto()
    PAUSED   = auto()
    CANCELING = auto()


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        self._cb_group = ReentrantCallbackGroup()

        # Action client for Nav2 NavigateToPose
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # Publishers
        self._status_pub = self.create_publisher(String, '/task_manager/status', 10)
        self._goal_pub   = self.create_publisher(PoseStamped, '/task_manager/current_goal', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/task_manager/add_waypoint',
                                 self._cb_add_waypoint, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Empty, '/task_manager/cancel',
                                 self._cb_cancel, 10,
                                 callback_group=self._cb_group)

        # Services
        self.create_service(Trigger,  '/task_manager/clear_queue', self._srv_clear_queue)
        self.create_service(SetBool,  '/task_manager/pause',       self._srv_pause)

        self._queue: deque[PoseStamped] = deque()
        self._state = TaskState.IDLE
        self._current_goal_handle = None

        # Timer: try to dispatch next task every 0.5 s when idle
        self.create_timer(0.5, self._dispatch_timer_cb, callback_group=self._cb_group)

        self.get_logger().info('TaskManagerNode started.')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cb_add_waypoint(self, msg: PoseStamped):
        self._queue.append(msg)
        self.get_logger().info(
            f'Waypoint enqueued. Queue length: {len(self._queue)}')
        self._publish_status()

    def _cb_cancel(self, _: Empty):
        if self._current_goal_handle is not None:
            self.get_logger().info('Canceling current navigation goal.')
            self._state = TaskState.CANCELING
            self._current_goal_handle.cancel_goal_async()
        self._queue.clear()
        self._publish_status()

    def _srv_clear_queue(self, _req, response):
        self._queue.clear()
        self.get_logger().info('Task queue cleared.')
        response.success = True
        response.message = 'Queue cleared.'
        self._publish_status()
        return response

    def _srv_pause(self, req, response):
        if req.data:
            self._state = TaskState.PAUSED
            self.get_logger().info('Task execution paused.')
        else:
            if self._state == TaskState.PAUSED:
                self._state = TaskState.IDLE
                self.get_logger().info('Task execution resumed.')
        response.success = True
        response.message = f'State: {self._state.name}'
        self._publish_status()
        return response

    # ------------------------------------------------------------------ #
    #  Dispatch logic                                                      #
    # ------------------------------------------------------------------ #

    def _dispatch_timer_cb(self):
        if self._state != TaskState.IDLE:
            return
        if not self._queue:
            return
        if not self._nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('NavigateToPose action server not available yet.')
            return

        goal_pose = self._queue.popleft()
        self._send_goal(goal_pose)

    def _send_goal(self, pose: PoseStamped):
        self._state = TaskState.RUNNING
        self._goal_pub.publish(pose)
        self._publish_status()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal: x={pose.pose.position.x:.2f} '
            f'y={pose.pose.position.y:.2f}')

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self._state = TaskState.IDLE
            self._publish_status()
            return
        self._current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self._current_goal_handle = None

        if self._state == TaskState.CANCELING:
            self.get_logger().info('Goal canceled.')
        else:
            self.get_logger().info(f'Goal reached. Result code: {future.result().status}')

        self._state = TaskState.IDLE
        self._publish_status()

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {dist:.2f} m')

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _publish_status(self):
        msg = String()
        msg.data = (
            f'state={self._state.name} '
            f'queue_size={len(self._queue)}'
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
