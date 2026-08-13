#!/usr/bin/env python3
"""3队分层任务编排器：用航向累计确认环行，并直接连接底盘入口。"""
import math
import time
from enum import IntEnum

import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class Level(IntEnum):
    WAIT_ORDER = 0
    SELECT_BRANCH = 1
    PASS_GATE_IN = 2
    ORBIT_C = 3
    PASS_GATE_OUT = 4
    SEEK_P = 5
    PARKED = 6
    DEADLINE = 7


class StageOrchestrator(Node):
    def __init__(self):
        super().__init__('charlie_stage_orchestrator')
        self.declare_parameter('hard_deadline_sec', 180.0)
        self.declare_parameter('required_heading_rad', 5.2)
        self.declare_parameter('return_radius', 0.18)
        self.declare_parameter('branch_duration', 1.05)
        self.declare_parameter('branch_rate', 0.70)
        self.declare_parameter('track_center_x', 320.0)
        self.declare_parameter('track_speed', 0.55)
        self.declare_parameter('track_gain', 0.0055)
        self.limit = float(self.get_parameter('hard_deadline_sec').value)
        self.heading_needed = float(self.get_parameter('required_heading_rad').value)
        self.return_radius = float(self.get_parameter('return_radius').value)
        self.branch_duration = float(self.get_parameter('branch_duration').value)
        self.branch_rate = float(self.get_parameter('branch_rate').value)
        self.track_center_x = float(self.get_parameter('track_center_x').value)
        self.track_speed = float(self.get_parameter('track_speed').value)
        self.track_gain = float(self.get_parameter('track_gain').value)

        self.drive_pub = self.create_publisher(Twist, '/medibot_cmd_vel', 10)
        self.snap_pub = self.create_publisher(Int32, '/snap_request', 10)
        self.track_switch_pub = self.create_publisher(String, '/track_switch', 10)
        self.stage_pub = self.create_publisher(String, '/charlie_stage', 10)
        self.speech_pub = self.create_publisher(String, '/speech_cmd', 10)
        self.display_pub = self.create_publisher(String, '/display_text', 10)
        self.create_subscription(PerceptionTargets, '/lane_center',
                                 self._lane_center, 10)
        self.create_subscription(Twist, '/drive_avoid_cmd', self._avoid, 10)
        self.create_subscription(Bool, '/avoid_active', self._avoid_flag, 10)
        self.create_subscription(Int32, '/barcode_id', self._barcode, 10)
        self.create_subscription(PerceptionTargets, '/vision_guard_result',
                                 self._vision, 10)
        self.create_subscription(String, '/scene_text', self._scene_text, 10)
        self.create_subscription(String, '/stop_point', self._stop_point, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)

        self.level = Level.WAIT_ORDER
        self.started = time.monotonic()
        self.direction = 0.0
        self.branch_until = 0.0
        self.track_cmd = Twist()
        self.avoid_cmd = Twist()
        self.track_time = 0.0
        self.avoid_time = 0.0
        self.avoid_enabled = False
        self.gate_visible = False
        self.person_requested = False
        self.description_ready = False
        self.previous_yaw = None
        self.signed_heading = 0.0
        self.origin = None
        self.pose = None
        self.stop_latched = False
        self.create_timer(0.04, self._cycle)
        self._announce_level()
        self._publish_track_mode('yellow')

    def _advance(self, level):
        if level == self.level:
            return
        self.get_logger().info(f'level {self.level.name} -> {level.name}')
        self.level = level
        self._announce_level()
        self._publish_track_mode('off' if level in
                                 (Level.PARKED, Level.DEADLINE) else 'yellow')

    def _publish_track_mode(self, value):
        mode = String()
        mode.data = value
        self.track_switch_pub.publish(mode)

    def _announce_level(self):
        msg = String()
        msg.data = self.level.name
        self.stage_pub.publish(msg)

    def _lane_center(self, msg):
        try:
            point = msg.targets[0].points[0].point[0]
            error = self.track_center_x - float(point.x)
        except (IndexError, AttributeError, TypeError):
            return
        cmd = Twist()
        cmd.linear.x = self.track_speed
        cmd.angular.z = max(-1.2, min(1.2, error * self.track_gain))
        self.track_cmd, self.track_time = cmd, time.monotonic()

    def _avoid(self, msg):
        self.avoid_cmd, self.avoid_time = msg, time.monotonic()

    def _avoid_flag(self, msg):
        self.avoid_enabled = bool(msg.data)

    def _barcode(self, msg):
        if self.level != Level.WAIT_ORDER or msg.data not in (3, 4):
            return
        self.direction = -1.0 if msg.data == 3 else 1.0
        self.branch_until = time.monotonic() + self.branch_duration
        self._advance(Level.SELECT_BRANCH)

    @staticmethod
    def _labels(msg):
        return {str(getattr(obj, 'type', '')).lower() for obj in msg.targets}

    def _vision(self, msg):
        labels = self._labels(msg)
        gate = 'tongdao' in labels
        if self.level == Level.SELECT_BRANCH and gate:
            self._advance(Level.PASS_GATE_IN)
        elif self.level == Level.PASS_GATE_IN and self.gate_visible and not gate:
            self.signed_heading = 0.0
            self._advance(Level.ORBIT_C)
        elif self.level == Level.ORBIT_C:
            if labels.intersection({'person', 'tuwen'}) and not self.person_requested:
                trigger = Int32()
                trigger.data = 1
                self.snap_pub.publish(trigger)
                self.person_requested = True
            correct_rotation = self.direction * self.signed_heading
            if gate and not self.gate_visible and correct_rotation >= self.heading_needed \
                    and self.description_ready:
                self._advance(Level.PASS_GATE_OUT)
        elif self.level == Level.PASS_GATE_OUT and self.gate_visible and not gate:
            self._advance(Level.SEEK_P)
        self.gate_visible = gate

    def _scene_text(self, msg):
        text = msg.data.strip()
        if self.level in (Level.ORBIT_C, Level.PASS_GATE_OUT) and text:
            self.description_ready = True
            relay = String()
            relay.data = text
            self.speech_pub.publish(relay)
            self.display_pub.publish(relay)

    def _stop_point(self, msg):
        if self.level == Level.SEEK_P and msg.data.strip().lower() in ('p', 'park', '1'):
            self._park()

    @staticmethod
    def _yaw(orientation):
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny, cosy)

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        if self.origin is None:
            self.origin = self.pose
        yaw = self._yaw(msg.pose.pose.orientation)
        if self.previous_yaw is not None and self.level == Level.ORBIT_C:
            delta = math.atan2(math.sin(yaw - self.previous_yaw),
                               math.cos(yaw - self.previous_yaw))
            if abs(delta) < 0.5:
                self.signed_heading += delta
        self.previous_yaw = yaw
        if self.level == Level.SEEK_P and self.origin is not None:
            if math.hypot(self.pose[0] - self.origin[0],
                          self.pose[1] - self.origin[1]) <= self.return_radius:
                self._park()

    def _park(self):
        self.stop_latched = True
        self._advance(Level.PARKED)

    def _selected_command(self, now):
        if now < self.branch_until:
            cmd = Twist()
            cmd.linear.x = 0.15
            cmd.angular.z = self.direction * self.branch_rate
            return cmd
        if self.avoid_enabled and now - self.avoid_time < 0.25:
            return self.avoid_cmd
        if now - self.track_time < 0.25:
            return self.track_cmd
        return Twist()

    def _cycle(self):
        now = time.monotonic()
        if not self.stop_latched and now - self.started >= self.limit:
            self.stop_latched = True
            self._advance(Level.DEADLINE)
        self.drive_pub.publish(Twist() if self.stop_latched
                               else self._selected_command(now))


def main(args=None):
    rclpy.init(args=args)
    node = StageOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_latched = True
        for _ in range(5):
            node.drive_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
