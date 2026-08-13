#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# >> Team 3 音频中继 — ALSA扬声器驱动, 支持打断式播放

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AudioRelay(Node):
    """语音播报中继：解析码标方向并通过语音引擎输出。"""

    def __init__(self):
        super().__init__('audio_relay')

        self.announced = False
        self.pending_text = None
        self.retry_count = 0
        self.max_retries = 3

        self.speech_pub = self.create_publisher(String, '/speech_cmd', 10)

        self.qr_sub = self.create_subscription(
            String,
            '/barcode_raw',
            self.on_barcode_payload,
            10
        )

        self.flush_timer = self.create_timer(0.15, self.attempt_delivery)

        self.get_logger().info("语音中继已就绪，监听 /barcode_raw")

    def on_barcode_payload(self, msg: String):
        if self.announced:
            return

        raw = msg.data.strip()
        direction = None

        if raw == "ClockWise":
            direction = "顺时针方向"
        elif raw == "AntiClockWise":
            direction = "逆时针方向"
        else:
            try:
                num = int(raw)
                if 1 <= num <= 9999:
                    direction = "顺时针方向" if num % 2 == 1 else "逆时针方向"
            except ValueError:
                pass

        if direction is None:
            self.get_logger().warn(f"无法识别的码标值: {raw}")
            return

        self.pending_text = f"码标 {raw} {direction}"
        self.retry_count = 0
        self.attempt_delivery()

    def attempt_delivery(self):
        if self.pending_text is None:
            return
        if self.speech_pub.get_subscription_count() == 0:
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.get_logger().error("语音引擎未就绪，播报失败")
                self.pending_text = None
            return

        out = String()
        out.data = self.pending_text
        self.speech_pub.publish(out)

        self.announced = True
        self.get_logger().info(f"已播报: {out.data}")
        self.pending_text = None


def main(args=None):
    rclpy.init(args=args)
    node = AudioRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
