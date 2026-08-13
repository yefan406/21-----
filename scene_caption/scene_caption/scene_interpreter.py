# >> Team 3 场景解释器 — 火山引擎 doubao-vision-pro-32k 视觉语言模型调用
import base64
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from hbm_img_msgs.msg import HbmMsg1080P
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from volcenginesdkarkruntime import Ark


class SceneInterpreter(Node):
    def __init__(self):
        super().__init__("scene_interpreter")

        self.declare_parameter("save_scene_picture", False)
        self.declare_parameter("scene_picture_dir", "/tmp/scene_captures")
        self.declare_parameter("margin_scene", 0.2)
        self.declare_parameter("scene_jpeg_quality", 90)
        self.declare_parameter(
            "volc_base_url", "https://api.volcengine.com/vision/v1"
        )
        self.declare_parameter("volc_model", "doubao-vision-pro-32k")
        self.declare_parameter("scene_prompt",
            "观察图中出现的医用指示标牌人物，归纳其体态、着装及动作特征。",)
        self.declare_parameter("vlm_timeout", 25.0)
        self.declare_parameter("vlm_max_retries", 1)

        self.save_scene_picture = self.get_parameter("save_scene_picture").value
        self.scene_picture_dir = self.get_parameter("scene_picture_dir").value
        self.margin_scene = max(
            0.0, float(self.get_parameter("margin_scene").value)
        )
        self.scene_jpeg_quality = max(
            1,
            min(100, int(self.get_parameter("scene_jpeg_quality").value)),
        )
        self.volc_model = self.get_parameter("volc_model").value
        self.scene_prompt = self.get_parameter("scene_prompt").value
        self.vlm_timeout = float(self.get_parameter("vlm_timeout").value)
        self.vlm_max_retries = int(self.get_parameter("vlm_max_retries").value)

        self.api_key = os.getenv("ARK_API_KEY") or os.getenv("VOLC_ACCESS_KEY") or ""
        self.volc_base_url = self.get_parameter("volc_base_url").value

        # NO_PROXY handling with a different approach
        no_proxy_host = "api.volcengine.com"
        for var_name in ("NO_PROXY", "no_proxy"):
            existing = os.environ.get(var_name, "")
            if no_proxy_host not in existing.split(","):
                os.environ[var_name] = (
                    f"{existing},{no_proxy_host}" if existing else no_proxy_host
                )

        self.client = Ark(
            base_url=self.volc_base_url,
            api_key=self.api_key,
            timeout=self.vlm_timeout,
            max_retries=self.vlm_max_retries,
        )

        self.qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.raw_video_sub = None
        self.snap_request_sub = self.create_subscription(
            Int32,
            "/snap_request",
            self.snap_request_callback,
            qos_reliable,
        )
        self.vision_guard_sub = self.create_subscription(
            PerceptionTargets,
            "/vision_guard_result",
            self.vision_guard_callback,
            qos_reliable,
        )

        self.text_pub = self.create_publisher(String, "/scene_text", qos_reliable)
        self.display_pub = self.create_publisher(String, "/display_text", qos_reliable)
        self.picture_pub = self.create_publisher(
            CompressedImage, "/scene_image", qos_reliable
        )
        self.picture_path_pub = self.create_publisher(
            String, "/scene_image_path", qos_reliable
        )

        self.lock = threading.Lock()
        self.latest_scene_roi = None
        self.capture_pending = False
        self.capture_started_at = 0.0
        self.capture_frame_timeout = 1.0
        self.is_calling_vlm = False
        self.picture_count = 0
        self.capture_timeout_timer = self.create_timer(
            0.1, self.capture_timeout_callback
        )

        os.makedirs(self.scene_picture_dir, exist_ok=True)
        self.get_logger().info(
            f"场景识别已就绪: 引擎={self.volc_model}, "
            f"来源=/nv12_img(按需), 画质={self.scene_jpeg_quality}"
        )

    def raw_video_callback(self, msg):
        with self.lock:
            if not self.capture_pending:
                return

        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step < width:
            self.get_logger().warn(
                f"视频帧参数异常: 宽={width}, 高={height}, 步长={step}"
            )
            return

        encoding = bytes(msg.encoding).split(b"\x00", 1)[0].decode(
            "utf-8", errors="ignore"
        )
        if encoding != "nv12":
            self.get_logger().warn(f"视频编码格式不符: {encoding}")
            return

        expected_size = step * height * 3 // 2
        if len(msg.data) < expected_size:
            self.get_logger().warn(
                f"帧数据长度不足: 实际={len(msg.data)}, 需要至少={expected_size}"
            )
            return

        try:
            frame_data = memoryview(msg.data)[:expected_size].tobytes()
        except TypeError:
            frame_data = bytes(msg.data[:expected_size])

        with self.lock:
            if not self.capture_pending:
                return
            self.capture_pending = False
            self.capture_started_at = 0.0
            scene_roi = self.latest_scene_roi
            raw_video_sub = self.raw_video_sub
            self.raw_video_sub = None

        if raw_video_sub is not None:
            self.destroy_subscription(raw_video_sub)

        self.process_captured_frame(
            (frame_data, width, height, step), scene_roi
        )

    def vision_guard_callback(self, msg):
        best_roi = None
        max_area_scene = 0
        for target in msg.targets:
            if target.type != "tuWen":
                continue
            for roi in target.rois:
                area_scene = roi.rect.width * roi.rect.height
                if area_scene > max_area_scene:
                    max_area_scene = area_scene
                    best_roi = (
                        int(roi.rect.x_offset),
                        int(roi.rect.y_offset),
                        int(roi.rect.width),
                        int(roi.rect.height),
                    )
        if best_roi is not None:
            with self.lock:
                self.latest_scene_roi = best_roi

    def snap_request_callback(self, msg):
        if msg.data != 1:
            return
        if self.is_calling_vlm:
            self.get_logger().info("场景识别处理中，跳过本次截图请求")
            return

        with self.lock:
            if self.capture_pending:
                self.get_logger().info("已有待处理帧，跳过重复截图请求")
                return
            self.capture_pending = True
            self.capture_started_at = time.monotonic()
            try:
                self.raw_video_sub = self.create_subscription(
                    HbmMsg1080P,
                    "/nv12_img",
                    self.raw_video_callback,
                    self.qos_image,
                )
            except Exception as exc:
                self.capture_pending = False
                self.capture_started_at = 0.0
                self.raw_video_sub = None
                self.get_logger().error(f"视频订阅失败: {exc}")
                return

        self.get_logger().info("收到场景截图触发，准备抓取下一帧")

    def capture_timeout_callback(self):
        with self.lock:
            if not self.capture_pending:
                return
            if time.monotonic() - self.capture_started_at < self.capture_frame_timeout:
                return
            self.capture_pending = False
            self.capture_started_at = 0.0
            raw_video_sub = self.raw_video_sub
            self.raw_video_sub = None

        if raw_video_sub is not None:
            self.destroy_subscription(raw_video_sub)
        self.get_logger().warn("等待视频帧超时，放弃本次截图")

    def process_captured_frame(self, raw_video_frame, scene_roi):
        image_msg = self.encode_scene_picture(raw_video_frame, scene_roi)
        if image_msg is None:
            return

        image_path = self.save_picture(image_msg.data)
        self.picture_pub.publish(image_msg)

        path_msg = String()
        path_msg.data = image_path
        self.picture_path_pub.publish(path_msg)

        base64_image = base64.b64encode(image_msg.data).decode("utf-8")
        self.is_calling_vlm = True
        threading.Thread(
            target=self.call_vlm,
            args=(base64_image, image_path),
            daemon=True,
        ).start()

    def encode_scene_picture(self, raw_video_frame, scene_roi):
        data, image_width, image_height, step = raw_video_frame
        try:
            nv12 = np.frombuffer(data, dtype=np.uint8)
            nv12 = nv12.reshape((image_height * 3 // 2, step))
            if step != image_width:
                nv12 = np.ascontiguousarray(nv12[:, :image_width])
            image = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f"帧数据解析失败: {exc}")
            return None

        cropped = image
        crop_description = "全幅图像"
        if scene_roi is not None:
            x, y, width, height = scene_roi
            margin_x = int(round(width * self.margin_scene))
            margin_y = int(round(height * self.margin_scene))
            x1 = max(0, x - margin_x)
            y1 = max(0, y - margin_y)
            x2 = min(image_width, x + width + margin_x)
            y2 = min(image_height, y + height + margin_y)
            if x2 > x1 and y2 > y1:
                cropped = image[y1:y2, x1:x2]
                crop_description = f"({x1}, {y1})-({x2}, {y2})"
            else:
                self.get_logger().warn("场景裁剪区域越界，回退到全幅")
        else:
            self.get_logger().warn("未检测到场景区域，采用全幅")

        success, encoded = cv2.imencode(
            ".jpg",
            cropped,
            [cv2.IMWRITE_JPEG_QUALITY, self.scene_jpeg_quality],
        )
        if not success:
            self.get_logger().error("JPEG编码失败")
            return None

        cropped_msg = CompressedImage()
        cropped_msg.format = "jpeg"
        cropped_msg.data = encoded.tobytes()
        self.get_logger().info(
            f"场景画面编码完成: 裁剪范围={crop_description}, "
            f"画质={self.scene_jpeg_quality}, 体积={len(cropped_msg.data)}字节"
        )
        return cropped_msg

    def save_picture(self, image_bytes):
        if self.save_scene_picture:
            name = f"scene_{int(time.time())}_{self.picture_count}.jpg"
        else:
            name = "latest_scene.jpg"
        self.picture_count += 1
        image_path = os.path.join(self.scene_picture_dir, name)
        with open(image_path, "wb") as f:
            f.write(image_bytes)
        self.get_logger().info(f"场景画面已落盘: {image_path}")
        return image_path

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.display_pub.publish(msg)

    @staticmethod
    def response_text(response):
        def value(item, name, default=None):
            if isinstance(item, dict):
                return item.get(name, default)
            return getattr(item, name, default)

        direct_text = value(response, "output_text")
        if direct_text:
            return direct_text.strip()

        texts = []
        for item in value(response, "output", []) or []:
            if value(item, "type") != "message":
                continue
            for content in value(item, "content", []) or []:
                if value(content, "type") == "output_text":
                    text = value(content, "text", "")
                    if text:
                        texts.append(text)
        return "\n".join(texts).strip()

    @staticmethod
    def response_debug(response):
        if hasattr(response, "model_dump_json"):
            return response.model_dump_json()[:2000]
        if hasattr(response, "json"):
            try:
                return response.json()[:2000]
            except TypeError:
                pass
        return str(response)[:2000]

    def call_vlm(self, base64_image, image_path):
        try:
            self.publish_text("start")
            if not self.api_key:
                raise RuntimeError("ARK_API_KEY/VOLC_ACCESS_KEY is not set")

            response = self.client.responses.create(
                model=self.volc_model,
                thinking={"type": "disabled"},
                input=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{base64_image}",
                            },
                            {"type": "input_text", "text": self.scene_prompt},
                        ],
                    }
                ],
            )
            result_text = self.response_text(response)
            if not result_text:
                raise RuntimeError(
                    "模型响应未包含文本输出: "
                    f"{self.response_debug(response)}"
                )
            self.publish_text(result_text)
            self.get_logger().info(f"场景识别结果: {result_text}")
        except Exception as e:
            self.get_logger().error(f"场景识别调用异常: {e}")
            self.publish_text("error")
        finally:
            if not self.save_scene_picture:
                try:
                    os.remove(image_path)
                except OSError:
                    pass
            self.is_calling_vlm = False


def main(args=None):
    rclpy.init(args=args)
    node = SceneInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
