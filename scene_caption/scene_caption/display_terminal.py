# >> Team 3 显示终端 — 队列驱动的信息面板渲染
import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Int32, String

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError as exc:
    tk = None
    tkfont = None
    TK_IMPORT_ERROR = exc
else:
    TK_IMPORT_ERROR = None


class DisplayTerminal(Node):
    def __init__(self, text_queue):
        super().__init__("display_terminal")
        self.text_queue = text_queue

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_boot = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.display_text_sub = self.create_subscription(
            String, "/display_text", self.text_callback, qos_reliable)
        self.barcode_raw_sub = self.create_subscription(
            String, "/barcode_raw", self.barcode_callback, qos_reliable)
        self.boot_ok_sub = self.create_subscription(
            Int32,
            "/system_boot_ok",
            self.boot_ok_callback,
            qos_boot,
        )
        self.get_logger().info(
            "场景识别终端已订阅码标结果和显示文本话题")

    def text_callback(self, msg):
        text = msg.data.strip()
        if text:
            self.text_queue.put(("text", text))

    def barcode_callback(self, msg):
        text = msg.data.strip()
        if text:
            self.text_queue.put(("barcode", text))

    def boot_ok_callback(self, _msg):
        self.text_queue.put(("boot_ok", None))


class DisplayWindow:
    BARCODE_WAITING_TEXT = "码标：待解码"
    SCENE_WAITING_TEXT = "场景：等待识别"

    def __init__(self, text_queue):
        if tk is None:
            raise RuntimeError(f"无法加载tkinter: {TK_IMPORT_ERROR}")

        self.text_queue = text_queue
        self.barcode_text = self.BARCODE_WAITING_TEXT
        self.scene_text = self.SCENE_WAITING_TEXT
        self.barcode_locked = False
        self.scene_locked = False
        self.resize_after_id = None

        self.root = tk.Tk()
        self.root.title("场景识别终端")
        self.root.configure(bg="#0d1117")
        self.root.attributes("-fullscreen", True)
        self.root.bind(
            "<Escape>",
            lambda _event: self.root.attributes("-fullscreen", False),
        )
        self.root.bind("q", lambda _event: self.root.destroy())
        self.root.bind("<Configure>", self.schedule_fit)

        self.barcode_font = self.pick_font(28, "bold")
        self.scene_font = self.pick_font(20, "normal")

        self.barcode_label = tk.Label(
            self.root,
            text=self.barcode_text,
            fg="#58a6ff",
            bg="#0d1117",
            font=self.barcode_font,
            anchor="w",
            justify="left",
        )
        self.barcode_label.pack(
            fill="x", expand=False, padx=24, pady=(24, 12))

        divider = tk.Frame(self.root, bg="#21262d", height=2)
        divider.pack(fill="x", padx=24)

        self.scene_label = tk.Label(
            self.root,
            text=self.scene_text,
            fg="#c9d1d9",
            bg="#0d1117",
            font=self.scene_font,
            anchor="nw",
            justify="left",
        )
        self.scene_label.pack(
            fill="both", expand=True, padx=24, pady=(12, 24))

        self.root.after(100, self.fit_content)

    @staticmethod
    def pick_font(size, weight):
        families = set(tkfont.families())
        for family in (
            "Noto Sans CJK SC",
            "WenQuanYi Zen Hei",
            "Microsoft YaHei",
            "Arial",
        ):
            if family in families:
                return tkfont.Font(family=family, size=size, weight=weight)
        return tkfont.Font(size=size, weight=weight)

    def run(self):
        self.poll_queue()
        self.root.mainloop()

    def poll_queue(self):
        while True:
            try:
                event_type, payload = self.text_queue.get_nowait()
            except queue.Empty:
                break
            if event_type == "boot_ok":
                self.handle_boot_ok()
            elif event_type == "barcode":
                self.handle_barcode(payload)
            elif event_type == "text":
                self.handle_text(payload)
        self.root.after(100, self.poll_queue)

    def handle_boot_ok(self):
        self.barcode_text = self.BARCODE_WAITING_TEXT
        self.scene_text = self.SCENE_WAITING_TEXT
        self.barcode_locked = False
        self.scene_locked = False
        self.render()

    def handle_barcode(self, text):
        if self.barcode_locked:
            return

        parsed = self.parse_barcode(text)
        if parsed is None:
            return

        value, direction = parsed
        self.barcode_text = f"码标：{value}  {direction}"
        self.barcode_locked = True
        self.render()

    def handle_text(self, text):
        normalized = "\n".join(
            " ".join(line.split())
            for line in text.splitlines()
            if line.strip()
        )
        lowered = normalized.lower()

        if lowered == "start":
            if not self.scene_locked:
                self.scene_text = "场景：正在识别..."
        elif lowered == "error":
            if not self.scene_locked:
                self.scene_text = "场景：识别失败"
        elif not self.scene_locked:
            self.scene_text = f"场景：{normalized}"
            self.scene_locked = True
        else:
            return

        self.render()

    @staticmethod
    def parse_barcode(text):
        normalized = text.strip()
        if normalized == "ClockWise":
            return normalized, "顺时针"
        if normalized == "AntiClockWise":
            return normalized, "逆时针"

        try:
            number = int(normalized)
        except ValueError:
            return None
        if not 1 <= number <= 9999:
            return None

        direction = "逆时针" if number % 2 == 0 else "顺时针"
        return normalized, direction

    def render(self):
        self.root.after_idle(self.fit_content)

    def schedule_fit(self, event):
        if event.widget is not self.root:
            return
        if self.resize_after_id is not None:
            self.root.after_cancel(self.resize_after_id)
        self.resize_after_id = self.root.after(80, self.fit_content)

    def fit_content(self):
        self.resize_after_id = None
        available_width = self.root.winfo_width() - 48
        if available_width <= 1:
            self.root.after(100, self.fit_content)
            return

        barcode_display = self.fit_line(
            self.barcode_font,
            self.barcode_text,
            preferred_size=28,
            minimum_size=14,
            available_width=available_width,
        )
        self.barcode_label.configure(text=barcode_display)
        self.scene_font.configure(size=18)
        self.scene_label.configure(
            text=self.scene_text,
            wraplength=available_width,
        )

    @staticmethod
    def fit_line(
            font, text, preferred_size, minimum_size, available_width):
        for size in range(preferred_size, minimum_size - 1, -1):
            font.configure(size=size)
            if font.measure(text) <= available_width:
                return text

        suffix = "..."
        low = 0
        high = len(text)
        while low < high:
            middle = (low + high + 1) // 2
            candidate = text[:middle].rstrip() + suffix
            if font.measure(candidate) <= available_width:
                low = middle
            else:
                high = middle - 1
        return text[:low].rstrip() + suffix


def spin_ros(text_queue):
    rclpy.init()
    node = DisplayTerminal(text_queue)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    del args
    text_queue = queue.Queue()
    ros_thread = threading.Thread(
        target=spin_ros, args=(text_queue,), daemon=True)
    ros_thread.start()

    window = DisplayWindow(text_queue)
    window.run()


if __name__ == "__main__":
    main()
