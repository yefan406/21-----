#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# >> Team 3 障碍处理器 — 置信度加权评分+记忆预测恢复
"""
hazard_handler.py - 障碍处理节点
基于置信度加权评分的障碍规避系统。
与旧版 obstacle_avoider 的关键差异:
- 不锁定单一目标，而是对所有检测结果计算加权分数
- 使用基于记忆的预测恢复丢失目标，而非简单后退
- 检测类型: dock_point / code_mark / barrier
"""

import time
import math
import rclpy
from ai_msgs.msg import PerceptionTargets
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class HazardHandler(Node):
    """
    障碍处理 (Hazard Handler)

    算法概述:
    1. 接收视觉监控结果 (/vision_guard_result)
    2. 解析检测类型: dock_point, code_mark, barrier
    3. 对每个检测计算置信度加权分数
    4. 选择最高评分目标作为当前关注对象
    5. 根据目标类型和距离生成规避指令
    6. 丢失目标时使用记忆预测恢复
    """

    def __init__(self):
        super().__init__('hazard_handler')

        # ============================================================
        #  参数声明
        # ============================================================
        self.declare_parameter('dock_stop', 415)
        self.declare_parameter('barcode_stop', 170)
        self.declare_parameter('barrier_stop', 148)
        self.declare_parameter('base_speed', 0.6)
        self.declare_parameter('creep_speed', 0.6)
        self.declare_parameter('avoid_gain', 1.1)
        self.declare_parameter('center_weight', 0.35)
        self.declare_parameter('memory_decay', 0.80)
        self.declare_parameter('prediction_frames', 10)
        self.declare_parameter('min_barrier_conf', 0.55)
        self.declare_parameter('min_barcode_conf', 0.65)
        self.declare_parameter('min_dock_conf', 0.50)

        self.dock_stop = self.get_parameter('dock_stop').value
        self.barcode_stop = self.get_parameter('barcode_stop').value
        self.barrier_stop = self.get_parameter('barrier_stop').value
        self.base_speed = self.get_parameter('base_speed').value
        self.creep_speed = self.get_parameter('creep_speed').value
        self.avoid_gain = self.get_parameter('avoid_gain').value
        self.center_weight = self.get_parameter('center_weight').value
        self.memory_decay = self.get_parameter('memory_decay').value
        self.prediction_frames = self.get_parameter('prediction_frames').value
        self.min_barrier_conf = self.get_parameter('min_barrier_conf').value
        self.min_barcode_conf = self.get_parameter('min_barcode_conf').value
        self.min_dock_conf = self.get_parameter('min_dock_conf').value

        # ============================================================
        #  发布器
        # ============================================================
        self.cmd_pub = self.create_publisher(Twist, '/drive_avoid_cmd', 10)
        self.active_pub = self.create_publisher(Bool, '/avoid_active', 10)
        self.mode_pub = self.create_publisher(String, '/avoid_op_mode', 10)

        # ============================================================
        #  订阅器
        # ============================================================
        self.create_subscription(
            PerceptionTargets, '/vision_guard_result', self._on_detection, 10)

        # ============================================================
        #  内部状态
        # ============================================================
        # 记忆追踪: {target_id: {'type':, 'score':, 'cx':, 'cy':, 'area':, 'age':, 'vx':, 'vy'}}
        self.memory_bank = {}
        self.memory_id_counter = 0

        # 当前最佳目标
        self.current_target = None
        self.target_lost_count = 0

        # 机动状态
        self.maneuver_active = False
        self.avoid_mode = 'CLEAR'

        # 定时
        self._prev_time = self.get_clock().now()
        self._tick = self.create_timer(0.03, self._control_tick)

        self.get_logger().info('障碍处理器已激活 — 置信度加权评分模式')

    # ================================================================
    #  检测回调
    # ================================================================

    def _on_detection(self, msg):
        """解析 /vision_guard_result"""
        detections = self._message_detections(msg)
        if not detections:
            self._handle_no_detection()
            return

        # 计算所有检测的评分
        scored = self._score_detections(detections)

        # 更新记忆
        self._update_memory(scored)

        # 选择最佳目标
        self.current_target = self._select_best_target()
        self.target_lost_count = 0

    @staticmethod
    def _message_detections(msg):
        detections = []
        aliases = {'zt': 'barrier', 'qrcode': 'code_mark', 'p': 'dock_point'}
        for target in msg.targets:
            for roi in target.rois:
                rect = roi.rect
                x1 = float(rect.x_offset)
                y1 = float(rect.y_offset)
                x2 = x1 + float(rect.width)
                y2 = y1 + float(rect.height)
                detections.append({
                    'cls': aliases.get(target.type, target.type),
                    'conf': float(roi.confidence),
                    'cx': (x1 + x2) / 2.0,
                    'cy': (y1 + y2) / 2.0,
                    'area': max(0.0, x2 - x1) * max(0.0, y2 - y1),
                    'bottom': y2,
                })
        return detections

    def _handle_no_detection(self):
        """无检测时的处理"""
        self.target_lost_count += 1
        if self.target_lost_count > 15:
            self.current_target = None
            self.memory_bank.clear()

    def _parse_detections(self, data):
        """解析检测数据字符串
        格式: "cls,conf,x1,y1,x2,y2;cls,conf,x1,y1,x2,y2;..."
        """
        detections = []
        items = data.split(';')
        for item in items:
            parts = item.strip().split(',')
            if len(parts) < 6:
                continue
            try:
                cls = parts[0].strip()
                conf = float(parts[1].strip())
                x1 = float(parts[2].strip())
                y1 = float(parts[3].strip())
                x2 = float(parts[4].strip())
                y2 = float(parts[5].strip())
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1
                area = w * h
                bottom = y2
                detections.append({
                    'cls': cls,
                    'conf': conf,
                    'cx': cx,
                    'cy': cy,
                    'area': area,
                    'bottom': bottom,
                })
            except (ValueError, IndexError):
                continue
        return detections

    # ================================================================
    #  置信度加权评分系统
    # ================================================================

    def _score_detections(self, detections):
        """
        对每个检测计算加权分数。
        score = area * confidence * position_bonus
        position_bonus: 越居中越高
        """
        IMG_CENTER = 320.0  # 假设图像中心
        scored = []
        for det in detections:
            cls = det['cls']
            conf = det['conf']
            area = det['area']
            cx = det['cx']
            bottom = det['bottom']

            # 置信度阈值过滤
            if cls == 'dock_point' and conf < self.min_dock_conf:
                continue
            if cls == 'code_mark' and conf < self.min_barcode_conf:
                continue
            if cls == 'barrier' and conf < self.min_barrier_conf:
                continue

            # 位置加分: 越接近水平中心分值越高
            center_offset = abs(cx - IMG_CENTER) / IMG_CENTER
            position_bonus = 1.0 + self.center_weight * (1.0 - center_offset)

            # 底部位置加分: 越靠近图像底部(越近)分值越高
            bottom_bonus = 1.0 + 0.15 * (bottom / 480.0)

            # 计算总分
            score = area * conf * position_bonus * bottom_bonus

            scored.append({
                **det,
                'score': score,
                'position_bonus': position_bonus,
            })

        # 按分数降序排序
        scored.sort(key=lambda x: x['score'], reverse=True)
        return scored

    # ================================================================
    #  记忆系统
    # ================================================================

    def _update_memory(self, scored_detections):
        """更新目标记忆库"""
        now = self._now_sec()

        # 为每个检测匹配记忆目标
        matched_ids = set()
        for det in scored_detections:
            best_match = None
            best_dist = 100.0
            for tid, mem in self.memory_bank.items():
                if tid in matched_ids:
                    continue
                if mem['cls'] != det['cls']:
                    continue
                dx = det['cx'] - mem['cx']
                dy = det['cy'] - mem['cy']
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < best_dist and dist < 80.0:
                    best_dist = dist
                    best_match = tid

            if best_match is not None:
                # 更新已有目标
                mem = self.memory_bank[best_match]
                vx = det['cx'] - mem['cx']
                vy = det['cy'] - mem['cy']
                mem['cx'] = det['cx']
                mem['cy'] = det['cy']
                mem['area'] = det['area']
                mem['score'] = det['score']
                mem['vx'] = 0.7 * mem.get('vx', 0) + 0.3 * vx
                mem['vy'] = 0.7 * mem.get('vy', 0) + 0.3 * vy
                mem['age'] = 0
                matched_ids.add(best_match)
            else:
                # 新目标
                tid = f't{self.memory_id_counter}'
                self.memory_id_counter += 1
                self.memory_bank[tid] = {
                    'id': tid,
                    'cls': det['cls'],
                    'score': det['score'],
                    'cx': det['cx'],
                    'cy': det['cy'],
                    'area': det['area'],
                    'bottom': det['bottom'],
                    'vx': 0.0,
                    'vy': 0.0,
                    'age': 0,
                    'last_seen': now,
                }

        # 老化未匹配目标
        stale = []
        for tid, mem in self.memory_bank.items():
            if tid not in matched_ids:
                mem['age'] += 1
                mem['score'] *= self.memory_decay
                # 使用速度预测下一帧位置
                mem['cx'] += mem.get('vx', 0)
                mem['cy'] += mem.get('vy', 0)
                if mem['age'] > self.prediction_frames * 3:
                    stale.append(tid)

        for tid in stale:
            del self.memory_bank[tid]

    def _select_best_target(self):
        """从记忆库中选择最佳目标"""
        if not self.memory_bank:
            return None
        best = max(self.memory_bank.values(), key=lambda m: m['score'])
        return best

    # ================================================================
    #  控制循环
    # ================================================================

    def _control_tick(self):
        """生成规避指令"""
        cmd = Twist()
        active = False
        mode = 'CLEAR'

        target = self.current_target
        if target is None:
            self.cmd_pub.publish(cmd)
            self.active_pub.publish(Bool(data=False))
            self.mode_pub.publish(String(data=mode))
            return

        cls = target['cls']
        bottom = target['bottom']
        cx = target['cx']
        score = target['score']

        # 图像中心偏移 (正=右偏, 负=左偏)
        IMG_CENTER = 320.0
        lateral_error = (cx - IMG_CENTER) / IMG_CENTER  # [-1, 1]

        if cls == 'barrier':
            # 障碍物: 减速 + 转向规避
            mode = 'BARRIER'
            active = True
            if bottom > self.barrier_stop:
                # 近距离：减速规避
                cmd.linear.x = self.creep_speed
                cmd.angular.z = self.avoid_gain * lateral_error * (-1.0)
            else:
                # 远距离：轻度调整
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.5 * lateral_error * (-1.0)

        elif cls == 'code_mark':
            # 码标: 减速靠近
            mode = 'BARCODE'
            active = True
            if bottom > self.barcode_stop:
                cmd.linear.x = self.creep_speed
                cmd.angular.z = 0.3 * lateral_error
            else:
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.15 * lateral_error

        elif cls == 'dock_point':
            # 停靠点: 精细对接
            mode = 'DOCK'
            active = True
            if bottom > self.dock_stop:
                cmd.linear.x = 0.0  # 停靠
                cmd.angular.z = 0.0
                mode = 'DOCKED'
            else:
                cmd.linear.x = self.creep_speed
                cmd.angular.z = 0.15 * lateral_error

        self.cmd_pub.publish(cmd)
        self.active_pub.publish(Bool(data=active))
        self.mode_pub.publish(String(data=mode))

    # ================================================================
    #  回调
    # ================================================================

    # ================================================================
    #  工具函数
    # ================================================================

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = HazardHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('障碍处理器收到中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
