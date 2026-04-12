#!/usr/bin/env python3
"""
一键检查实机关节状态与 URDF 限位的对比脚本
自动从 URDF/xacro 文件解析关节限位，订阅 /cerebellum_sdk/arm/joint_states 对比
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import os
import glob
import xml.etree.ElementTree as ET

# Servo 配置中的 joint_limit_margin
JOINT_LIMIT_MARGIN = 0.15  # rad

# URDF 描述文件搜索路径
DESCRIPTION_PATH = "/home/pji/linden_robot_moveit/src/linden_robot_description"


def parse_joint_limits_from_file(filepath):
    """从单个 URDF/xacro 文件中解析所有 revolute 关节的限位"""
    limits = {}
    try:
        tree = ET.parse(filepath)
        root = tree.getroot()
    except ET.ParseError:
        return limits

    for joint in root.iter("joint"):
        jtype = joint.get("type", "")
        if jtype != "revolute":
            continue
        name = joint.get("name", "")
        if not name:
            continue
        limit_elem = joint.find("limit")
        if limit_elem is None:
            continue
        lower = limit_elem.get("lower")
        upper = limit_elem.get("upper")
        if lower is not None and upper is not None:
            limits[name] = (float(lower), float(upper))
    return limits


def load_all_joint_limits(base_path):
    """递归搜索所有 .urdf 和 .xacro 文件，汇总关节限位"""
    all_limits = {}
    patterns = [
        os.path.join(base_path, "**", "*.urdf"),
        os.path.join(base_path, "**", "*.xacro"),
    ]
    files_searched = set()
    for pattern in patterns:
        for filepath in glob.glob(pattern, recursive=True):
            if filepath in files_searched:
                continue
            files_searched.add(filepath)
            limits = parse_joint_limits_from_file(filepath)
            # xacro 文件优先（更接近源定义）
            for name, val in limits.items():
                if name not in all_limits or filepath.endswith(".xacro"):
                    all_limits[name] = val

    return all_limits


class JointLimitChecker(Node):
    def __init__(self, joint_limits):
        super().__init__("joint_limit_checker")
        self.joint_limits = joint_limits
        self.received = False
        self.sub = self.create_subscription(
            JointState,
            "/cerebellum_sdk/arm/joint_states",
            self.callback,
            1,
        )
        self.timer = self.create_timer(5.0, self.timeout)
        self.get_logger().info(
            "从 URDF 中解析到 %d 个关节限位" % len(self.joint_limits)
        )
        self.get_logger().info("等待 /cerebellum_sdk/arm/joint_states ...")

    def timeout(self):
        if not self.received:
            self.get_logger().error("5 秒内未收到 joint_states，请检查话题是否发布")
            rclpy.shutdown()

    def callback(self, msg: JointState):
        if self.received:
            return
        self.received = True
        self.print_report(msg)
        rclpy.shutdown()

    def print_report(self, msg: JointState):
        margin = JOINT_LIMIT_MARGIN
        print("\n" + "=" * 100)
        print("  关节限位检查报告")
        print(
            "  joint_limit_margin = %.2f rad (%.1f°)"
            % (margin, math.degrees(margin))
        )
        print("=" * 100)

        fmt = "%-32s %8s %8s %8s %8s %8s  %s"
        print(
            fmt
            % ("关节名", "当前(°)", "下限(°)", "上限(°)", "距下限(°)", "距上限(°)", "状态")
        )
        print("-" * 100)

        warnings = []
        unknown = []

        for i, name in enumerate(msg.name):
            if name not in self.joint_limits:
                unknown.append(name)
                continue

            pos = msg.position[i]
            lower, upper = self.joint_limits[name]

            dist_lower = pos - lower
            dist_upper = upper - pos

            # 判断状态
            if dist_lower < 0 or dist_upper < 0:
                status = "!! 超限 !!"
            elif dist_lower < margin or dist_upper < margin:
                status = "** 触发限位保护 **"
            elif dist_lower < margin * 2 or dist_upper < margin * 2:
                status = "~  接近限位"
            else:
                status = "OK"

            print(
                fmt
                % (
                    name,
                    "%8.2f" % math.degrees(pos),
                    "%8.2f" % math.degrees(lower),
                    "%8.2f" % math.degrees(upper),
                    "%8.2f" % math.degrees(dist_lower),
                    "%8.2f" % math.degrees(dist_upper),
                    status,
                )
            )

            if "触发" in status or "超限" in status:
                warnings.append((name, pos, lower, upper, dist_lower, dist_upper))

        print("-" * 100)

        if unknown:
            print("\n[?] 以下关节未在 URDF 中找到限位定义: %s" % ", ".join(unknown))

        if warnings:
            print(
                "\n[!] 以下关节会触发 Servo 限位保护 (margin=%.2f rad):\n" % margin
            )
            for name, pos, lower, upper, dl, du in warnings:
                closer = "下限" if dl < du else "上限"
                dist = min(dl, du)
                print(
                    "    %s: 当前 %.4f rad, 距%s仅 %.4f rad (%.2f°)"
                    % (name, pos, closer, dist, math.degrees(dist))
                )
            print()
        else:
            print("\n[OK] 所有关节均在安全范围内\n")


def main():
    print("正在从 %s 解析 URDF 关节限位..." % DESCRIPTION_PATH)
    joint_limits = load_all_joint_limits(DESCRIPTION_PATH)

    if not joint_limits:
        print("[错误] 未找到任何关节限位定义，请检查路径: %s" % DESCRIPTION_PATH)
        return

    print("找到 %d 个关节限位:" % len(joint_limits))
    for name, (lower, upper) in sorted(joint_limits.items()):
        print("  %-35s [%8.4f, %8.4f] rad" % (name, lower, upper))

    rclpy.init()
    node = JointLimitChecker(joint_limits)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
