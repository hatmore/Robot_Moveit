#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys


class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')

        # 创建订阅者
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 超时检测定时器（如果超过这个时间没收到消息也报错）
        self.timeout_seconds = 5.0
        self.timer = self.create_timer(self.timeout_seconds, self.check_timeout)
        self.last_message_time = self.get_clock().now()
        self.received_first_message = False

        self.get_logger().info('JointState 监控已启动，监听话题: /joint_states')
        self.get_logger().info(f'超时阈值: {self.timeout_seconds} 秒')

    def joint_state_callback(self, msg):
        """JointState 消息回调函数"""
        self.last_message_time = self.get_clock().now()
        self.received_first_message = True

        # 检查消息是否为空
        is_empty = False
        error_details = []

        # 检查关节名称
        if not msg.name or len(msg.name) == 0:
            is_empty = True
            error_details.append("关节名称列表为空")

        # 检查关节位置
        if not msg.position or len(msg.position) == 0:
            is_empty = True
            error_details.append("关节位置列表为空")

        # 检查数据长度是否一致
        if msg.name and msg.position and len(msg.name) != len(msg.position):
            is_empty = True
            error_details.append(
                f"数据长度不匹配: names={len(msg.name)}, positions={len(msg.position)}"
            )

        # 如果发现空消息，报错并退出
        if is_empty:
            self.get_logger().error('=' * 60)
            self.get_logger().error('检测到空的 JointState 消息！')
            self.get_logger().error('错误详情:')
            for detail in error_details:
                self.get_logger().error(f'  - {detail}')
            self.get_logger().error(f'消息内容: {msg}')
            self.get_logger().error('=' * 60)

            # 退出程序
            sys.exit(1)
        else:
            # 正常情况下输出简要信息
            self.get_logger().info(
                f'收到正常 JointState: {len(msg.name)} 个关节',
                throttle_duration_sec=2.0  # 每2秒最多输出一次
            )

    def check_timeout(self):
        """检查是否超时未收到消息"""
        if not self.received_first_message:
            self.get_logger().warn(
                f'警告: 启动后 {self.timeout_seconds} 秒内未收到任何 JointState 消息'
            )
            return

        time_since_last = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9

        if time_since_last > self.timeout_seconds:
            self.get_logger().error('=' * 60)
            self.get_logger().error(f'错误: 已超过 {time_since_last:.2f} 秒未收到 JointState 消息！')
            self.get_logger().error('可能的原因:')
            self.get_logger().error('  1. 关节控制器未启动')
            self.get_logger().error('  2. robot_state_publisher 节点未运行')
            self.get_logger().error('  3. /joint_states 话题发布者异常')
            self.get_logger().error('=' * 60)
            sys.exit(1)


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = JointStateMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\n监控已停止')
    except SystemExit as e:
        print(f'\n程序退出，退出码: {e.code}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()