import os
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.exceptions
import rclpy.publisher
import rclpy.subscription
import sensor_msgs.msg
import std_msgs.msg
import time
import sys

from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped
from tf2_ros import TransformBroadcaster


SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
left_arm_joint_group = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
]
right_arm_joint_group = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint"
]

class InitilizeNode(Node):
    def __init__(self):
        super().__init__('Initialized_Node')
        self.isaac_joint_state_sub = self.create_subscription(JointState, '/joint_commands', self.isaac_state_cb, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # self.orderd_name = left_arm_joint_group+right_arm_joint_group+["lifter_joint","right_vacuum_control_joint","head_motor","body_revolute_joint","left_Left_1_Joint","left_vacuum_control_joint"]
        self.orderd_name = left_arm_joint_group+right_arm_joint_group
        # self.joint_state.position = [0.0 for x in range(0,17)]
        self.left_arm_joint_state = JointState()
        self.right_arm_joint_state = JointState()
        self.joint_state = JointState()
        
        self.tf_broadcaster_= TransformBroadcaster(self)
        print("finish init.... ") 
        
    def isaac_state_cb(self, msg:JointState):
        if msg.name[0]=="left_shoulder_pitch_joint":
            self.left_arm_joint_state = msg
        if msg.name[0]=="right_shoulder_pitch_joint":
            self.right_arm_joint_state = msg
    
    def publish_tf(self, stamped_pose, parent_frame_id, child_frame_id):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = stamped_pose.pose.position.x
        tf_msg.transform.translation.y = stamped_pose.pose.position.y
        tf_msg.transform.translation.z = stamped_pose.pose.position.z
        tf_msg.transform.rotation.x = stamped_pose.pose.orientation.x
        tf_msg.transform.rotation.y = stamped_pose.pose.orientation.y
        tf_msg.transform.rotation.z = stamped_pose.pose.orientation.z
        tf_msg.transform.rotation.w = stamped_pose.pose.orientation.w
        self.tf_broadcaster_.sendTransform(tf_msg)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = self.orderd_name
        for joint_name in self.orderd_name:
            if joint_name in self.left_arm_joint_state.name:
                index = self.left_arm_joint_state.name.index(joint_name)
                joint_state.position.append(self.left_arm_joint_state.position[index])
            elif joint_name in self.right_arm_joint_state.name:
                index = self.right_arm_joint_state.name.index(joint_name)
                joint_state.position.append(self.right_arm_joint_state.position[index])
            else:
                # 如果该关节在当前消息中不存在，比如某些额外的关节，可能默认设置为0或其他适当的值
                joint_state.position.append(0.0)
            if joint_name == "lifter_joint":
                joint_state.position[-1] = 0.0
            if joint_name == "head_motor":
                joint_state.position[-1] = 55.0/180.0*np.pi
        self.js_pub.publish(joint_state)

        # #compatible for POC3
        # self.zed_tf = PoseStamped()
        # self.zed_tf.header.frame_id = "head_link"
        # self.zed_tf.pose.position.x = -0.009481502650729112
        # self.zed_tf.pose.position.y = 0.10207225426183397 
        # self.zed_tf.pose.position.z = 0.03186890620011429
        # self.zed_tf.pose.orientation.x = -0.5073130681692803
        # self.zed_tf.pose.orientation.y = 0.4888282617972088 
        # self.zed_tf.pose.orientation.z = 0.4980965253582471
        # self.zed_tf.pose.orientation.w = 0.5055494365134154 
        
        # #compatible for TMP 
        # self.zed_tf_world = PoseStamped()
        # self.zed_tf_world.header.frame_id = 'base_link'
        # self.zed_tf_world.pose.position.x = 0.16
        # self.zed_tf_world.pose.position.y = 0.0317
        # self.zed_tf_world.pose.position.z = 1.339
        # self.zed_tf_world.pose.orientation.x = 0.6845
        # self.zed_tf_world.pose.orientation.y = -0.66648
        # self.zed_tf_world.pose.orientation.z = 0.209
        # self.zed_tf_world.pose.orientation.w = -0.208


        # self.front_shelf_tf = PoseStamped()
        # self.front_shelf_tf.header.frame_id = "base_link"
        # self.front_shelf_tf.pose.position.x = 1.129
        # self.front_shelf_tf.pose.position.y = 0.0259
        # self.front_shelf_tf.pose.position.z = 1.9338
        # self.front_shelf_tf.pose.orientation.x = 0.50228
        # self.front_shelf_tf.pose.orientation.y = -0.49169
        # self.front_shelf_tf.pose.orientation.z = -0.48676
        # self.front_shelf_tf.pose.orientation.w = 0.51867
        # self.publish_tf(self.zed_tf, "head_link", "zed_left_camera_optical_frame")
        # self.publish_tf(self.front_shelf_tf, "base_link", "place_tag_middle")

        # self.left_zed_tf = PoseStamped()
        # self.left_zed_tf.header.frame_id = "base_link"
        # self.left_zed_tf.pose.position.x = 0.011315 
        # self.left_zed_tf.pose.position.y = 0.809584
        # self.left_zed_tf.pose.position.z = 1.669109
        # self.left_zed_tf.pose.orientation.x = 0.998356
        # self.left_zed_tf.pose.orientation.y = -0.030130 
        # self.left_zed_tf.pose.orientation.z = 0.040210
        # self.left_zed_tf.pose.orientation.w = -0.027567 
        # self.publish_tf(self.left_zed_tf, "base_link", "zed_left_camera_optical_frame_left_yellow")

        # self.right_zed_tf = PoseStamped()
        # self.right_zed_tf.header.frame_id = "base_link"
        # self.right_zed_tf.pose.position.x = 0.185
        # self.right_zed_tf.pose.position.y = -0.671
        # self.right_zed_tf.pose.position.z = 1.751
        # self.right_zed_tf.pose.orientation.x = -0.04
        # self.right_zed_tf.pose.orientation.y = 0.9988642
        # self.right_zed_tf.pose.orientation.z = 0.0246
        # self.right_zed_tf.pose.orientation.w = -0.0080236
        # self.publish_tf(self.right_zed_tf, "base_link", "zed_left_camera_optical_frame_right_yellow")


         

if __name__ == '__main__':
    rclpy.init()
    init_process= InitilizeNode()
    rclpy.spin(init_process)
    print("Has moved to init. End of process")
    
