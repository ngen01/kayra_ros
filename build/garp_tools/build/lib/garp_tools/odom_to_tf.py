#!/usr/bin/env python3
"""
Odometry mesajlarını TF transformlarına dönüştüren düğüm.
odom -> base_footprint transform'unu yayınlar.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Parametreler
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom_frame')
        self.declare_parameter('base_frame', 'base_footprint')
        
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry subscriber
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info(
            f'odom_to_tf başlatıldı: {odom_topic} -> {self.odom_frame} -> {self.base_frame}'
        )

    def odom_callback(self, msg: Odometry):
        # Transform mesajı oluştur
        t = TransformStamped()
        
        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Pozisyon
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Oryantasyon
        t.transform.rotation = msg.pose.pose.orientation
        
        # TF yayınla
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()