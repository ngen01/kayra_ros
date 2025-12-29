#!/usr/bin/env python3
"""
GARP AMR - Diferansiyel Odometri Hesaplayıcı
Gerçek robotlarda tekerlek encoder'larından odometri hesaplama örneği.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw: float) -> Quaternion:
    """Yaw açısını quaternion'a dönüştür"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DifferentialOdometry(Node):
    def __init__(self):
        super().__init__("differential_odometry")
        
        # Robot parametreleri
        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_separation", 0.35)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        
        # Durum değişkenleri
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = None
        
        # Publisher ve subscriber
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.joint_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_callback,
            10
        )
        
        self.get_logger().info(
            f"Diferansiyel Odometri başlatıldı\n"
            f"  wheel_radius: {self.wheel_radius}m\n"
            f"  wheel_separation: {self.wheel_separation}m"
        )

    def joint_callback(self, msg: JointState):
        # Joint isimlerini bul
        try:
            left_idx = msg.name.index("base_link_to_left_wheel")
            right_idx = msg.name.index("base_link_to_right_wheel")
        except ValueError:
            return
        
        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]
        current_time = self.get_clock().now()
        
        # İlk çağrı
        if self.last_left_pos is None:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_time = current_time
            return
        
        # Delta hesapla
        delta_left = left_pos - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Mesafeye çevir
        dl = delta_left * self.wheel_radius
        dr = delta_right * self.wheel_radius
        
        # Odometri güncelle
        self._update_odometry(dl, dr, dt)
        
        # Yayınla
        self._publish_odometry(current_time, dl, dr, dt)
        
        # Güncelle
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
        self.last_time = current_time

    def _update_odometry(self, dl: float, dr: float, dt: float):
        """Diferansiyel odometri hesaplama"""
        
        if abs(dl - dr) < 1e-6:
            # Düz hareket
            d = (dl + dr) / 2.0
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)
        else:
            # Eğik hareket
            alpha = (dr - dl) / self.wheel_separation
            r = (self.wheel_separation / 2.0) * (dl + dr) / (dr - dl)
            
            self.x += r * (math.cos(self.theta + alpha) - math.cos(self.theta))
            self.y += r * (math.sin(self.theta + alpha) - math.sin(self.theta))
            self.theta += alpha
        
        # Açıyı normalize et
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def _publish_odometry(self, timestamp, dl: float, dr: float, dt: float):
        """Odometry mesajı ve TF yayınla"""
        
        # Hız hesapla
        linear_vel = (dl + dr) / (2.0 * dt)
        angular_vel = (dr - dl) / (self.wheel_separation * dt)
        
        # Odometry mesajı
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Pozisyon
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)
        
        # Hız
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = euler_to_quaternion(self.theta)
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
