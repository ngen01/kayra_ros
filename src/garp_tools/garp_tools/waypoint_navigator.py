#!/usr/bin/env python3
"""
Nav2 kullanarak sıralı waypoint'lere navigasyon yapan düğüm.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Nav2 Action Client
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        # Waypoint listesi (x, y, yaw)
        self.waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57),
        ]
        
        self.current_waypoint = 0
        
        self.get_logger().info('Waypoint Navigator başlatıldı')
        self.get_logger().info(f'Toplam {len(self.waypoints)} waypoint')
        
        # Başlat
        self.timer = self.create_timer(2.0, self.start_navigation)

    def start_navigation(self):
        self.timer.cancel()
        self.navigate_to_next()

    def navigate_to_next(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Tüm waypoint\'ler tamamlandı!')
            return
        
        wp = self.waypoints[self.current_waypoint]
        self.get_logger().info(
            f'Waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: '
            f'({wp[0]}, {wp[1]}, {wp[2]})'
        )
        
        # Pose oluştur
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(wp[0], wp[1], wp[2])
        
        # Action server'ı bekle
        self._action_client.wait_for_server()
        
        # Goal gönder
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal reddedildi!')
            return
        
        self.get_logger().info('Goal kabul edildi')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint {self.current_waypoint + 1} tamamlandı')
        
        self.current_waypoint += 1
        self.navigate_to_next()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        if remaining > 0:
            self.get_logger().info(f'Kalan mesafe: {remaining:.2f}m', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()