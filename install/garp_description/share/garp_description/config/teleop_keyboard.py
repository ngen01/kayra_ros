#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
WASD Teleop
-----------------
  w : ileri
  s : geri
  a : sola dön
  d : sağa dön
  boşluk : dur
  q : çıkış
"""

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

class TeleopWASD(Node):
    def __init__(self):
        super().__init__("teleop_wasd")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.declare_parameter("lin_speed", 0.5)
        self.declare_parameter("ang_speed", 0.5)
        
        self.lin_speed = self.get_parameter("lin_speed").value
        self.ang_speed = self.get_parameter("ang_speed").value
        
        self.twist = Twist()
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(HELP)

    def loop(self):
        key = get_key(0.01)
        if key:
            key = key.lower()
            if key == "w":
                self.twist.linear.x = self.lin_speed
                self.twist.angular.z = 0.0
            elif key == "s":
                self.twist.linear.x = -self.lin_speed
                self.twist.angular.z = 0.0
            elif key == "a":
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.ang_speed
            elif key == "d":
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.ang_speed
            elif key == " ":
                self.twist = Twist()
            elif key == "q":
                self.pub.publish(Twist())
                rclpy.shutdown()
        
        self.pub.publish(self.twist)

def main():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    rclpy.init()
    node = TeleopWASD()
    
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()