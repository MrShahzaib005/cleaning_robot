#!/usr/bin/env python3
"""
Zigzag Cleaning Pattern — Nav2 handles obstacle avoidance automatically.
Speed is controlled by Nav2 params (max_vel_x in nav2_params.yaml).
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math, time

class ZigzagCleaner(Node):
    def __init__(self):
        super().__init__('zigzag_cleaner')
        self.declare_parameter('min_x', -7.0)
        self.declare_parameter('max_x', 7.0)
        self.declare_parameter('min_y', -2.5)
        self.declare_parameter('max_y', 2.5)
        self.declare_parameter('strip_width', 0.5)
        self.declare_parameter('timeout_sec', 120.0)

        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.strip = self.get_parameter('strip_width').value
        self.timeout = self.get_parameter('timeout_sec').value

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.st = self.create_publisher(String, '/cleaning/status', 10)
        self.get_logger().info(f'Zigzag | x[{self.min_x},{self.max_x}] y[{self.min_y},{self.max_y}] strip={self.strip}')
        self.nav.wait_for_server()
        self.get_logger().info('Nav2 ready — starting zigzag')
        self.pub_st('ZIGZAG_STARTED')
        self.run()

    def pub_st(self, s):
        m = String(); m.data = s; self.st.publish(m)

    def pose(self, x, y, yaw):
        p = PoseStamped(); p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x); p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw/2); p.pose.orientation.w = math.cos(yaw/2)
        return p

    def goto(self, x, y, yaw, n, tot):
        self.get_logger().info(f'[{n}/{tot}] → ({x:.1f}, {y:.1f})')
        g = NavigateToPose.Goal(); g.pose = self.pose(x, y, yaw)
        f = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().warn('  ✗ Rejected'); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=self.timeout)
        if rf.result() is None:
            self.get_logger().warn('  ✗ Timeout'); gh.cancel_goal_async(); time.sleep(2); return False
        if rf.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('  ✓ OK'); return True
        self.get_logger().warn(f'  ✗ Status={rf.result().status}'); return False

    def run(self):
        wp = []; y = self.min_y; right = True
        while y <= self.max_y:
            if right:
                wp.append((self.min_x, y, 0.0)); wp.append((self.max_x, y, 0.0))
            else:
                wp.append((self.max_x, y, math.pi)); wp.append((self.min_x, y, math.pi))
            y += self.strip; right = not right
        t = len(wp); ok = 0; fail = 0
        for i,(x,y,yaw) in enumerate(wp):
            self.pub_st(f'ZIGZAG_{i+1}_OF_{t}')
            if self.goto(x,y,yaw,i+1,t): ok += 1
            else: fail += 1; time.sleep(1)
        self.get_logger().info(f'ZIGZAG DONE — ok={ok} fail={fail}')
        self.pub_st('ZIGZAG_COMPLETED')

def main(args=None):
    rclpy.init(args=args); n = ZigzagCleaner(); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()