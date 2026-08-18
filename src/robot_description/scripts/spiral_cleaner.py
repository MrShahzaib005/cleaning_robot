#!/usr/bin/env python3
"""Spiral Cleaning — Nav2 handles obstacle avoidance."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math, time

class SpiralCleaner(Node):
    def __init__(self):
        super().__init__('spiral_cleaner')
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('max_radius', 2.5)
        self.declare_parameter('radius_increment', 0.4)
        self.declare_parameter('points_per_loop', 8)
        self.declare_parameter('direction', 'outward')
        self.declare_parameter('timeout_sec', 120.0)
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value
        self.mr = self.get_parameter('max_radius').value
        self.ri = self.get_parameter('radius_increment').value
        self.ppl = self.get_parameter('points_per_loop').value
        self.dir = self.get_parameter('direction').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.st = self.create_publisher(String, '/cleaning/status', 10)
        self.get_logger().info(f'Spiral | center=({self.cx},{self.cy}) max_r={self.mr}')
        self.nav.wait_for_server()
        self.pub_st('SPIRAL_STARTED'); self.run()

    def pub_st(self, s): m = String(); m.data = s; self.st.publish(m)
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
        if not gh or not gh.accepted: return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=self.timeout)
        if rf.result() is None: gh.cancel_goal_async(); time.sleep(2); return False
        return rf.result().status == GoalStatus.STATUS_SUCCEEDED

    def run(self):
        wp = []; step = (2*math.pi)/self.ppl; r = self.ri
        while r <= self.mr:
            for i in range(self.ppl):
                a = i * step
                wp.append((self.cx + r*math.cos(a), self.cy + r*math.sin(a), a + math.pi/2))
                r += self.ri / self.ppl
        if self.dir == 'inward': wp.reverse()
        t = len(wp); ok = 0; fail = 0
        for i,(x,y,yaw) in enumerate(wp):
            self.pub_st(f'SPIRAL_{i+1}_OF_{t}')
            if self.goto(x,y,yaw,i+1,t): ok += 1
            else: fail += 1; time.sleep(1)
        self.get_logger().info(f'SPIRAL DONE — ok={ok} fail={fail}')
        self.pub_st('SPIRAL_COMPLETED')

def main(args=None):
    rclpy.init(args=args); n = SpiralCleaner(); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()