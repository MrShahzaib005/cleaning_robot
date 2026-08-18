#!/usr/bin/env python3
"""Random Cleaning — Nav2 handles obstacle avoidance."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math, random, time

class RandomCleaner(Node):
    def __init__(self):
        super().__init__('random_cleaner')
        self.declare_parameter('min_x', -7.0)
        self.declare_parameter('max_x', 7.0)
        self.declare_parameter('min_y', -2.5)
        self.declare_parameter('max_y', 2.5)
        self.declare_parameter('num_points', 25)
        self.declare_parameter('timeout_sec', 120.0)
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.num = self.get_parameter('num_points').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.st = self.create_publisher(String, '/cleaning/status', 10)
        self.get_logger().info(f'Random | {self.num} points | timeout={self.timeout}s')
        self.nav.wait_for_server()
        self.pub_st('RANDOM_STARTED'); self.run()

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
        ok = 0; fail = 0
        for i in range(self.num):
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            yaw = random.uniform(-math.pi, math.pi)
            self.pub_st(f'RANDOM_{i+1}_OF_{self.num}')
            if self.goto(x, y, yaw, i+1, self.num): ok += 1
            else: fail += 1; time.sleep(1)
        self.get_logger().info(f'RANDOM DONE — ok={ok} fail={fail}')
        self.pub_st('RANDOM_COMPLETED')

def main(args=None):
    rclpy.init(args=args); n = RandomCleaner(); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()