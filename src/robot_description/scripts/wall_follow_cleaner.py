#!/usr/bin/env python3
"""
Wall Follow Cleaning — REACTIVE
Speed: 0.40 m/s | 180° front scan | Corner handling | Emergency backup
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import struct, math, time

class WallFollowCleaner(Node):
    def __init__(self):
        super().__init__('wall_follow_cleaner')
        self.declare_parameter('desired_distance', 0.7)
        self.declare_parameter('forward_speed', 0.40)
        self.declare_parameter('search_speed', 0.40)
        self.declare_parameter('max_turn_speed', 0.8)
        self.declare_parameter('corner_turn_speed', 0.6)
        self.declare_parameter('wall_side', 'right')
        self.declare_parameter('duration', 600.0)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.8)
        self.declare_parameter('front_stop_dist', 0.60)
        self.declare_parameter('emergency_dist', 0.30)
        self.declare_parameter('too_close_dist', 0.35)
        self.declare_parameter('wall_detect_dist', 2.5)

        self.desired_dist = self.get_parameter('desired_distance').value
        self.fwd_speed = self.get_parameter('forward_speed').value
        self.search_speed = self.get_parameter('search_speed').value
        self.max_turn = self.get_parameter('max_turn_speed').value
        self.corner_turn = self.get_parameter('corner_turn_speed').value
        self.wall_side = self.get_parameter('wall_side').value
        self.duration = self.get_parameter('duration').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.front_stop = self.get_parameter('front_stop_dist').value
        self.emergency = self.get_parameter('emergency_dist').value
        self.too_close = self.get_parameter('too_close_dist').value
        self.wall_detect = self.get_parameter('wall_detect_dist').value

        self.prev_error = 0.0
        self.scan = None
        self.ultra_front = float('inf')
        self.t0 = None
        self.running = True
        self.state = 'WAIT'
        self.lc = 0
        self.avoid_t0 = 0.0
        self.corners = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(PointCloud2, '/ultrasonic/pointcloud', self.ultra_cb, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.st = self.create_publisher(String, '/cleaning/status', 10)
        self.create_subscription(String, '/cleaning/cancel', self.cancel_cb, 10)
        self.create_timer(0.1, self.loop)

        self.get_logger().info(f'Wall Follow | side={self.wall_side} speed={self.fwd_speed} dist={self.desired_dist}')
        self.pub_st('WALL_FOLLOW_STARTED')

    def pub_st(self, s):
        m = String(); m.data = s; self.st.publish(m)
    def cancel_cb(self, msg):
        self.running = False; self.stop(); self.pub_st('WALL_FOLLOW_CANCELLED')
    def scan_cb(self, msg):
        self.scan = msg
        if self.t0 is None:
            self.t0 = time.time(); self.state = 'SEARCH'
            self.get_logger().info('>>> SEARCH')
    def ultra_cb(self, msg):
        mf = float('inf')
        for i in range(msg.width):
            o = i * msg.point_step
            if o + 12 > len(msg.data): break
            x, y, z = struct.unpack_from('fff', msg.data, o)
            if x > 0.0:
                d = math.sqrt(x*x + y*y)
                if d < mf: mf = d
        self.ultra_front = mf
    def stop(self):
        self.cmd.publish(Twist())

    def rang(self, deg):
        if not self.scan: return float('inf')
        rad = math.radians(deg)
        i = int((rad - self.scan.angle_min) / self.scan.angle_increment)
        i = max(0, min(i, len(self.scan.ranges)-1))
        w = 5; s = max(0, i-w); e = min(len(self.scan.ranges), i+w+1)
        v = [r for r in self.scan.ranges[s:e] if self.scan.range_min < r < self.scan.range_max]
        return min(v) if v else float('inf')

    def arc_min(self, a, b, step=5):
        m = float('inf')
        d = a
        while d <= b:
            r = self.rang(d)
            if r < m: m = r
            d += step
        return m

    def chg(self, ns, r=''):
        if self.state != ns:
            o = self.state; self.state = ns
            el = int(time.time()-self.t0) if self.t0 else 0
            x = f' — {r}' if r else ''
            self.get_logger().info(f'[{el}s] {o} → {ns}{x}')

    def loop(self):
        if not self.running or not self.scan: return
        if self.duration > 0 and self.t0 and time.time()-self.t0 >= self.duration:
            self.stop(); self.running = False
            self.pub_st('WALL_FOLLOW_COMPLETED'); return

        # Distances
        fn = self.arc_min(-40, 40, 4)
        fw = self.arc_min(-80, 80, 5)
        uf = self.ultra_front
        of = min(fn, uf)

        R = self.wall_side == 'right'
        sd = self.rang(-90 if R else 90)
        sf = self.rang(-50 if R else 50)

        tw = Twist()

        # EMERGENCY
        if of < self.emergency and self.state != 'AVOID':
            self.chg('AVOID', f'{of:.2f}m'); self.avoid_t0 = time.time()

        if self.state == 'AVOID':
            t = time.time() - self.avoid_t0
            if t < 1.0:
                tw.linear.x = -0.20
            elif t < 2.5:
                tw.angular.z = self.max_turn if R else -self.max_turn
            else:
                self.chg('FOLLOW' if sd < self.wall_detect else 'SEARCH')
                self.prev_error = 0.0
            self.cmd.publish(tw); return

        # SEARCH
        if self.state == 'SEARCH':
            if sd < self.wall_detect:
                self.chg('FOLLOW', f'wall={sd:.1f}m'); self.prev_error = 0.0; return
            if fn < self.front_stop:
                self.corners += 1; self.chg('CORNER', f'fn={fn:.1f}m'); return
            tw.linear.x = self.search_speed
            if fw < self.front_stop * 2.0: tw.linear.x *= 0.6
            tw.angular.z = (-0.20 if R else 0.20)
            self.lc += 1
            if self.lc % 30 == 0:
                el = int(time.time()-self.t0)
                self.get_logger().info(f'  [SEARCH {el}s] fn={fn:.1f} sd={sd:.1f}')
                self.pub_st(f'SEARCHING_{el}s')

        # CORNER
        elif self.state == 'CORNER':
            fc = fn > self.front_stop * 1.6
            ws = sd < self.wall_detect
            if fc and ws:
                self.chg('FOLLOW', f'corner done wall={sd:.1f}m'); self.prev_error = 0.0; return
            if fc and not ws:
                tw.linear.x = 0.15
                tw.angular.z = (self.corner_turn * 0.5 if R else -self.corner_turn * 0.5)
            else:
                tw.angular.z = (self.corner_turn if R else -self.corner_turn)
            self.lc += 1
            if self.lc % 15 == 0:
                self.get_logger().info(f'  [CORNER #{self.corners}] fn={fn:.1f} sd={sd:.1f}')

        # FOLLOW
        elif self.state == 'FOLLOW':
            if sd > self.wall_detect:
                self.chg('SEARCH', f'lost! {sd:.1f}m'); return
            if fn < self.front_stop:
                self.corners += 1; self.chg('CORNER', f'#{self.corners}'); return

            if sd < self.too_close:
                tw.linear.x = self.fwd_speed * 0.5
                tw.angular.z = (self.max_turn * 0.4 if R else -self.max_turn * 0.4)
            else:
                err = sd - self.desired_dist
                de = err - self.prev_error; self.prev_error = err
                cor = self.kp * err + self.kd * de
                cor = max(-self.max_turn, min(self.max_turn, cor))
                tw.linear.x = self.fwd_speed
                if fn < self.front_stop * 1.5: tw.linear.x *= 0.6
                tw.angular.z = (-cor if R else cor)

            self.lc += 1
            if self.lc % 30 == 0:
                el = int(time.time()-self.t0)
                self.get_logger().info(f'  [FOLLOW {el}s] w={sd:.2f} fn={fn:.1f} v={tw.linear.x:.2f} c={self.corners}')
                self.pub_st(f'FOLLOWING_{el}s')

        self.cmd.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    n = WallFollowCleaner()
    try: rclpy.spin(n)
    except KeyboardInterrupt: n.stop()
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()