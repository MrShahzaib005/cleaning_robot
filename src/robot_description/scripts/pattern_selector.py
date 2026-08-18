#!/usr/bin/env python3
"""
Pattern Selector — 5 Cleaning Patterns (Client Spec)

1. Wall Following (0-30%)  — 20cm wall distance, switches left/right
2. Spiral (30-60%)         — expanding/contracting, reverses on obstacle
3. Zigzag (30-60%)         — parallel back-and-forth, 90° turns
4. Random (60-85%)         — random directions, obstacle avoidance
5. Targeted (85-100%)      — flood fill, finds missed spots

MultiThreadedExecutor — no crash, auto-cancel on switch.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math, random, time, threading, struct
import numpy as np


class PatternSelector(Node):
    def __init__(self):
        super().__init__('pattern_selector')
        self.action_group = ReentrantCallbackGroup()
        self.sub_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('min_x', -7.0)
        self.declare_parameter('max_x', 7.0)
        self.declare_parameter('min_y', -2.5)
        self.declare_parameter('max_y', 2.5)
        self.declare_parameter('pattern', 'none')
        self.declare_parameter('timeout_sec', 120.0)

        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.initial = self.get_parameter('pattern').value
        self.timeout = self.get_parameter('timeout_sec').value

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                callback_group=self.action_group)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.scan = None
        self.ultra_front = float('inf')
        self.costmap = None
        self.costmap_info = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos,
                                 callback_group=self.sub_group)
        self.create_subscription(PointCloud2, '/ultrasonic/pointcloud', self.ultra_cb, 10,
                                 callback_group=self.sub_group)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 10,
                                 callback_group=self.sub_group)

        self.st = self.create_publisher(String, '/cleaning/status', 10)
        self.create_subscription(String, '/cleaning/select', self.sel_cb, 10)
        self.create_subscription(String, '/cleaning/cancel', self.cancel_cb, 10)

        self.cleaning = False
        self.cancel_flag = False
        self.cur = 'none'
        self.thread = None
        self.goal_handle = None
        self.lock = threading.Lock()
        # Track visited cells for targeted cleaning
        self.visited = set()

        self.get_logger().info('========================================')
        self.get_logger().info('   PATTERN SELECTOR v2.0 (5 Patterns)')
        self.get_logger().info('========================================')
        self.get_logger().info('1. wall_follow  — Wall Following (0-30%)')
        self.get_logger().info('2. spiral       — Spiral (30-60%)')
        self.get_logger().info('3. zigzag       — Zigzag (30-60%)')
        self.get_logger().info('4. random       — Random (60-85%)')
        self.get_logger().info('5. targeted     — Targeted Cleaning (85-100%)')
        self.get_logger().info('========================================')
        self.nav.wait_for_server()
        self.get_logger().info('Nav2 ready!')

        if self.initial != 'none':
            self.start_pattern(self.initial)

    # ========== CALLBACKS ==========
    def pub_st(self, s):
        m = String(); m.data = s; self.st.publish(m)
    def scan_cb(self, msg): self.scan = msg
    def ultra_cb(self, msg):
        mf = float('inf')
        for i in range(msg.width):
            o = i * msg.point_step
            if o + 12 > len(msg.data): break
            x, y, z = struct.unpack_from('fff', msg.data, o)
            if x > 0: mf = min(mf, math.sqrt(x*x + y*y))
        self.ultra_front = mf
    def costmap_cb(self, msg):
        self.costmap = msg
        self.costmap_info = msg.info

    def sel_cb(self, msg):
        p = msg.data.strip().lower()
        valid = ['zigzag', 'wall_follow', 'random', 'spiral', 'targeted']
        if p not in valid:
            self.get_logger().error(f'Unknown: {p} — valid: {valid}'); return
        if self.cleaning:
            self.get_logger().info(f'Switching: {self.cur} → {p}')
            self.do_cancel()
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=8.0)
            time.sleep(0.3)
        self.start_pattern(p)

    def cancel_cb(self, msg):
        if self.cleaning: self.do_cancel()

    def do_cancel(self):
        self.cancel_flag = True
        self.stop_robot()
        with self.lock:
            if self.goal_handle:
                try: self.goal_handle.cancel_goal_async()
                except: pass
                self.goal_handle = None
        self.pub_st('CANCELLING')

    def stop_robot(self): self.cmd.publish(Twist())

    # ========== PATTERN START ==========
    def start_pattern(self, p):
        with self.lock:
            self.cur = p; self.cancel_flag = False
            self.cleaning = True; self.goal_handle = None
        self.get_logger().info(f'>>> START: {p.upper()}')
        self.pub_st(f'{p.upper()}_STARTED')
        self.thread = threading.Thread(target=self.run_pattern, args=(p,), daemon=True)
        self.thread.start()

    def run_pattern(self, p):
        try:
            if p == 'zigzag': self.do_zigzag()
            elif p == 'wall_follow': self.do_wall_follow()
            elif p == 'random': self.do_random()
            elif p == 'spiral': self.do_spiral()
            elif p == 'targeted': self.do_targeted()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.stop_robot()
            with self.lock: self.cleaning = False; self.goal_handle = None
            s = 'CANCELLED' if self.cancel_flag else 'COMPLETED'
            self.pub_st(f'{p.upper()}_{s}')
            self.get_logger().info(f'<<< {p.upper()} {s}')

    # ========== NAV2 GOAL ==========
    def goto(self, x, y, yaw, n, tot):
        if self.cancel_flag: return False
        self.get_logger().info(f'[{self.cur.upper()} {n}/{tot}] → ({x:.1f},{y:.1f})')
        self.pub_st(f'{self.cur.upper()}_{n}_OF_{tot}')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        goal = NavigateToPose.Goal(); goal.pose = pose
        send_future = self.nav.send_goal_async(goal)

        deadline = time.time() + 10.0
        while not send_future.done() and time.time() < deadline:
            if self.cancel_flag: return False
            time.sleep(0.1)
        if not send_future.done() or self.cancel_flag: return False

        gh = send_future.result()
        if not gh or not gh.accepted: return False
        with self.lock: self.goal_handle = gh

        result_future = gh.get_result_async()
        deadline = time.time() + self.timeout
        while not result_future.done() and time.time() < deadline:
            if self.cancel_flag:
                try: gh.cancel_goal_async()
                except: pass
                with self.lock: self.goal_handle = None
                return False
            time.sleep(0.1)

        with self.lock: self.goal_handle = None
        if not result_future.done():
            try: gh.cancel_goal_async()
            except: pass
            return False

        # Track visited position
        self.visited.add((round(x, 1), round(y, 1)))

        result = result_future.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            return True
        return False

    def do_waypoints(self, wp):
        t = len(wp); ok = 0
        for i, (x, y, yaw) in enumerate(wp):
            if self.cancel_flag: break
            if self.goto(x, y, yaw, i+1, t): ok += 1
            elif not self.cancel_flag: time.sleep(1)
        self.get_logger().info(f'Waypoints: {ok}/{t}')

    # ====================================================================
    # 1. WALL FOLLOWING (0-30% Coverage)
    # Maintains 20cm distance from walls, switches left/right periodically
    # Uses ultrasonic sensors for wall distance
    # ====================================================================
    def rang(self, deg):
        if not self.scan: return float('inf')
        rad = math.radians(deg)
        i = int((rad - self.scan.angle_min) / self.scan.angle_increment)
        i = max(0, min(i, len(self.scan.ranges)-1))
        w=5; s=max(0,i-w); e=min(len(self.scan.ranges),i+w+1)
        v = [r for r in self.scan.ranges[s:e] if self.scan.range_min < r < self.scan.range_max]
        return min(v) if v else float('inf')

    def arc(self, a, b, step=5):
        m = float('inf'); d = a
        while d <= b: r = self.rang(d); m = min(m,r); d += step
        return m

    def do_wall_follow(self):
        """
        Wall Following: robot maintains 0.20m from walls.
        Switches between right and left wall following every 120 seconds.
        Uses ultrasonic + lidar for distance.
        """
        WALL_DIST = 0.20   # 20cm as per client spec
        SPD = 0.40; MTURN = 0.8; CTURN = 0.6
        FSTOP = 0.55; EMERG = 0.25; CLOSE = 0.18; WDET = 2.0
        KP = 2.5; KD = 1.0; DUR = 600.0
        SWITCH_INTERVAL = 120.0  # Switch left/right every 120s

        pe = 0.0; state = 'SEARCH'; corners = 0; avoid_t = 0
        t0 = time.time(); side = 'right'; last_switch = t0

        self.get_logger().info(f'Wall Follow: dist={WALL_DIST}m speed={SPD} switch every {SWITCH_INTERVAL}s')

        while self.scan is None and not self.cancel_flag: time.sleep(0.1)
        if self.cancel_flag: return

        while not self.cancel_flag:
            elapsed = time.time() - t0
            if DUR > 0 and elapsed >= DUR: break

            # Switch left/right periodically
            if time.time() - last_switch >= SWITCH_INTERVAL:
                side = 'left' if side == 'right' else 'right'
                last_switch = time.time()
                state = 'SEARCH'; pe = 0.0
                self.get_logger().info(f'Switching to {side.upper()} wall')

            fn = self.arc(-40, 40, 4); uf = self.ultra_front; of = min(fn, uf)
            R = side == 'right'
            sd = self.rang(-90 if R else 90)

            tw = Twist()

            if of < EMERG and state != 'AVOID':
                state = 'AVOID'; avoid_t = time.time()

            if state == 'AVOID':
                t = time.time() - avoid_t
                if t < 1.0: tw.linear.x = -0.20
                elif t < 2.5: tw.angular.z = (MTURN if R else -MTURN)
                else: state = 'FOLLOW' if sd < WDET else 'SEARCH'; pe = 0.0

            elif state == 'SEARCH':
                if sd < WDET: state = 'FOLLOW'; pe = 0.0
                elif fn < FSTOP: corners += 1; state = 'CORNER'
                else: tw.linear.x = SPD; tw.angular.z = (-0.25 if R else 0.25)

            elif state == 'CORNER':
                fc = fn > FSTOP * 1.6; ws = sd < WDET
                if fc and ws: state = 'FOLLOW'; pe = 0.0
                elif fc: tw.linear.x = 0.15; tw.angular.z = (CTURN*0.5 if R else -CTURN*0.5)
                else: tw.angular.z = (CTURN if R else -CTURN)

            elif state == 'FOLLOW':
                if sd > WDET: state = 'SEARCH'
                elif fn < FSTOP: corners += 1; state = 'CORNER'
                elif sd < CLOSE:
                    tw.linear.x = SPD * 0.5
                    tw.angular.z = (MTURN * 0.4 if R else -MTURN * 0.4)
                else:
                    err = sd - WALL_DIST
                    de = err - pe; pe = err
                    cor = KP * err + KD * de
                    cor = max(-MTURN, min(MTURN, cor))
                    tw.linear.x = SPD
                    if fn < FSTOP * 1.5: tw.linear.x *= 0.6
                    tw.angular.z = (-cor if R else cor)

            self.cmd.publish(tw)
            el = int(elapsed)
            if el % 5 == 0 and el > 0:
                self.pub_st(f'WALL_FOLLOW_{side.upper()}_{el}s_C{corners}')
            time.sleep(0.1)
        self.stop_robot()

    # ====================================================================
    # 2. SPIRAL PATTERN (30-60% Coverage)
    # Expanding and contracting spiral, reverses on obstacles
    # Varies inner/outer wheel speeds
    # ====================================================================
    def do_spiral(self):
        """
        Spiral: expanding outward from center.
        Reverses direction (contracts) when obstacle encountered.
        Uses varying speeds to create spiral motion.
        """
        SPD = 0.35; MAX_TURN = 1.2; MIN_TURN = 0.15
        DUR = 600.0; FSTOP = 0.50; EMERG = 0.30
        expanding = True; turn_rate = MAX_TURN; t0 = time.time()
        DECAY = 0.003  # How fast spiral expands (turn rate decreases)

        self.get_logger().info(f'Spiral: speed={SPD} duration={DUR}s')

        while self.scan is None and not self.cancel_flag: time.sleep(0.1)
        if self.cancel_flag: return

        while not self.cancel_flag:
            elapsed = time.time() - t0
            if DUR > 0 and elapsed >= DUR: break

            fn = self.arc(-40, 40, 4); uf = self.ultra_front; of = min(fn, uf)
            tw = Twist()

            if of < EMERG:
                # Reverse direction on obstacle
                tw.linear.x = -0.15
                self.cmd.publish(tw); time.sleep(0.8)
                expanding = not expanding
                turn_rate = MAX_TURN if not expanding else turn_rate
                self.get_logger().info(f'Obstacle! Now {"expanding" if expanding else "contracting"}')
                continue

            if of < FSTOP:
                # Slow down near obstacles
                tw.linear.x = SPD * 0.4
                tw.angular.z = turn_rate * 1.5  # Turn faster to avoid
            else:
                tw.linear.x = SPD
                tw.angular.z = turn_rate

            # Expand: decrease turn rate (larger circles)
            # Contract: increase turn rate (smaller circles)
            if expanding:
                turn_rate -= DECAY
                if turn_rate < MIN_TURN:
                    expanding = False
                    self.get_logger().info('Max radius — contracting')
            else:
                turn_rate += DECAY
                if turn_rate > MAX_TURN:
                    expanding = True
                    self.get_logger().info('Min radius — expanding')

            self.cmd.publish(tw)
            el = int(elapsed)
            if el % 10 == 0 and el > 0:
                self.pub_st(f'SPIRAL_{"EXPAND" if expanding else "CONTRACT"}_{el}s')
            time.sleep(0.1)
        self.stop_robot()

    # ====================================================================
    # 3. ZIGZAG PATTERN (30-60% Coverage)
    # Back-and-forth parallel lines, 90° turns
    # Adjusts row width based on room dimensions
    # ====================================================================
    def do_zigzag(self):
        """
        Zigzag: systematic back-and-forth parallel lines.
        90° turns at each end. Row width = 0.5m.
        Nav2 handles obstacle avoidance during navigation.
        """
        wp = []; y = self.min_y; right = True
        strip = 0.5  # Row width based on robot size

        while y <= self.max_y:
            if right:
                wp.append((self.min_x, y, 0.0))
                wp.append((self.max_x, y, 0.0))
            else:
                wp.append((self.max_x, y, math.pi))
                wp.append((self.min_x, y, math.pi))
            y += strip; right = not right

        self.get_logger().info(f'Zigzag: {len(wp)} waypoints, strip={strip}m')
        self.do_waypoints(wp)

    # ====================================================================
    # 4. RANDOM PATTERN (60-85% Coverage)
    # Random directions, changes direction on obstacles
    # Reaches areas missed by systematic patterns
    # ====================================================================
    def do_random(self):
        """
        Random: robot navigates to randomly selected positions.
        Changes direction when obstacles encountered (Nav2 handles avoidance).
        Covers areas that systematic patterns might miss.
        """
        num_points = 30
        wp = []
        for _ in range(num_points):
            x = random.uniform(self.min_x + 0.5, self.max_x - 0.5)
            y = random.uniform(self.min_y + 0.3, self.max_y - 0.3)
            yaw = random.uniform(-math.pi, math.pi)
            wp.append((x, y, yaw))

        self.get_logger().info(f'Random: {num_points} random waypoints')
        self.do_waypoints(wp)

    # ====================================================================
    # 5. TARGETED CLEANING (85-100% Coverage)
    # Flood fill to identify uncleaned areas
    # Seeks out largest missed spots, spiral cleans them
    # ====================================================================
    def do_targeted(self):
        """
        Targeted: finds uncleaned areas using a coverage grid.
        Generates waypoints for the largest uncovered regions.
        Performs spiral cleaning when reaching target areas.
        """
        self.get_logger().info('Targeted Cleaning: analyzing coverage...')

        # Create coverage grid
        res = 0.5  # 50cm grid cells
        x_cells = int((self.max_x - self.min_x) / res)
        y_cells = int((self.max_y - self.min_y) / res)

        # Mark previously visited cells
        grid = [[False]*y_cells for _ in range(x_cells)]
        for (vx, vy) in self.visited:
            xi = int((vx - self.min_x) / res)
            yi = int((vy - self.min_y) / res)
            if 0 <= xi < x_cells and 0 <= yi < y_cells:
                grid[xi][yi] = True
                # Also mark neighbors (robot covers ~0.5m width)
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        nx, ny = xi+dx, yi+dy
                        if 0 <= nx < x_cells and 0 <= ny < y_cells:
                            grid[nx][ny] = True

        # Find uncleaned regions using flood fill
        uncleaned = []
        visited_cells = set()

        def flood_fill(sx, sy):
            """Find connected uncleaned region"""
            region = []
            stack = [(sx, sy)]
            while stack:
                cx, cy = stack.pop()
                if (cx, cy) in visited_cells: continue
                if cx < 0 or cx >= x_cells or cy < 0 or cy >= y_cells: continue
                if grid[cx][cy]: continue
                visited_cells.add((cx, cy))
                region.append((cx, cy))
                stack.extend([(cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)])
            return region

        for xi in range(x_cells):
            for yi in range(y_cells):
                if not grid[xi][yi] and (xi, yi) not in visited_cells:
                    region = flood_fill(xi, yi)
                    if len(region) >= 2:  # Only significant regions
                        uncleaned.append(region)

        # Sort by size (largest first)
        uncleaned.sort(key=len, reverse=True)

        total_cells = x_cells * y_cells
        cleaned_cells = sum(sum(1 for c in row if c) for row in grid)
        coverage = cleaned_cells / total_cells * 100 if total_cells > 0 else 0

        self.get_logger().info(f'Coverage: {coverage:.1f}% | Uncleaned regions: {len(uncleaned)}')

        if not uncleaned:
            self.get_logger().info('No uncleaned areas found! Full coverage achieved.')
            self.pub_st('TARGETED_100%_COVERAGE')
            return

        # Generate waypoints for each uncleaned region
        wp = []
        for region in uncleaned[:15]:  # Max 15 regions
            if self.cancel_flag: break
            # Go to center of region
            cx = sum(c[0] for c in region) / len(region)
            cy = sum(c[1] for c in region) / len(region)
            wx = self.min_x + cx * res + res / 2
            wy = self.min_y + cy * res + res / 2
            # Clamp to safe area
            wx = max(self.min_x + 0.5, min(self.max_x - 0.5, wx))
            wy = max(self.min_y + 0.3, min(self.max_y - 0.3, wy))
            wp.append((wx, wy, 0.0))

            # Add small spiral around the target point
            for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
                sx = wx + 0.3 * math.cos(angle)
                sy = wy + 0.3 * math.sin(angle)
                sx = max(self.min_x + 0.5, min(self.max_x - 0.5, sx))
                sy = max(self.min_y + 0.3, min(self.max_y - 0.3, sy))
                wp.append((sx, sy, angle + math.pi/2))

        self.get_logger().info(f'Targeted: {len(wp)} waypoints for {len(uncleaned)} regions')
        self.do_waypoints(wp)

        # Report final coverage
        new_cleaned = cleaned_cells + len(self.visited)
        new_coverage = min(100.0, new_cleaned / total_cells * 100)
        self.get_logger().info(f'Estimated coverage after targeted: {new_coverage:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = PatternSelector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: node.stop_robot()
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()