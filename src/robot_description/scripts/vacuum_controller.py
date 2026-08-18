#!/usr/bin/env python3
"""
Vacuum Speed Controller Node
Floor type ke hisaab se vacuum motor ki speed control karta hai.

Speed Mapping:
  tiles/marble → 30% PWM (low suction — smooth floor)
  wood         → 60% PWM (medium suction)
  carpet       → 100% PWM (full suction — deep clean)
  unknown      → 50% PWM (safe default)

Subscribes: /floor_type (String)
Publishes:
  /vacuum/speed (String)     — "60%"
  /vacuum/pwm (String)       — PWM value "153" (0-255)
  /vacuum/status (String)    — "WOOD_60%_PWM153"

Usage: ros2 run robot_description vacuum_controller.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VacuumController(Node):
    def __init__(self):
        super().__init__('vacuum_controller')

        # ===== PARAMETERS =====
        self.declare_parameter('tiles_speed', 30)     # percentage
        self.declare_parameter('wood_speed', 60)
        self.declare_parameter('carpet_speed', 100)
        self.declare_parameter('unknown_speed', 50)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('ramp_step', 5)        # PWM ramp per cycle (smooth transition)

        self.speed_map = {
            'tiles': self.get_parameter('tiles_speed').value,
            'wood': self.get_parameter('wood_speed').value,
            'carpet': self.get_parameter('carpet_speed').value,
            'unknown': self.get_parameter('unknown_speed').value,
        }
        self.max_pwm = self.get_parameter('max_pwm').value
        self.ramp_step = self.get_parameter('ramp_step').value

        self.current_floor = 'unknown'
        self.target_pwm = 0
        self.current_pwm = 0
        self.target_percent = 0
        self.current_percent = 0
        self.vacuum_on = True

        # Subscribers
        self.floor_sub = self.create_subscription(
            String, '/floor_type', self.floor_callback, 10)

        # Vacuum on/off control
        self.control_sub = self.create_subscription(
            String, '/vacuum/control', self.control_callback, 10)

        # Publishers
        self.speed_pub = self.create_publisher(String, '/vacuum/speed', 10)
        self.pwm_pub = self.create_publisher(String, '/vacuum/pwm', 10)
        self.status_pub = self.create_publisher(String, '/vacuum/status', 10)

        # Ramp timer — smooth PWM transition 10Hz
        self.ramp_timer = self.create_timer(0.1, self.ramp_loop)

        # Status timer — publish status 2Hz
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║     VACUUM SPEED CONTROLLER         ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'Speed mapping:')
        self.get_logger().info(f'  TILES/MARBLE → {self.speed_map["tiles"]}% (light suction)')
        self.get_logger().info(f'  WOOD         → {self.speed_map["wood"]}% (medium suction)')
        self.get_logger().info(f'  CARPET       → {self.speed_map["carpet"]}% (full suction)')
        self.get_logger().info(f'  UNKNOWN      → {self.speed_map["unknown"]}% (safe default)')
        self.get_logger().info(f'PWM range: 0-{self.max_pwm}')
        self.get_logger().info(f'Ramp step: {self.ramp_step} per cycle')
        self.get_logger().info('')
        self.get_logger().info('Listening on /floor_type...')
        self.get_logger().info('Control: ros2 topic pub --once /vacuum/control std_msgs/msg/String "data: \'off\'"')

        # Initial state
        self.update_target('unknown')

    def floor_callback(self, msg):
        """Floor type change → vacuum speed update"""
        floor_type = msg.data.strip().lower()

        if floor_type not in self.speed_map:
            floor_type = 'unknown'

        if floor_type != self.current_floor:
            self.current_floor = floor_type
            self.update_target(floor_type)

    def control_callback(self, msg):
        """Vacuum on/off control"""
        cmd = msg.data.strip().lower()
        if cmd == 'off' or cmd == 'stop':
            self.vacuum_on = False
            self.target_pwm = 0
            self.target_percent = 0
            self.get_logger().info('Vacuum OFF')
        elif cmd == 'on' or cmd == 'start':
            self.vacuum_on = True
            self.update_target(self.current_floor)
            self.get_logger().info('Vacuum ON')

    def update_target(self, floor_type):
        """Target PWM set karo floor type ke hisaab se"""
        if not self.vacuum_on:
            return

        percent = self.speed_map.get(floor_type, self.speed_map['unknown'])
        pwm = int((percent / 100.0) * self.max_pwm)

        old_percent = self.target_percent
        self.target_percent = percent
        self.target_pwm = pwm

        if old_percent != percent:
            self.get_logger().info(
                f'🔄 Floor: {floor_type.upper()} → Vacuum: {percent}% (PWM: {pwm}/{self.max_pwm})')

    def ramp_loop(self):
        """Smooth PWM ramping — sudden speed change se motor damage na ho"""
        if self.current_pwm < self.target_pwm:
            self.current_pwm = min(self.current_pwm + self.ramp_step, self.target_pwm)
        elif self.current_pwm > self.target_pwm:
            self.current_pwm = max(self.current_pwm - self.ramp_step, self.target_pwm)

        # Calculate current percent
        if self.max_pwm > 0:
            self.current_percent = int((self.current_pwm / self.max_pwm) * 100)

        # Publish PWM
        pwm_msg = String()
        pwm_msg.data = str(self.current_pwm)
        self.pwm_pub.publish(pwm_msg)

        # Publish speed percent
        speed_msg = String()
        speed_msg.data = f'{self.current_percent}%'
        self.speed_pub.publish(speed_msg)

    def publish_status(self):
        """Detailed status publish"""
        status_msg = String()
        if not self.vacuum_on:
            status_msg.data = 'VACUUM_OFF'
        else:
            status_msg.data = (
                f'{self.current_floor.upper()}_'
                f'{self.current_percent}%_'
                f'PWM{self.current_pwm}_'
                f'TARGET{self.target_percent}%')
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VacuumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Vacuum Controller...')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()