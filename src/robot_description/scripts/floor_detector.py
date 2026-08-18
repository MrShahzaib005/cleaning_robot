#!/usr/bin/env python3
"""
Floor Type Detector — 4 Modes:
  color   — RGB color detection (Gazebo simulation)
  ai      — Trained model on LIVE camera (real hardware)
  test_ai — Loads dataset images + runs through AI model (TESTING)
  manual  — User selects manually
test_ai mode: Based on robot X position in Gazebo:
  X < -3.12  → loads random TILES image from dataset → AI predicts
  X -3.12 to +3.12 → loads random WOOD image → AI predicts  
  X > +3.12  → loads random CARPET image → AI predicts
  This PROVES the AI model works without needing Gazebo textures!
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import time, os, glob, random


class FloorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')
        self.declare_parameter('detection_method', 'test_ai')
        self.declare_parameter('default_mode', 'auto')
        self.declare_parameter('model_path', '')
        self.declare_parameter('class_indices_path', '')
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('debug', True)

        self.method = self.get_parameter('detection_method').value
        self.mode = self.get_parameter('default_mode').value
        self.model_path = self.get_parameter('model_path').value
        self.class_path = self.get_parameter('class_indices_path').value
        self.update_rate = self.get_parameter('update_rate_hz').value
        self.debug_mode = self.get_parameter('debug').value

        # Auto-detect paths
        base = os.path.expanduser('~/cleaning_robot_ws/src/robot_description/ai_models')
        if not self.model_path:
            for c in [os.path.join(base, 'model/floor_model.h5')]:
                if os.path.exists(c):
                    self.model_path = c
                    break
        if not self.class_path:
            for c in [os.path.join(base, 'model/class_indices.json')]:
                if os.path.exists(c):
                    self.class_path = c
                    break

        self.dataset_dir = os.path.join(base, 'dataset')
        self.dataset_images = {'tiles': [], 'wood': [], 'carpet': []}
        self.load_dataset_images()

        # Load AI model
        self.ai_model = None
        self.idx_to_class = {0: 'carpet', 1: 'tiles', 2: 'wood'}
        if self.method in ['ai', 'test_ai']:
            self.load_ai_model()

        # Robot position (for test_ai zone detection)
        self.robot_x = 0.0
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Camera
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/imx500/image_raw', self.image_callback, sensor_qos)

        # Mode control
        self.create_subscription(String, '/floor_type/mode', self.mode_callback, 10)
        self.create_subscription(String, '/floor_type/manual_set', self.manual_set_callback, 10)

        # Publishers
        self.floor_pub = self.create_publisher(String, '/floor_type', 10)
        self.conf_pub = self.create_publisher(String, '/floor_type/confidence', 10)
        self.debug_pub = self.create_publisher(String, '/floor_type/debug', 10)
        self.mode_pub = self.create_publisher(String, '/floor_type/current_mode', 10)

        self.last_publish = 0.0
        self.publish_interval = 1.0 / self.update_rate
        self.current_floor = 'unknown'
        self.prev_floor = 'unknown'
        self.stable_count = 0
        self.stable_threshold = 3
        self.manual_floor = 'tiles'
        self.last_test_time = 0.0

        self.mode_timer = self.create_timer(0.5, self.publish_mode)

        # test_ai mode timer
        if self.method == 'test_ai':
            self.create_timer(1.0, self.test_ai_tick)

        self.get_logger().info('=' * 50)
        self.get_logger().info('   FLOOR DETECTOR — AUTO + MANUAL + TEST_AI')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Method     : {self.method}')
        self.get_logger().info(f'Mode       : {self.mode.upper()}')
        if self.method in ['ai', 'test_ai']:
            self.get_logger().info(f'Model      : {self.model_path}')
            self.get_logger().info(f'AI loaded  : {"YES" if self.ai_model else "NO"}')
        if self.method == 'test_ai':
            for t in ['tiles', 'wood', 'carpet']:
                self.get_logger().info(f'Dataset {t:7s}: {len(self.dataset_images[t])} images')
            self.get_logger().info(f'Zone: X<-3.12=tiles, -3.12<X<3.12=wood, X>3.12=carpet')
        self.get_logger().info(f'Camera     : /imx500/image_raw')
        self.get_logger().info('=' * 50)

    def load_dataset_images(self):
        for t in ['tiles', 'wood', 'carpet']:
            folder = os.path.join(self.dataset_dir, t)
            if os.path.isdir(folder):
                for ext in ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']:
                    self.dataset_images[t].extend(glob.glob(os.path.join(folder, ext)))
                self.dataset_images[t].sort()

    def load_ai_model(self):
        try:
            os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
            import tensorflow as tf
            tf.get_logger().setLevel('ERROR')
            if not os.path.exists(self.model_path):
                self.get_logger().error(f'Model not found: {self.model_path}')
                self.get_logger().warn('Falling back to color detection')
                self.method = 'color'
                return
            self.get_logger().info(f'Loading AI model...')
            self.ai_model = tf.keras.models.load_model(self.model_path)
            self.get_logger().info('AI model loaded!')
            if os.path.exists(self.class_path):
                import json
                with open(self.class_path, 'r') as f:
                    ci = json.load(f)
                self.idx_to_class = {v: k for k, v in ci.items()}
                self.get_logger().info(f'Classes: {self.idx_to_class}')
        except ImportError:
            self.get_logger().error('TensorFlow not installed!')
            self.method = 'color'
        except Exception as e:
            self.get_logger().error(f'Model load error: {e}')
            self.method = 'color'

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x

    def mode_callback(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode in ['auto', 'manual']:
            self.mode = new_mode
            if self.mode == 'manual':
                self.current_floor = self.manual_floor

    def manual_set_callback(self, msg):
        floor = msg.data.strip().lower()
        if floor in ['tiles', 'wood', 'carpet']:
            self.manual_floor = floor
            if self.mode == 'manual':
                self.current_floor = floor
                m = String(); m.data = floor; self.floor_pub.publish(m)
                c = String(); c.data = f'{floor}:100.0% (manual)'; self.conf_pub.publish(c)

    def publish_mode(self):
        m = String(); m.data = self.mode.upper(); self.mode_pub.publish(m)
        if self.mode == 'manual':
            fm = String(); fm.data = self.manual_floor; self.floor_pub.publish(fm)

    # ====== TEST_AI MODE ======
    def test_ai_tick(self):
        """Every 1 second: pick image from dataset based on robot position, run through AI"""
        if self.mode == 'manual' or self.ai_model is None:
            return

        now = time.time()
        if now - self.last_test_time < 1.0:
            return
        self.last_test_time = now

        # Determine floor zone based on robot X position
        if self.robot_x < -3.12:
            zone = 'tiles'
        elif self.robot_x > 3.12:
            zone = 'carpet'
        else:
            zone = 'wood'

        # Pick random image from that zone's dataset
        zone_imgs = self.dataset_images[zone]
        if not zone_imgs:
            return
        img_path = random.choice(zone_imgs)

        # Run through AI model
        try:
            import tensorflow as tf
            img = tf.keras.preprocessing.image.load_img(img_path, target_size=(224, 224))
            img_array = tf.keras.preprocessing.image.img_to_array(img) / 255.0
            img_array = np.expand_dims(img_array, axis=0)

            predictions = self.ai_model.predict(img_array, verbose=0)
            predicted_idx = np.argmax(predictions[0])
            confidence = predictions[0][predicted_idx] * 100
            floor_type = self.idx_to_class.get(predicted_idx, 'unknown')
            all_probs = {self.idx_to_class[i]: f'{predictions[0][i]*100:.1f}%'
                         for i in range(len(self.idx_to_class))}

            # Stability filter
            if floor_type == self.prev_floor:
                self.stable_count += 1
            else:
                self.stable_count = 1
                self.prev_floor = floor_type

            if self.stable_count >= 2:
                if floor_type != self.current_floor:
                    self.current_floor = floor_type
                    self.get_logger().info(
                        f'AI DETECT: {floor_type.upper()} ({confidence:.1f}%) '
                        f'[zone={zone}] [img={os.path.basename(img_path)}]')

            fm = String(); fm.data = self.current_floor; self.floor_pub.publish(fm)
            cm = String()
            cm.data = f'{self.current_floor}:{confidence:.1f}% (test_ai|zone={zone}|{os.path.basename(img_path)})'
            self.conf_pub.publish(cm)

            if self.debug_mode:
                dm = String()
                dm.data = f'TEST_AI zone={zone} img={os.path.basename(img_path)} pred={floor_type} conf={confidence:.1f}% probs={all_probs}'
                self.debug_pub.publish(dm)

        except Exception as e:
            self.get_logger().warn(f'test_ai error: {e}')

    # ====== CAMERA-BASED DETECTION ======
    def image_callback(self, msg):
        if self.mode == 'manual' or self.method == 'test_ai':
            return

        now = time.time()
        if now - self.last_publish < self.publish_interval:
            return
        self.last_publish = now

        if self.method == 'ai' and self.ai_model is not None:
            floor_type, confidence, debug_info = self.detect_by_ai(msg)
        else:
            floor_type, confidence, debug_info = self.detect_by_color(msg)

        if floor_type == self.prev_floor:
            self.stable_count += 1
        else:
            self.stable_count = 1
            self.prev_floor = floor_type

        if self.stable_count >= self.stable_threshold:
            if floor_type != self.current_floor:
                self.current_floor = floor_type
                self.get_logger().info(
                    f'Floor: {self.current_floor.upper()} ({confidence:.1f}%) [{self.method.upper()}]')

        fm = String(); fm.data = self.current_floor; self.floor_pub.publish(fm)
        cm = String(); cm.data = f'{self.current_floor}:{confidence:.1f}% ({self.method})'; self.conf_pub.publish(cm)

        if self.debug_mode:
            dm = String(); dm.data = debug_info; self.debug_pub.publish(dm)

    def detect_by_ai(self, msg):
        try:
            import tensorflow as tf
            w, h, ch = msg.width, msg.height, 3
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, ch))
            roi = img_array[int(h*0.6):, :, :]
            roi_resized = tf.image.resize(roi, [224, 224]).numpy() / 255.0
            input_data = np.expand_dims(roi_resized, axis=0)
            predictions = self.ai_model.predict(input_data, verbose=0)
            pi = np.argmax(predictions[0])
            conf = predictions[0][pi] * 100
            ft = self.idx_to_class.get(pi, 'unknown')
            ap = {self.idx_to_class[i]: f'{predictions[0][i]*100:.1f}%' for i in range(len(self.idx_to_class))}
            return ft, conf, f'AI: {ft} {conf:.1f}% | {ap}'
        except Exception as e:
            self.get_logger().warn(f'AI error: {e}')
            return self.detect_by_color(msg)

    def detect_by_color(self, msg):
        w, h, ch, step = msg.width, msg.height, 3, msg.step
        ry = int(h * 0.6); rxs = int(w * 0.2); rxe = int(w * 0.8)
        tr, tg, tb, cnt = 0.0, 0.0, 0.0, 0
        for y in range(ry, h, 2):
            for x in range(rxs, rxe, 2):
                o = y * step + x * ch
                if o + 2 < len(msg.data):
                    tr += msg.data[o]; tg += msg.data[o+1]; tb += msg.data[o+2]; cnt += 1
        if cnt == 0:
            return 'unknown', 0.0, 'no pixels'
        ar, ag, ab = tr/cnt, tg/cnt, tb/cnt
        sc = {'tiles': 0.0, 'wood': 0.0, 'carpet': 0.0}
        if ar > 100:
            rd = ar - max(ag, ab)
            if rd > 30:
                sc['carpet'] = min(100.0, rd * 1.5)
        if ar > ag > ab:
            rg = ar - ag; gb = ag - ab
            if 10 < rg < 80 and 10 < gb < 80:
                sc['wood'] = min(100.0, rg + gb)
        diff = max(abs(ar-ag), abs(ar-ab), abs(ag-ab))
        if diff < 30:
            sc['tiles'] = max(0.0, 100.0 - diff * 3.0)
        best = max(sc, key=sc.get); score = sc[best]
        if score < 15.0:
            best = 'unknown'; score = 0.0
        return best, score, f'RGB=({ar:.0f},{ag:.0f},{ab:.0f}) T={sc["tiles"]:.0f} W={sc["wood"]:.0f} C={sc["carpet"]:.0f}'


def main(args=None):
    rclpy.init(args=args)
    node = FloorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()