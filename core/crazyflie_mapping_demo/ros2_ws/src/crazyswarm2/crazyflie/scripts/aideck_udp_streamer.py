#!/usr/bin/env python3
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

def softmax(x, axis=None):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)

def order_points(pts):
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]

    return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)


def angle_cos(p0, p1, p2):
    d1 = p0 - p1
    d2 = p2 - p1
    denom = (np.linalg.norm(d1) * np.linalg.norm(d2)) + 1e-8
    return abs(np.dot(d1, d2) / denom)

def detect_digit_box(gray):
    h, w = gray.shape
    img_area = h * w

    # Improve local contrast
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    # norm = clahe.apply(gray)
    norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
    norm = cv2.equalizeHist(norm)

    blur = cv2.GaussianBlur(norm, (5, 5), 0)

    # Dark digit on brighter background
    digit_bin = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )[1]

    # Connect broken strokes a little
    kernel = np.ones((3, 3), np.uint8)
    digit_bin = cv2.morphologyEx(digit_bin, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(digit_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, digit_bin

    best_box = None
    best_score = -1

    img_center = np.array([w / 2.0, h / 2.0], dtype=np.float32)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 80:
            continue

        area_ratio = area / float(img_area)
        if area_ratio < 0.003 or area_ratio > 0.35:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw < 8 or bh < 12:
            continue

        aspect = bh / float(bw + 1e-8)
        if aspect < 0.8 or aspect > 6.0:
            continue

        # Avoid very edge-touching blobs
        if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
            continue

        # Extent: how much of bounding box is filled
        extent = area / float(bw * bh + 1e-8)
        if extent < 0.15 or extent > 0.95:
            continue

        cx = x + bw / 2.0
        cy = y + bh / 2.0
        dist = np.linalg.norm(np.array([cx, cy]) - img_center)

        # Prefer larger, more central blobs
        score = area - 0.35 * dist

        if score > best_score:
            best_score = score
            best_box = (x, y, bw, bh)

    return best_box, digit_bin


def digit_crop_to_mnist(digit_crop):
    if digit_crop is None or digit_crop.size == 0:
        return None

    if len(digit_crop.shape) == 3:
        digit_crop = cv2.cvtColor(digit_crop, cv2.COLOR_BGR2GRAY)

    # Threshold again inside crop for cleaner isolation
    digit_bin = cv2.threshold(
        digit_crop, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )[1]

    cnts, _ = cv2.findContours(digit_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    # Union box of all meaningful contours
    xs, ys, x2s, y2s = [], [], [], []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 10:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        xs.append(x)
        ys.append(y)
        x2s.append(x + w)
        y2s.append(y + h)

    if not xs:
        return None

    x1 = min(xs)
    y1 = min(ys)
    x2 = max(x2s)
    y2 = max(y2s)

    digit_bin = digit_bin[y1:y2, x1:x2]
    if digit_bin.size == 0:
        return None

    hh, ww = digit_bin.shape
    target_inner = 20
    scale = min(target_inner / float(ww), target_inner / float(hh))

    new_w = max(1, int(round(ww * scale)))
    new_h = max(1, int(round(hh * scale)))

    digit_resized = cv2.resize(digit_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Final MNIST style: black digit on white background
    digit_black = 255 - digit_resized
    canvas = np.ones((28, 28), dtype=np.uint8) * 255

    x_off = (28 - new_w) // 2
    y_off = (28 - new_h) // 2
    canvas[y_off:y_off + new_h, x_off:x_off + new_w] = digit_black

    return canvas

def find_paper_quad(gray):
    h, w = gray.shape
    img_area = h * w

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    norm = clahe.apply(gray)

    blur = cv2.GaussianBlur(norm, (5, 5), 0)
    edges = cv2.Canny(blur, 40, 120)

    # kernel = np.ones((3, 3), np.uint8)
    # edges = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    best_quad = None
    best_score = -1
    img_center = np.array([w / 2.0, h / 2.0], dtype=np.float32)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 800:
            continue

        # much looser than before
        if area / float(img_area) < 0.02:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            pts = approx.reshape(4, 2).astype(np.float32)
        else:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            pts = np.array(box, dtype=np.float32)

        pts = order_points(pts)

        rect = cv2.minAreaRect(cnt)
        rw, rh = rect[1]
        if rw < 15 or rh < 15:
            continue

        aspect = max(rw, rh) / (min(rw, rh) + 1e-8)
        if aspect > 3.0:
            continue

        center = pts.mean(axis=0)
        dist = np.linalg.norm(center - img_center)

        # strongly prefer larger regions, mildly prefer center
        score = area - 0.5 * dist

        if score > best_score:
            best_score = score
            best_quad = pts

    return best_quad

def warp_paper(gray, quad, out_size=200):
    dst = np.array([
        [0, 0],
        [out_size - 1, 0],
        [out_size - 1, out_size - 1],
        [0, out_size - 1]
    ], dtype=np.float32)

    M = cv2.getPerspectiveTransform(quad, dst)
    warped = cv2.warpPerspective(gray, M, (out_size, out_size))
    return warped


def extract_digit_from_warped(warped):
    # Improve local contrast inside the detected paper
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    warped = clahe.apply(warped)

    # Ignore a little border in case paper edge is dark/noisy
    margin = 4
    roi = warped[margin:-margin, margin:-margin]

    digit_bin = cv2.threshold(
        roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )[1]

    # Smaller morphology than before, so thin strokes survive better
    kernel = np.ones((2, 2), np.uint8)
    digit_bin = cv2.morphologyEx(digit_bin, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(digit_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    H, W = digit_bin.shape
    paper_area = H * W

    best = None
    best_score = -1

    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 40:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w < 4 or h < 8:
            continue

        area_ratio = area / float(paper_area)
        if area_ratio < 0.003 or area_ratio > 0.35:
            continue

        # Reject blobs touching border
        # if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:
        #     continue

        cx = x + w / 2.0
        cy = y + h / 2.0
        dist_to_center = np.sqrt((cx - W / 2.0) ** 2 + (cy - H / 2.0) ** 2)

        score = area - 0.4 * dist_to_center

        if score > best_score:
            best_score = score
            best = (x, y, w, h)

    if best is None:
        return None

    x, y, w, h = best
    digit_crop = digit_bin[y:y+h, x:x+w]

    target_inner = 20
    hh, ww = digit_crop.shape
    scale = min(target_inner / ww, target_inner / hh)

    new_w = max(1, int(ww * scale))
    new_h = max(1, int(hh * scale))

    digit_resized = cv2.resize(digit_crop, (new_w, new_h), interpolation=cv2.INTER_AREA)

    digit_black = 255 - digit_resized
    canvas = np.ones((28, 28), dtype=np.uint8) * 255

    x_off = (28 - new_w) // 2
    y_off = (28 - new_h) // 2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = digit_black

    return canvas


def extract_mnist_digit(gray):
    # First try paper-based pipeline
    # quad = find_paper_quad(gray)
    # if quad is not None:
    #     warped = warp_paper(gray, quad, out_size=200)
    #     mnist = extract_digit_from_warped(warped)
    #     if mnist is not None:
    #         return mnist, quad, warped, None, "paper"

    # Fallback: detect digit directly in full frame
    digit_box, digit_bin = detect_digit_box(gray)
    if digit_box is not None:
        x, y, w_box, h_box = digit_box

        pad = 8
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(gray.shape[1], x + w_box + pad)
        y2 = min(gray.shape[0], y + h_box + pad)

        digit_crop = gray[y1:y2, x1:x2]
        mnist = digit_crop_to_mnist(digit_crop)

        if mnist is not None:
            return mnist, None, digit_crop, (x1, y1, x2 - x1, y2 - y1), "digit"

    return None, None, None, None, "none"

class AiDeckUdpStreamer(Node):
    def __init__(self):
        super().__init__('aideck_udp_streamer')

        self.declare_parameter('deck_ip', '192.168.4.1')
        self.declare_parameter('deck_port', 5000)
        self.declare_parameter('listen_ip', '0.0.0.0')
        self.declare_parameter('listen_port', 5001)
        self.declare_parameter('image_topic', '/aideck/image_raw')
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('robot_prefix', 'crazyflie_real')
        self.declare_parameter('start_after_takeoff', True)
        self.declare_parameter('start_height_threshold', 0.24)
        self.declare_parameter('start_stable_delay', 1.0)
        self.declare_parameter('require_fresh_odom', True)
        self.declare_parameter('odom_timeout_sec', 0.3)
        self.declare_parameter('start_retry_seconds', 2.0)
        self.declare_parameter('restart_backoff_sec', 1.0)
        self.declare_parameter('prediction_conf_threshold', 0.7)
        self.declare_parameter('enable_prediction', True)
        self.declare_parameter('publish_mnist_image', True)
        self.declare_parameter('log_fps', False)

        self.deck_ip = self.get_parameter('deck_ip').value
        self.deck_port = int(self.get_parameter('deck_port').value)
        self.listen_ip = self.get_parameter('listen_ip').value
        self.listen_port = int(self.get_parameter('listen_port').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.start_after_takeoff = bool(self.get_parameter('start_after_takeoff').value)
        self.start_height_threshold = float(self.get_parameter('start_height_threshold').value)
        self.start_stable_delay = float(self.get_parameter('start_stable_delay').value)
        self.require_fresh_odom = bool(self.get_parameter('require_fresh_odom').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.start_retry_seconds = float(self.get_parameter('start_retry_seconds').value)
        self.restart_backoff_sec = float(self.get_parameter('restart_backoff_sec').value)
        self.prediction_conf_threshold = float(self.get_parameter('prediction_conf_threshold').value)
        self.enable_prediction = bool(self.get_parameter('enable_prediction').value)
        self.publish_mnist_image = bool(self.get_parameter('publish_mnist_image').value)
        self.log_fps = bool(self.get_parameter('log_fps').value)

        self.publisher_ = self.create_publisher(Image, self.image_topic, 10)
        self.mnist_publisher_ = self.create_publisher(Image, '/aideck/mnist_input', 10)
        self.prediction_publisher_ = self.create_publisher(Int32, '/aideck/digit_prediction', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, self.robot_prefix + '/odom', self.odom_callback, 10)

        self.last_published_digit = None
        self.last_publish_time = 0.0
        self.publish_cooldown_sec = 1.0

        package_dir = get_package_share_directory("crazyflie")
        model_path = os.path.join(package_dir, "models", "mnist_inverted.onnx")
        self.get_logger().info(f"Loading model: {model_path}")
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"]
        )
        self.in_name = self.session.get_inputs()[0].name
        self.out_name = self.session.get_outputs()[0].name

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(0.001)

        self.CPX_HEADER_SIZE = 4
        self.IMG_HEADER_MAGIC = 0xBC
        self.IMG_HEADER_SIZE = 11

        self.streams = {}
        self.current_z = 0.0
        self.last_odom_time = None
        self.above_height_since = None
        self.start_gate_latched = not self.start_after_takeoff
        self.stream_requested = False
        self.stream_started = False
        self.last_frame_time = None
        self.last_start_sent_time = 0.0
        self.last_restart_attempt_time = 0.0
        self.last_fps_log_time = 0.0

        self.get_logger().info(
            f'Listening UDP on {self.listen_ip}:{self.listen_port}; deck target {self.deck_ip}:{self.deck_port}'
        )

        if self.start_gate_latched:
            self.request_stream_start('startup without takeoff gate')
        else:
            self.get_logger().info(
                'Waiting for takeoff gate before starting AiDeck stream: '
                f'z >= {self.start_height_threshold:.2f}m for {self.start_stable_delay:.1f}s'
            )

        self.timer = self.create_timer(self.timer_period, self.receive_callback)
        self.control_timer = self.create_timer(0.1, self.stream_control_callback)
        self.retry_timer = self.create_timer(0.5, self.retry_stream_start)

    def odom_callback(self, msg):
        self.current_z = float(msg.pose.pose.position.z)
        self.last_odom_time = time.time()

        if self.current_z >= self.start_height_threshold:
            if self.above_height_since is None:
                self.above_height_since = self.last_odom_time
        else:
            self.above_height_since = None

    def clear_stream_state(self):
        self.streams = {}
        self.stream_started = False
        self.last_frame_time = None

    def publish_prediction(self, pred):
        now = time.time()
        if pred == self.last_published_digit and (now - self.last_publish_time) < self.publish_cooldown_sec:
            return
        msg = Int32()
        msg.data = int(pred)
        self.prediction_publisher_.publish(msg)
        self.last_published_digit = pred
        self.last_publish_time = now

    def send_start_packet(self):
        try:
            self.sock.sendto(b'FER', (self.deck_ip, self.deck_port))
            self.last_start_sent_time = time.time()
            self.get_logger().info(f'Sent start packet to {self.deck_ip}:{self.deck_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to send start packet: {e}')

    def request_stream_start(self, reason):
        now = time.time()
        if now - self.last_restart_attempt_time < self.restart_backoff_sec:
            return
        self.last_restart_attempt_time = now
        self.clear_stream_state()
        self.stream_requested = True
        self.get_logger().info(f'Requesting AiDeck stream start: {reason}')
        self.send_start_packet()

    def stream_control_callback(self):
        if self.start_gate_latched:
            return

        now = time.time()
        odom_fresh = (
            self.last_odom_time is not None and
            (now - self.last_odom_time) <= self.odom_timeout_sec
        )
        if self.require_fresh_odom and not odom_fresh:
            return
        if self.above_height_since is None:
            return
        if (now - self.above_height_since) < self.start_stable_delay:
            return

        self.start_gate_latched = True
        self.request_stream_start('takeoff gate satisfied')

    def retry_stream_start(self):
        if not self.stream_requested:
            return

        now = time.time()

        if not self.stream_started:
            if now - self.last_start_sent_time >= self.start_retry_seconds:
                self.get_logger().warn('No frames yet, retrying AiDeck stream start')
                self.request_stream_start('no frames received')
            return

        if self.last_frame_time is not None and (now - self.last_frame_time) >= self.start_retry_seconds:
            self.get_logger().warn('Stream timeout detected, restarting AiDeck stream')
            self.request_stream_start('stream timeout')

    def publish_cv_image(self, decoded: np.ndarray, publisher=None, frame_id='aideck_camera'):
        if publisher is None:
            publisher = self.publisher_

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        if decoded.ndim == 2:
            h, w = decoded.shape
            msg.height = h
            msg.width = w
            msg.encoding = 'mono8'
            msg.step = w
            msg.data = decoded.tobytes()
        else:
            h, w, c = decoded.shape
            if c == 3:
                msg.height = h
                msg.width = w
                msg.encoding = 'bgr8'
                msg.step = w * 3
                msg.data = decoded.tobytes()
            elif c == 4:
                msg.height = h
                msg.width = w
                msg.encoding = 'bgra8'
                msg.step = w * 4
                msg.data = decoded.tobytes()
            else:
                self.get_logger().warn(f'Unsupported channel count: {c}')
                return

        msg.is_bigendian = 0
        publisher.publish(msg)

    def receive_callback(self):
        if not self.stream_requested:
            return

        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                break
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')
                break

            if addr not in self.streams:
                self.streams[addr] = {
                    'buffer': bytearray(),
                    'expected_size': None,
                    'receiving': False,
                    'packet_count': 0,
                    'last_frame_time': None
                }

            stream = self.streams[addr]

            if len(data) >= self.CPX_HEADER_SIZE + 1 and data[self.CPX_HEADER_SIZE] == self.IMG_HEADER_MAGIC:
                payload = data[self.CPX_HEADER_SIZE:]
                if len(payload) < self.IMG_HEADER_SIZE:
                    self.get_logger().warn('Incomplete image header')
                    continue

                _, width, height, depth, fmt, size = struct.unpack(
                    '<BHHBBI', payload[:self.IMG_HEADER_SIZE]
                )

                stream['expected_size'] = size
                stream['buffer'] = bytearray(payload[self.IMG_HEADER_SIZE:])
                stream['receiving'] = True
                stream['packet_count'] = 1

            elif stream['receiving']:
                stream['buffer'].extend(data[self.CPX_HEADER_SIZE:])
                stream['packet_count'] += 1

                if stream['expected_size'] is not None and len(stream['buffer']) >= stream['expected_size']:
                    now = time.time()
                    first_frame = not self.stream_started
                    self.stream_started = True
                    self.last_frame_time = now

                    if first_frame:
                        self.get_logger().info('First frame received')

                    if self.log_fps and stream['last_frame_time'] is not None and (now - self.last_fps_log_time) >= 1.0:
                        delta = now - stream['last_frame_time']
                        fps = 1.0 / delta if delta > 0 else 0.0
                        self.get_logger().info(f'UDP FPS: {fps:.2f}')
                        self.last_fps_log_time = now
                    stream['last_frame_time'] = now

                    try:
                        np_data = np.frombuffer(stream['buffer'], np.uint8)
                        decoded = cv2.imdecode(np_data, cv2.IMREAD_UNCHANGED)
                        if decoded is not None:
                            if decoded.ndim == 3:
                                gray = cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY)
                                display_img = decoded.copy()
                            else:
                                gray = decoded
                                display_img = cv2.cvtColor(decoded, cv2.COLOR_GRAY2BGR)

                            mnist_img, paper_quad, debug_crop, digit_box, mode = extract_mnist_digit(gray)
                            overlay_text = 'Digit: None'

                            if paper_quad is not None:
                                pts = paper_quad.astype(np.int32).reshape((-1, 1, 2))
                                cv2.polylines(display_img, [pts], True, (0, 255, 0), 2)

                            if digit_box is not None:
                                x, y, w_box, h_box = digit_box
                                cv2.rectangle(display_img, (x, y), (x + w_box, y + h_box), (255, 0, 0), 2)

                            if mnist_img is not None:
                                if self.publish_mnist_image:
                                    self.publish_cv_image(mnist_img, self.mnist_publisher_, 'aideck_mnist')

                                if self.enable_prediction:
                                    x = mnist_img.astype(np.float32) / 255.0
                                    x = x.reshape(1, 1, 28, 28)
                                    out = self.session.run([self.out_name], {self.in_name: x})[0]
                                    probs = softmax(out, axis=1)[0]
                                    pred = int(np.argmax(probs))
                                    conf = float(probs[pred])
                                    overlay_text = f'Digit: {pred}  Conf: {conf:.2f}'
                                    if conf > self.prediction_conf_threshold:
                                        self.publish_prediction(pred)
                            elif self.publish_mnist_image:
                                blank = np.ones((28, 28), dtype=np.uint8) * 255
                                self.publish_cv_image(blank, self.mnist_publisher_, 'aideck_mnist')

                            cv2.putText(
                                display_img,
                                overlay_text,
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 0),
                                2,
                                cv2.LINE_AA
                            )
                            self.publish_cv_image(display_img, self.publisher_, 'aideck_camera')
                        else:
                            self.get_logger().warn('Failed to decode image')

                    except Exception as e:
                        self.get_logger().error(f'Decode error: {e}')

                    stream['receiving'] = False
                    stream['expected_size'] = None
                    stream['packet_count'] = 0


def main(args=None):
    rclpy.init(args=args)
    node = AiDeckUdpStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
