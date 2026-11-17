#!/usr/bin/env python3
"""
overhead_pose_publisher.py  -  VECTOR overhead localization  ->  ROS 2  ->  Unity twin

Runs on the BASE STATION PC (Ubuntu 24.04, ROS 2 Jazzy).

What it does
------------
Wraps the colleague's 6-camera ArUco tracker
(~/bachelor_7CAMERA_SYSTEM/src/track_all_cameras_WITH_VELOCITY_LARGE_TEXT_BEST_VIEW.py)
WITHOUT changing it: it imports her Camera class and process_camera() and only adds
  1. FUSION   - one pose per marker ID: when several cameras see the same car,
                the camera that sees it closest to its image centre wins
                (her 'view_quality', which she computed but never used).
  2. SPEED    - computed AFTER fusion, so it does not reset when a car crosses
                from one camera to the next.
  3. FRAME    - converts her world frame to the twin's REAL frame
                (origin = city top-left corner, +x along 475 cm, +y along 380 cm),
                using parameters measured with the 3-spot test.
  4. ROS 2    - publishes exactly the format the twin already uses
                (same as fake_fleet_publisher):
                  /car_XX/pose            geometry_msgs/PoseStamped, METERS, frame_id 'city'
                  /car_XX/overhead/speed  std_msgs/Float32, m/s
                Terminal prints are in CENTIMETERS with t = stamp_to_t(), so lines
                match Unity's RosPoseSubscriber (lastStampT) 1:1.

A pose is published only when a NEW camera detection exists (never repeated or
invented). If a car disappears, nothing is sent and Unity stops it by itself
after staleWarningSeconds.

Run
---
  source /opt/ros/jazzy/setup.bash
  python3 overhead_pose_publisher.py
  python3 overhead_pose_publisher.py --ros-args -p swap_xy:=true -p sign_y:=-1.0 -p offset_x_cm:=12.5

The camera password is asked at start (never stored in code / GitHub).
Optional: export VECTOR_CAM_PASSWORD=... to skip the prompt.
Press Q in the camera window (or Ctrl+C) to stop.
"""
import getpass
import importlib
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

M_TO_CM = 100.0
CM_TO_M = 0.01


def stamp_to_t(stamp):
    """Short readable ID for a message: seconds mod 1000 + fraction (same as Unity)."""
    return (stamp.sec % 1000) + stamp.nanosec * 1e-9


def wall_to_stamp(t_wall):
    """Wall-clock seconds (time.time()) -> ROS builtin_interfaces/Time."""
    sec = int(t_wall)
    return TimeMsg(sec=sec, nanosec=int((t_wall - sec) * 1e9))


class OverheadPosePublisher(Node):
    def __init__(self):
        super().__init__('overhead_pose_publisher')

        # ---------------- parameters ----------------
        p = self.declare_parameter
        # Where her tracker + camX_calib.npz / camX_homography.npz live
        p('tracker_dir', os.path.expanduser('~/bachelor_7CAMERA_SYSTEM/src'))
        p('tracker_module', 'track_all_cameras_WITH_VELOCITY_LARGE_TEXT_BEST_VIEW')
        # Marker ID -> car number:  car = marker_id + id_offset  (ID 0 -> car_01)
        p('id_offset', 1)
        p('pose_topic_format', '/car_{:02d}/pose')
        p('speed_topic_format', '/car_{:02d}/overhead/speed')
        p('frame_id', 'city')
        # Timing
        p('rate_hz', 30.0)          # how often fused poses are checked/published
        p('max_age_s', 0.25)        # a detection older than this is not used
        # HER frame -> TWIN REAL frame (fill from the 3-spot test)
        #   1) swap x/y   2) multiply by signs   3) add offsets (cm)
        p('swap_xy', False)
        p('sign_x', 1.0)
        p('sign_y', 1.0)
        p('offset_x_cm', 0.0)
        p('offset_y_cm', 0.0)
        p('heading_offset_deg', 0.0)
        # Marker centre -> twin reference point on the car (cm, in the car's own axes)
        #   forward = towards the car front, left = towards the car's left side
        #   e.g. marker at car centre, twin uses rear-right corner: forward -18, left -9.5
        p('ref_forward_cm', 0.0)
        p('ref_left_cm', 0.0)
        # Speed smoothing (0 = raw, 0.9 = very smooth)
        p('speed_smoothing', 0.5)
        # Display / printing
        p('show_window', True)
        p('print_hz', 2.0)          # TX lines per second per car (0 = off)

        g = lambda n: self.get_parameter(n).value
        self.id_offset = int(g('id_offset'))
        self.pose_topic_format = g('pose_topic_format')
        self.speed_topic_format = g('speed_topic_format')
        self.frame_id = g('frame_id')
        self.rate_hz = float(g('rate_hz'))
        self.max_age_s = float(g('max_age_s'))
        self.swap_xy = bool(g('swap_xy'))
        self.sign_x = float(g('sign_x'))
        self.sign_y = float(g('sign_y'))
        self.offset_x_cm = float(g('offset_x_cm'))
        self.offset_y_cm = float(g('offset_y_cm'))
        self.heading_offset_rad = math.radians(float(g('heading_offset_deg')))
        self.ref_forward_cm = float(g('ref_forward_cm'))
        self.ref_left_cm = float(g('ref_left_cm'))
        self.speed_smoothing = min(max(float(g('speed_smoothing')), 0.0), 0.99)
        self.show_window = bool(g('show_window'))
        self.print_hz = float(g('print_hz'))

        # ---------------- load HER tracker (unchanged) ----------------
        tracker_dir = os.path.expanduser(g('tracker_dir'))
        if not os.path.isdir(tracker_dir):
            raise RuntimeError(f'tracker_dir not found: {tracker_dir}')
        # Her Camera class loads camX_*.npz relative to the current folder
        os.chdir(tracker_dir)
        sys.path.insert(0, tracker_dir)
        self.trk = importlib.import_module(g('tracker_module'))
        self.get_logger().info(
            f'Using tracker {g("tracker_module")} from {tracker_dir} '
            f'(marker IDs {sorted(self.trk.MARKER_IDS)})')

        # ---------------- open the cameras (same as her main()) ----------------
        password = os.environ.get('VECTOR_CAM_PASSWORD') or getpass.getpass('Enter camera password: ')
        self.cameras = {}
        for index, ip in self.trk.CAMERA_IPS.items():
            try:
                cam = self.trk.Camera(index, ip, self.trk.USERNAME, password)
                if cam.open_stream():
                    self.cameras[index] = cam
                    threading.Thread(target=cam.capture_loop, daemon=True).start()
                    self.get_logger().info(f'Camera {index}: stream opened ({ip})')
                else:
                    self.get_logger().error(cam.open_error)
            except Exception as error:
                self.get_logger().error(f'Camera {index}: setup failed: {error}')
        if not self.cameras:
            raise RuntimeError('No cameras opened. Check the IPs, password and RTSP path.')

        # ---------------- state ----------------
        self.last_processed = {i: 0.0 for i in self.cameras}   # last frame time processed per camera
        self.detections = {i: {} for i in self.cameras}        # cam -> {marker_id: (result, t_wall)}
        self.camera_frames = {}                                 # cam -> annotated frame (display)
        self.pose_pubs = {}                                     # car -> publisher
        self.speed_pubs = {}
        self.last_pub_t = {}                                    # marker_id -> detection time last published
        self.speed_state = {}                                   # marker_id -> (x_cm, y_cm, t, speed_cm_s)
        self.last_print = {}                                    # marker_id -> wall time of last print
        self.msg_count = 0

        self.get_logger().info(
            f'Frame: swap_xy={self.swap_xy} sign=({self.sign_x:+.0f},{self.sign_y:+.0f}) '
            f'offset=({self.offset_x_cm:.1f},{self.offset_y_cm:.1f}) cm '
            f'heading_offset={math.degrees(self.heading_offset_rad):.1f} deg | '
            f'ref point fwd={self.ref_forward_cm:.1f} left={self.ref_left_cm:.1f} cm')
        self.get_logger().info('Publishing /car_XX/pose (meters, frame "city"). Press Q or Ctrl+C to stop.')

    # ------------------------------------------------------------------
    # 1) Run her per-camera detection on every NEW frame
    # ------------------------------------------------------------------
    def process_new_frames(self):
        for idx, cam in self.cameras.items():
            with cam.lock:
                t_read = cam.last_read_time
            if t_read <= self.last_processed[idx]:
                continue                      # no new frame from this camera yet
            frame = cam.get_frame()
            if frame is None:
                continue
            self.last_processed[idx] = t_read

            display, results = self.trk.process_camera(cam, frame)
            self.camera_frames[idx] = display
            # This camera's detections are replaced by its latest frame
            self.detections[idx] = {r['id']: (r, t_read) for r in results}

    # ------------------------------------------------------------------
    # 2) Fuse: one detection per marker ID (most central camera wins)
    # ------------------------------------------------------------------
    def fuse(self, now):
        best = {}
        for dets in self.detections.values():
            for marker_id, (r, t_det) in dets.items():
                if now - t_det > self.max_age_s:
                    continue
                if marker_id not in best or r['view_quality'] < best[marker_id][0]['view_quality']:
                    best[marker_id] = (r, t_det)
        return best

    # ------------------------------------------------------------------
    # 3) HER frame -> TWIN REAL frame
    # ------------------------------------------------------------------
    def to_twin_frame(self, x, y, theta_deg):
        # Heading as a unit vector, transformed exactly like the position
        th = math.radians(theta_deg)
        hx, hy = math.cos(th), math.sin(th)
        if self.swap_xy:
            x, y = y, x
            hx, hy = hy, hx
        x *= self.sign_x
        y *= self.sign_y
        hx *= self.sign_x
        hy *= self.sign_y
        x += self.offset_x_cm
        y += self.offset_y_cm
        yaw = math.atan2(hy, hx) + self.heading_offset_rad

        # Move from the marker centre to the twin's reference point on the car
        if self.ref_forward_cm != 0.0 or self.ref_left_cm != 0.0:
            c, s = math.cos(yaw), math.sin(yaw)
            x += self.ref_forward_cm * c - self.ref_left_cm * s
            y += self.ref_forward_cm * s + self.ref_left_cm * c
        return x, y, yaw

    # ------------------------------------------------------------------
    # Speed AFTER fusion (continuous across camera borders)
    # ------------------------------------------------------------------
    def update_speed(self, marker_id, x_cm, y_cm, t_det):
        prev = self.speed_state.get(marker_id)
        speed = 0.0
        if prev is not None:
            px, py, pt, pspeed = prev
            dt = t_det - pt
            if 0.0 < dt <= 1.0:
                raw = math.hypot(x_cm - px, y_cm - py) / dt
                a = self.speed_smoothing
                speed = a * pspeed + (1.0 - a) * raw
            # dt > 1 s: tracking was lost -> restart at 0
        self.speed_state[marker_id] = (x_cm, y_cm, t_det, speed)
        return speed

    # ------------------------------------------------------------------
    # 4) Publish
    # ------------------------------------------------------------------
    def get_publishers(self, car):
        if car not in self.pose_pubs:
            pose_topic = self.pose_topic_format.format(car)
            speed_topic = self.speed_topic_format.format(car)
            self.pose_pubs[car] = self.create_publisher(PoseStamped, pose_topic, 10)
            self.speed_pubs[car] = self.create_publisher(Float32, speed_topic, 10)
            self.get_logger().info(f'New car seen -> publishing {pose_topic} and {speed_topic}')
        return self.pose_pubs[car], self.speed_pubs[car]

    def publish_fused(self, now):
        for marker_id, (r, t_det) in self.fuse(now).items():
            if t_det <= self.last_pub_t.get(marker_id, 0.0):
                continue                      # nothing new for this car -> send nothing
            self.last_pub_t[marker_id] = t_det

            x_cm, y_cm, yaw = self.to_twin_frame(r['x'], r['y'], r['theta'])
            speed_cm_s = self.update_speed(marker_id, x_cm, y_cm, t_det)

            car = marker_id + self.id_offset
            pose_pub, speed_pub = self.get_publishers(car)

            msg = PoseStamped()
            msg.header.stamp = wall_to_stamp(t_det)   # time the frame was captured
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x_cm * CM_TO_M      # meters (ROS standard)
            msg.pose.position.y = y_cm * CM_TO_M      # meters (ROS standard)
            msg.pose.position.z = 0.0
            msg.pose.orientation.z = math.sin(yaw / 2.0)   # heading as a yaw quaternion
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            pose_pub.publish(msg)

            speed_msg = Float32()
            speed_msg.data = float(speed_cm_s * CM_TO_M)   # m/s
            speed_pub.publish(speed_msg)
            self.msg_count += 1

            # Terminal print in CENTIMETERS, same t as Unity
            if self.print_hz > 0.0 and now - self.last_print.get(marker_id, 0.0) >= 1.0 / self.print_hz:
                self.last_print[marker_id] = now
                t = stamp_to_t(msg.header.stamp)
                print(f'TX car_{car:02d} (ID {marker_id}) t={t:8.3f}  '
                      f'x={x_cm:6.1f}  y={y_cm:6.1f} cm  '
                      f'th={math.degrees(yaw) % 360.0:5.1f} deg  '
                      f'v={speed_cm_s:5.1f} cm/s  [cam {r["camera"]} | raw '
                      f'X={r["x"]:6.1f} Y={r["y"]:6.1f} Th={r["theta"]:5.1f}]', flush=True)

    # ------------------------------------------------------------------
    # Display: same 2x3 grid as her tracker
    # ------------------------------------------------------------------
    def show_grid(self):
        tiles = []
        for camera_index in range(1, 7):
            if camera_index in self.camera_frames:
                tile = cv2.resize(self.camera_frames[camera_index], (480, 270))
            else:
                tile = np.zeros((270, 480, 3), dtype=np.uint8)
                cv2.putText(tile, f'Camera {camera_index}: waiting', (15, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(tile, f'Camera {camera_index}', (10, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3, cv2.LINE_AA)
            tiles.append(tile)
        grid = np.vstack((np.hstack(tiles[:3]), np.hstack(tiles[3:])))
        cv2.imshow('Overhead -> ROS2 publisher (Q to stop)', grid)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        period = 1.0 / self.rate_hz
        next_pub = time.time()
        while rclpy.ok():
            self.process_new_frames()

            now = time.time()
            if now >= next_pub:
                self.publish_fused(now)
                next_pub = now + period

            if self.show_window:
                self.show_grid()
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
            else:
                time.sleep(0.002)

    def shutdown(self):
        for cam in self.cameras.values():
            cam.stop()
        time.sleep(0.2)                       # let capture threads exit
        if self.show_window:
            cv2.destroyAllWindows()
        self.get_logger().info(f'Stopped. Published {self.msg_count} poses.')


def main():
    rclpy.init()
    node = None
    try:
        node = OverheadPosePublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(f'[overhead_pose_publisher] {error}', flush=True)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():          # avoids the 'rcl_shutdown already called' error on Ctrl+C
            rclpy.shutdown()


if __name__ == '__main__':
    main()
