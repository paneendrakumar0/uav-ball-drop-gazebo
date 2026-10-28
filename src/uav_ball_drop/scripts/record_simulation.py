#!/usr/bin/env python3
"""
Dual-Camera Simulation Video Recorder for UAV Ball Drop Mechanism.
Captures both Overview and Close-Up camera streams with Picture-in-Picture (PiP),
executes drop sequence, overlays real-time telemetry HUD, and encodes MP4 video.
"""

import json
import os
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

class SimulationVideoRecorder(Node):
    def __init__(self, output_video_path, target_duration_sec=13.0):
        super().__init__('sim_video_recorder')
        self.output_video_path = output_video_path
        self.target_duration = target_duration_sec
        
        self.writer = None
        self.frame_count = 0
        self.fps = 30.0
        self.width = 1280
        self.height = 720
        
        self.start_time = None
        self.drop_count = 0
        self.status_data = {
            'balls_remaining': 6,
            'current_chamber': 1,
            'total_dropped': 0,
            'carousel_angle_deg': 0.0,
            'is_ready': True
        }
        
        self.latest_closeup_frame = None
        
        # Subscriptions
        self.create_subscription(Image, '/camera/image', self.main_image_cb, 10)
        self.create_subscription(Image, '/camera_closeup/image', self.closeup_image_cb, 10)
        self.create_subscription(String, '/ball_drop/status', self.status_cb, 10)
        
        # Service client for drops
        self.drop_client = self.create_client(Trigger, '/ball_drop/drop')
        
        # Snapshot dictionary
        self.snapshots = {}
        
        self.get_logger().info("Dual-Camera Video Recorder initialized. Waiting for camera feeds...")

    def status_cb(self, msg: String):
        try:
            self.status_data = json.loads(msg.data)
        except Exception:
            pass

    def msg_to_cv2(self, msg: Image):
        if msg.encoding in ['rgb8', 'RGB8']:
            raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(raw_data, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ['bgr8', 'BGR8']:
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
        else:
            raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(raw_data, cv2.COLOR_RGB2BGR)

    def closeup_image_cb(self, msg: Image):
        self.latest_closeup_frame = self.msg_to_cv2(msg)

    def main_image_cb(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output_video_path, fourcc, self.fps, (msg.width, msg.height))
            self.width = msg.width
            self.height = msg.height
            self.get_logger().info(f"Video stream started: {msg.width}x{msg.height} @ {self.fps} FPS")

        elapsed = time.time() - self.start_time
        frame = self.msg_to_cv2(msg)

        # Automated Drop Sequence
        # Drop 1 at t = 2.0s
        if elapsed >= 2.0 and self.drop_count == 0:
            self.trigger_drop()
            self.drop_count = 1
            self.snapshots['standby'] = frame.copy()
            if self.latest_closeup_frame is not None:
                self.snapshots['closeup_mechanism'] = self.latest_closeup_frame.copy()
            
        # Drop 2 at t = 5.2s
        if elapsed >= 5.2 and self.drop_count == 1:
            self.trigger_drop()
            self.drop_count = 2
            
        # Drop 3 at t = 8.5s
        if elapsed >= 8.5 and self.drop_count == 2:
            self.trigger_drop()
            self.drop_count = 3

        # Capture snapshots
        if 2.8 <= elapsed <= 3.2 and 'falling' not in self.snapshots:
            self.snapshots['falling'] = frame.copy()
        if 4.0 <= elapsed <= 4.4 and 'impact' not in self.snapshots:
            self.snapshots['impact'] = frame.copy()

        # Render Picture-in-Picture (PiP) Inset for Close-up View
        if self.latest_closeup_frame is not None:
            pip_w, pip_h = 360, 240
            pip_x = self.width - pip_w - 24
            pip_y = 74
            
            # Resize closeup
            resized_pip = cv2.resize(self.latest_closeup_frame, (pip_w, pip_h), interpolation=cv2.INTER_AREA)
            
            # Draw border and label
            cv2.rectangle(frame, (pip_x - 3, pip_y - 3), (pip_x + pip_w + 3, pip_y + pip_h + 3), (0, 200, 255), 2)
            frame[pip_y:pip_y + pip_h, pip_x:pip_x + pip_w] = resized_pip
            
            # PiP Label
            cv2.rectangle(frame, (pip_x, pip_y), (pip_x + 190, pip_y + 24), (20, 20, 25), -1)
            cv2.putText(frame, "CLOSE-UP: DRUM / APERTURE", (pip_x + 6, pip_y + 16),
                        cv2.FONT_HERSHEY_DUPLEX, 0.40, (0, 220, 255), 1, cv2.LINE_AA)

        # Draw Telemetry HUD
        self.draw_hud(frame, elapsed)

        # Write frame
        self.writer.write(frame)
        self.frame_count += 1

        if elapsed >= self.target_duration:
            self.get_logger().info(f"Target duration {self.target_duration}s reached. Finalizing recording...")
            self.finish()

    def draw_hud(self, frame, elapsed):
        h, w = frame.shape[:2]
        
        # 1. Top Banner
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 54), (20, 20, 25), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        
        cv2.putText(frame, "UAV BALL DROP MECHANISM | GAZEBO SIM HARMONIC", 
                    (24, 36), cv2.FONT_HERSHEY_DUPLEX, 0.82, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "AGH Embedded Vision Systems / AVADER UAV", 
                    (w - 530, 36), cv2.FONT_HERSHEY_DUPLEX, 0.65, (0, 215, 255), 1, cv2.LINE_AA)
        
        # 2. Bottom-Left Telemetry Card
        card_w = 420
        card_h = 160
        card_x = 24
        card_y = h - card_h - 24
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (card_x, card_y), (card_x + card_w, card_y + card_h), (15, 15, 20), -1)
        cv2.rectangle(overlay, (card_x, card_y), (card_x + card_w, card_y + card_h), (0, 180, 240), 2)
        cv2.addWeighted(overlay, 0.80, frame, 0.20, 0, frame)
        
        # Content in Card
        t_sec = f"{elapsed:.2f}s"
        rem = self.status_data.get('balls_remaining', 6)
        ch = self.status_data.get('current_chamber', 1)
        deg = self.status_data.get('carousel_angle_deg', 0.0)
        drp = self.status_data.get('total_dropped', 0)
        
        cv2.putText(frame, "TELEMETRY & MECHANISM STATE", (card_x + 14, card_y + 26),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Sim Time: {t_sec} | Drops Executed: {drp}/6", (card_x + 14, card_y + 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Chamber: #{ch} | Carousel: {deg:.1f} deg", (card_x + 14, card_y + 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        
        status_color = (0, 255, 120) if rem > 0 else (0, 50, 255)
        status_text = f"READY: {rem} BALLS REMAINING" if rem > 0 else "MECHANISM EMPTY"
        cv2.putText(frame, status_text, (card_x + 14, card_y + 124),
                    cv2.FONT_HERSHEY_DUPLEX, 0.60, status_color, 2, cv2.LINE_AA)

        # 3. Rig Altitude
        alt_x = w - 240
        alt_y = h - 40
        cv2.rectangle(frame, (alt_x - 10, alt_y - 25), (w - 20, alt_y + 15), (20, 20, 25), -1)
        cv2.putText(frame, "Rig Altitude: 2.50 m", (alt_x, alt_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 215, 0), 2, cv2.LINE_AA)

    def trigger_drop(self):
        if self.drop_client.wait_for_service(timeout_sec=0.5):
            req = Trigger.Request()
            self.drop_client.call_async(req)
            self.get_logger().info(f"Triggered drop #{self.drop_count + 1} at elapsed time!")
        else:
            self.get_logger().warn("Drop service not available to trigger!")

    def finish(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        self.get_logger().info(f"Video saved to: {self.output_video_path} (Frames: {self.frame_count})")
        
        base_dir = os.path.dirname(self.output_video_path)
        for key, img in self.snapshots.items():
            snap_path = os.path.join(base_dir, f"snapshot_{key}.png")
            cv2.imwrite(snap_path, img)
            self.get_logger().info(f"Saved snapshot: {snap_path}")
            
        rclpy.shutdown()

def main():
    rclpy.init()
    out_video = sys.argv[1] if len(sys.argv) > 1 else '/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws/ball_drop_simulation_recording.mp4'
    recorder = SimulationVideoRecorder(out_video, target_duration_sec=13.0)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if recorder.writer:
            recorder.writer.release()

if __name__ == '__main__':
    main()
