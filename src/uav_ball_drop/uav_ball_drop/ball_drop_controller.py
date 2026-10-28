#!/usr/bin/env python3
"""
UAV Ball Drop Controller Node (ROS 2 Jazzy).
Handles autonomous ball drop indexing, release actuation, and telemetry.
Designed for UAV precision drop missions (AGH Embedded Vision Systems Laboratory).
"""

import json
import math
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Empty, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class BallDropController(Node):
    def __init__(self):
        super().__init__('ball_drop_controller')
        
        # Declare parameters
        self.declare_parameter('total_chambers', 6)
        self.declare_parameter('world_name', 'ball_drop_world')
        self.declare_parameter('ball_model_sdf', '')
        self.declare_parameter('aperture_radial_offset', 0.048) # 48 mm from drum center
        self.declare_parameter('aperture_z_offset', -0.058)      # 58 mm below drone mounting plane
        self.declare_parameter('mechanism_pose_z', 1.50)        # Calibrated altitude: 1.50m
        
        self.total_chambers = self.get_parameter('total_chambers').value
        self.world_name = self.get_parameter('world_name').value
        self.aperture_r = self.get_parameter('aperture_radial_offset').value
        self.aperture_dz = self.get_parameter('aperture_z_offset').value
        
        # State variables
        self.balls_remaining = self.total_chambers
        self.total_dropped = 0
        self.current_chamber = 1
        self.carousel_angle = 0.0
        self.step_angle = 2.0 * math.pi / self.total_chambers # 60 deg
        self.is_busy = False
        self.drop_logs = []
        
        # Current UAV position and velocity tracking
        self.uav_pos = [0.0, 0.0, self.get_parameter('mechanism_pose_z').value]
        self.uav_vel = [0.0, 0.0, 0.0]
        
        # Resolve ball SDF model path
        default_ball_sdf = self.get_parameter('ball_model_sdf').value
        if not default_ball_sdf:
            # Look up standard package model path
            import os
            from ament_index_python.packages import get_package_share_directory
            try:
                pkg_share = get_package_share_directory('uav_ball_drop')
                self.ball_sdf_path = os.path.join(pkg_share, 'models', 'payload_ball', 'model.sdf')
            except Exception:
                self.ball_sdf_path = '/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws/src/uav_ball_drop/models/payload_ball/model.sdf'
        else:
            self.ball_sdf_path = default_ball_sdf
            
        # Publishers
        self.cmd_carousel_pub = self.create_publisher(Float64, '/ball_drop/carousel_cmd', 10)
        self.status_pub = self.create_publisher(String, '/ball_drop/status', 10)
        
        # Subscriptions
        self.create_subscription(Empty, '/ball_drop/cmd_drop', self.cmd_drop_cb, 10)
        self.create_subscription(Float64, '/ball_drop/cmd_carousel_angle', self.cmd_angle_cb, 10)
        self.create_subscription(Odometry, '/model/uav/odometry', self.uav_odom_cb, 10)
        self.create_subscription(PoseStamped, '/uav/pose', self.uav_pose_cb, 10)
        
        # Services
        self.drop_srv = self.create_service(Trigger, '/ball_drop/drop', self.handle_drop_service)
        self.reload_srv = self.create_service(Trigger, '/ball_drop/reload', self.handle_reload_service)
        
        # Periodic Telemetry Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_telemetry)
        
        self.get_logger().info(
            f"UAV Ball Drop Controller initialized. Chambers: {self.total_chambers}, Ready for drop commands."
        )

    def uav_odom_cb(self, msg: Odometry):
        self.uav_pos[0] = msg.pose.pose.position.x
        self.uav_pos[1] = msg.pose.pose.position.y
        self.uav_pos[2] = msg.pose.pose.position.z
        self.uav_vel[0] = msg.twist.twist.linear.x
        self.uav_vel[1] = msg.twist.twist.linear.y
        self.uav_vel[2] = msg.twist.twist.linear.z

    def uav_pose_cb(self, msg: PoseStamped):
        self.uav_pos[0] = msg.pose.position.x
        self.uav_pos[1] = msg.pose.position.y
        self.uav_pos[2] = msg.pose.position.z

    def cmd_drop_cb(self, msg: Empty):
        self.execute_drop()

    def cmd_angle_cb(self, msg: Float64):
        self.carousel_angle = msg.data
        cmd = Float64()
        cmd.data = self.carousel_angle
        self.cmd_carousel_pub.publish(cmd)
        self.get_logger().info(f"Carousel commanded to {math.degrees(self.carousel_angle):.1f}°")

    def handle_drop_service(self, request, response):
        if self.balls_remaining <= 0:
            response.success = False
            response.message = "Cannot drop: Mechanism is EMPTY (0 balls remaining). Please call /ball_drop/reload."
            self.get_logger().warn(response.message)
            return response
            
        success, msg = self.execute_drop()
        response.success = success
        response.message = msg
        return response

    def handle_reload_service(self, request, response):
        self.balls_remaining = self.total_chambers
        self.current_chamber = 1
        self.carousel_angle = 0.0
        
        cmd = Float64()
        cmd.data = 0.0
        self.cmd_carousel_pub.publish(cmd)
        
        response.success = True
        response.message = f"Mechanism reloaded with {self.total_chambers} balls. Carousel returned to home position."
        self.get_logger().info(response.message)
        return response

    def execute_drop(self):
        if self.balls_remaining <= 0:
            return False, "Mechanism is EMPTY."
            
        self.is_busy = True
        drop_id = self.total_dropped + 1
        chamber_num = self.current_chamber
        
        # 1. Step carousel by 60 degrees to align next chamber
        self.carousel_angle += self.step_angle
        cmd_msg = Float64()
        cmd_msg.data = self.carousel_angle
        self.cmd_carousel_pub.publish(cmd_msg)
        
        # 2. Compute drop point at exit aperture in world coordinates
        # Aperture is at offset (0, R_orbit, z_offset) relative to mechanism base
        drop_x = self.uav_pos[0]
        drop_y = self.uav_pos[1] + self.aperture_r
        drop_z = self.uav_pos[2] + self.aperture_dz
        
        # 3. Spawn payload ball entity in Gazebo Sim
        ball_name = f"payload_ball_{drop_id}"
        spawn_success = self.spawn_ball_in_gazebo(ball_name, drop_x, drop_y, drop_z)
        
        # 4. Update internal state
        self.balls_remaining -= 1
        self.total_dropped += 1
        self.current_chamber = (self.current_chamber % self.total_chambers) + 1
        self.is_busy = False
        
        status_msg = (
            f"Ball #{drop_id} dropped from chamber {chamber_num} at ({drop_x:.2f}, {drop_y:.2f}, {drop_z:.2f})m. "
            f"Remaining: {self.balls_remaining}/{self.total_chambers}."
        )
        self.get_logger().info(status_msg)
        return True, status_msg

    def spawn_ball_in_gazebo(self, ball_name, x, y, z):
        """Calls Gazebo Sim /world/<world>/create service via CLI for robustness."""
        req_text = (
            f'sdf_filename: "{self.ball_sdf_path}", '
            f'name: "{ball_name}", '
            f'pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, orientation: {{w: 1.0}}}}'
        )
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world_name}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req_text
        ]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            return 'data: true' in res.stdout
        except Exception as e:
            self.get_logger().error(f"Failed to spawn ball {ball_name} in Gazebo: {e}")
            return False

    def publish_telemetry(self):
        telemetry = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'balls_remaining': self.balls_remaining,
            'total_chambers': self.total_chambers,
            'current_chamber': self.current_chamber,
            'total_dropped': self.total_dropped,
            'carousel_angle_deg': round(math.degrees(self.carousel_angle), 1),
            'is_ready': (self.balls_remaining > 0) and not self.is_busy,
            'uav_altitude_m': round(self.uav_pos[2], 2)
        }
        msg = String()
        msg.data = json.dumps(telemetry)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDropController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
