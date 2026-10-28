#!/usr/bin/env python3
"""
Test client for UAV Ball Drop Mechanism.
Demonstrates calling the drop service and inspecting status telemetry.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class DropClient(Node):
    def __init__(self):
        super().__init__('test_drop_client')
        self.cli = self.create_client(Trigger, '/ball_drop/drop')
        self.reload_cli = self.create_client(Trigger, '/ball_drop/reload')
        self.create_subscription(String, '/ball_drop/status', self.status_cb, 10)
        self.last_status = ""

    def status_cb(self, msg: String):
        self.last_status = msg.data

    def drop_ball(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ball_drop/drop service...')
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def reload(self):
        while not self.reload_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ball_drop/reload service...')
        req = Trigger.Request()
        future = self.reload_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = DropClient()
    
    num_drops = 1
    if len(sys.argv) > 1:
        try:
            num_drops = int(sys.argv[1])
        except ValueError:
            num_drops = 1

    print(f"\n==========================================")
    print(f" UAV Ball Drop Mechanism Test Client")
    print(f" Executing {num_drops} ball drop sequence...")
    print(f"==========================================\n")
    
    for i in range(num_drops):
        res = client.drop_ball()
        print(f"[DROP #{i+1}] Result: {res.message}")
        time.sleep(1.2)
        
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
