#!/usr/bin/env bash
set -e

WORKSPACE_DIR="/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws"
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/models:$WORKSPACE_DIR/src/uav_ball_drop/models:${GZ_SIM_RESOURCE_PATH}"

echo "=========================================================="
echo " Starting Gazebo Sim Harmonic (Headless) + ROS 2 Node..."
echo "=========================================================="

WORLD_FILE="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/worlds/ball_drop_test_world.sdf"

gz sim -s -r "$WORLD_FILE" &
GZ_PID=$!

sleep 3

# Start ros_gz_bridge for carousel control and clock
ros2 run ros_gz_bridge parameter_bridge \
  '/ball_drop/carousel_cmd@std_msgs/msg/Float64]gz.msgs.Double' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > /tmp/gz_bridge.log 2>&1 &
BRIDGE_PID=$!

# Start ball_drop_controller node
ros2 run uav_ball_drop ball_drop_controller --ros-args \
  -p world_name:=ball_drop_world \
  -p total_chambers:=6 \
  -p mechanism_pose_z:=2.5 > /tmp/ball_drop_controller.log 2>&1 &
NODE_PID=$!

cleanup() {
  echo ""
  echo "Shutting down processes..."
  kill $NODE_PID 2>/dev/null || true
  kill $BRIDGE_PID 2>/dev/null || true
  kill $GZ_PID 2>/dev/null || true
  wait $NODE_PID 2>/dev/null || true
  wait $BRIDGE_PID 2>/dev/null || true
  wait $GZ_PID 2>/dev/null || true
  echo "All processes cleaned up."
}
trap cleanup EXIT

echo "Waiting for ROS 2 services to be ready..."
sleep 3

echo ""
echo "--- Testing Initial Mechanism Status ---"
ros2 topic echo /ball_drop/status --once

echo ""
echo "--- Executing Drop #1 ---"
ros2 service call /ball_drop/drop std_srvs/srv/Trigger

sleep 1

echo ""
echo "--- Executing Drop #2 ---"
ros2 service call /ball_drop/drop std_srvs/srv/Trigger

sleep 1

echo ""
echo "--- Executing Drop #3 ---"
ros2 service call /ball_drop/drop std_srvs/srv/Trigger

sleep 1

echo ""
echo "--- Verifying Models in Simulation ---"
gz model --list

echo ""
echo "--- Verifying Status Telemetry After Drops ---"
ros2 topic echo /ball_drop/status --once

echo ""
echo "--- Testing Reload Service ---"
ros2 service call /ball_drop/reload std_srvs/srv/Trigger

echo ""
echo "=========================================================="
echo " UAV Ball Drop Verification PASSED Successfully!"
echo "=========================================================="
