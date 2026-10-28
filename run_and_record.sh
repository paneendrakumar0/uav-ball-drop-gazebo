#!/usr/bin/env bash
set -e

WORKSPACE_DIR="/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws"
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/models:$WORKSPACE_DIR/src/uav_ball_drop/models:${GZ_SIM_RESOURCE_PATH}"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"

RAW_VIDEO="/tmp/raw_sim_recording.mp4"
FINAL_VIDEO="$WORKSPACE_DIR/ball_drop_simulation_recording.mp4"

echo "=========================================================="
echo " Starting Gazebo Sim + Dual-Camera Recording Pipeline..."
echo "=========================================================="

WORLD_FILE="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/worlds/ball_drop_test_world.sdf"

gz sim -s -r "$WORLD_FILE" > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!
sleep 3

# Start ros_gz_bridge for carousel control and clock
ros2 run ros_gz_bridge parameter_bridge \
  '/ball_drop/carousel_cmd@std_msgs/msg/Float64]gz.msgs.Double' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > /tmp/gz_bridge.log 2>&1 &
BRIDGE_PID=$!

# Start ros_gz_image bridge for Overview and Close-up cameras
ros2 run ros_gz_image image_bridge /camera/image /camera_closeup/image > /tmp/gz_image.log 2>&1 &
IMAGE_BRIDGE_PID=$!

# Start ball_drop_controller node
ros2 run uav_ball_drop ball_drop_controller --ros-args \
  -p world_name:=ball_drop_world \
  -p total_chambers:=6 \
  -p mechanism_pose_z:=2.5 > /tmp/ball_drop_controller.log 2>&1 &
NODE_PID=$!

cleanup() {
  echo ""
  echo "Stopping simulation nodes..."
  kill $NODE_PID 2>/dev/null || true
  kill $IMAGE_BRIDGE_PID 2>/dev/null || true
  kill $BRIDGE_PID 2>/dev/null || true
  kill $GZ_PID 2>/dev/null || true
  wait $NODE_PID 2>/dev/null || true
  wait $IMAGE_BRIDGE_PID 2>/dev/null || true
  wait $BRIDGE_PID 2>/dev/null || true
  wait $GZ_PID 2>/dev/null || true
  echo "All processes cleanly shut down."
}
trap cleanup EXIT

echo "Waiting for camera feeds and drop services to stabilize..."
sleep 3

echo "Starting automated dual-camera recording (13 seconds)..."
/home/paneendra/opencv-learning/.venv/bin/python \
  "$WORKSPACE_DIR/src/uav_ball_drop/scripts/record_simulation.py" "$RAW_VIDEO"

echo "Encoding final H.264 video with maximum compatibility..."
ffmpeg -y -i "$RAW_VIDEO" -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p "$FINAL_VIDEO"
rm -f "$RAW_VIDEO"

# Copy to user's brain artifacts directory for easy access
ARTIFACT_DIR="/home/paneendra/.gemini/antigravity/brain/d20601b3-0a83-40d9-83b8-4f8266c525b7"
cp "$FINAL_VIDEO" "$ARTIFACT_DIR/ball_drop_simulation_recording.mp4" || true
cp /tmp/snapshot_*.png "$ARTIFACT_DIR/" 2>/dev/null || true
cp /tmp/snapshot_*.png "$WORKSPACE_DIR/" 2>/dev/null || true

echo ""
echo "=========================================================="
echo " Video recording successfully completed!"
echo " Output path: $FINAL_VIDEO"
echo " File size: $(ls -lh "$FINAL_VIDEO" | awk '{print $5}')"
echo "=========================================================="
