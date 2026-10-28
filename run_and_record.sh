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
echo " Starting Gazebo Sim Harmonic (Clean Simulation Run)..."
echo "=========================================================="

WORLD_FILE="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/worlds/ball_drop_test_world.sdf"

gz sim -s -r "$WORLD_FILE" > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!
sleep 3

# Start ros_gz_bridge for carousel control, clock, and camera pose control
ros2 run ros_gz_bridge parameter_bridge \
  '/ball_drop/carousel_cmd@std_msgs/msg/Float64]gz.msgs.Double' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/world/ball_drop_world/set_pose@ros_gz_interfaces/srv/SetEntityPose' > /tmp/gz_bridge.log 2>&1 &
BRIDGE_PID=$!

# Start ros_gz_image bridge for primary camera stream
ros2 run ros_gz_image image_bridge /camera/image > /tmp/gz_image.log 2>&1 &
IMAGE_BRIDGE_PID=$!

# Start ball_drop_controller node
ros2 run uav_ball_drop ball_drop_controller --ros-args \
  -p world_name:=ball_drop_world \
  -p total_chambers:=6 \
  -p mechanism_pose_z:=1.50 > /tmp/ball_drop_controller.log 2>&1 &
NODE_PID=$!

cleanup() {
  echo ""
  echo "Shutting down simulation nodes..."
  kill $NODE_PID 2>/dev/null || true
  kill $IMAGE_BRIDGE_PID 2>/dev/null || true
  kill $BRIDGE_PID 2>/dev/null || true
  kill $GZ_PID 2>/dev/null || true
  wait $NODE_PID 2>/dev/null || true
  wait $IMAGE_BRIDGE_PID 2>/dev/null || true
  wait $BRIDGE_PID 2>/dev/null || true
  wait $GZ_PID 2>/dev/null || true
  echo "All processes cleanly stopped."
}
trap cleanup EXIT

echo "Waiting for camera and drop services to stabilize..."
sleep 3

echo "Recording cinematic simulation (22s: 11s wide view + drone flight glide + close-up drops)..."
/home/paneendra/opencv-learning/.venv/bin/python \
  "$WORKSPACE_DIR/src/uav_ball_drop/scripts/record_simulation.py" "$RAW_VIDEO"

echo "Encoding clean H.264 video with high quality (CRF 18)..."
ffmpeg -y -i "$RAW_VIDEO" -c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p "$FINAL_VIDEO"
rm -f "$RAW_VIDEO"

# Copy to user's brain artifacts directory
ARTIFACT_DIR="/home/paneendra/.gemini/antigravity/brain/d20601b3-0a83-40d9-83b8-4f8266c525b7"
cp "$FINAL_VIDEO" "$ARTIFACT_DIR/ball_drop_simulation_recording.mp4" || true
cp "$WORKSPACE_DIR"/snapshot_*.png "$ARTIFACT_DIR/" 2>/dev/null || true

echo ""
echo "=========================================================="
echo " Video recording complete: $FINAL_VIDEO"
echo " Size: $(ls -lh "$FINAL_VIDEO" | awk '{print $5}')"
echo "=========================================================="
