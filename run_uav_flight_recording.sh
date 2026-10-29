#!/usr/bin/env bash
set -e

WORKSPACE_DIR="/home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws"
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/models:$WORKSPACE_DIR/src/uav_ball_drop/models:${GZ_SIM_RESOURCE_PATH}"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"

RAW_VIDEO="/tmp/raw_uav_flight_recording.mp4"
FINAL_VIDEO="$WORKSPACE_DIR/uav_flight_drop_recording.mp4"

echo "=========================================================="
echo " Starting Airborne UAV Flight & Drop Simulation..."
echo "=========================================================="

WORLD_FILE="$WORKSPACE_DIR/install/uav_ball_drop/share/uav_ball_drop/worlds/uav_flight_drop_world.sdf"

gz sim -s -r "$WORLD_FILE" > /tmp/gz_sim_flight.log 2>&1 &
GZ_PID=$!
sleep 3

# Start ros_gz_bridge for carousel control, clock, and UAV/Camera pose control
ros2 run ros_gz_bridge parameter_bridge \
  '/ball_drop/carousel_cmd@std_msgs/msg/Float64]gz.msgs.Double' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/world/uav_flight_world/set_pose@ros_gz_interfaces/srv/SetEntityPose' > /tmp/gz_bridge_flight.log 2>&1 &
BRIDGE_PID=$!

# Start ros_gz_image bridge for flight camera stream
ros2 run ros_gz_image image_bridge /camera/image > /tmp/gz_image_flight.log 2>&1 &
IMAGE_BRIDGE_PID=$!

# Start ball_drop_controller node configured for airborne UAV operation
ros2 run uav_ball_drop ball_drop_controller --ros-args \
  -p world_name:=uav_flight_world \
  -p total_chambers:=6 \
  -p mechanism_pose_z:=1.55 \
  -p aperture_z_offset:=-0.098 > /tmp/ball_drop_controller_flight.log 2>&1 &
NODE_PID=$!

cleanup() {
  echo ""
  echo "Shutting down flight simulation nodes..."
  kill $NODE_PID 2>/dev/null || true
  kill $IMAGE_BRIDGE_PID 2>/dev/null || true
  kill $BRIDGE_PID 2>/dev/null || true
  kill $GZ_PID 2>/dev/null || true
  wait $NODE_PID 2>/dev/null || true
  wait $IMAGE_BRIDGE_PID 2>/dev/null || true
  wait $BRIDGE_PID 2>/dev/null || true
  wait $GZ_PID 2>/dev/null || true
  echo "All flight processes cleanly stopped."
}
trap cleanup EXIT

echo "Waiting for flight camera and drop services to stabilize..."
sleep 3

echo "Recording autonomous UAV flight, hover & drop mission (16 seconds)..."
/home/paneendra/opencv-learning/.venv/bin/python \
  "$WORKSPACE_DIR/src/uav_ball_drop/scripts/record_uav_flight_drop.py" "$RAW_VIDEO"

echo "Encoding clean H.264 video with high quality (CRF 18)..."
ffmpeg -y -i "$RAW_VIDEO" -c:v libx264 -preset fast -crf 18 -pix_fmt yuv420p "$FINAL_VIDEO"
rm -f "$RAW_VIDEO"

# Copy to user's brain artifacts directory
ARTIFACT_DIR="/home/paneendra/.gemini/antigravity/brain/d20601b3-0a83-40d9-83b8-4f8266c525b7"
cp "$FINAL_VIDEO" "$ARTIFACT_DIR/uav_flight_drop_recording.mp4" || true
cp "$WORKSPACE_DIR"/snapshot_flight_*.png "$ARTIFACT_DIR/" 2>/dev/null || true
cp "$WORKSPACE_DIR"/snapshot_airborne_*.png "$ARTIFACT_DIR/" 2>/dev/null || true
cp "$WORKSPACE_DIR"/snapshot_forward_*.png "$ARTIFACT_DIR/" 2>/dev/null || true
cp "$WORKSPACE_DIR"/uav_flight_validation_metrics.json "$ARTIFACT_DIR/" 2>/dev/null || true

echo ""
echo "=========================================================="
echo " UAV Flight Video recording complete: $FINAL_VIDEO"
echo " Size: $(ls -lh "$FINAL_VIDEO" | awk '{print $5}')"
echo "=========================================================="
