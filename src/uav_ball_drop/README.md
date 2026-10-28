# UAV Ball Drop Mechanism Simulation (Gazebo Sim Harmonic & ROS 2 Jazzy)

A high-fidelity simulation model, mechanical CAD recreation, and autonomous drop controller for an aerial robotics ball drop mechanism. Developed for autonomous target drop missions (inspired by **AGH Embedded Vision Systems Group / Avader UAV** research initiatives and precision competitions).

---

## 1. Overview & Mechanical Design

Based on the laboratory CAD specification:
* **Orientation Correction**: The reference CAD render showed the device inverted ("upside down"). When integrated under the UAV airframe, the aperture plate faces downward towards the ground.
* **Cylindrical Drum Housing**: 160 mm outer diameter, 52 mm height, housing 6 internal ball chambers.
* **Drop Aperture**: Off-center circular exit port (48 mm diameter) positioned at radial orbit $R = 48\text{ mm}$, providing smooth clearance for standard 40 mm competition balls.
* **Central Anodized Hub**: Orange circular disc with an 8-bolt circular pattern matching standard motor/encoder mountings.
* **6 Perimeter L-Bracket Clamps**: Distributed at $60^\circ$ intervals with curved inner clamping lips and 3-bolt triangular mounting patterns.
* **Internal 6-Chamber Revolving Carousel**: Actuated by a continuous revolute joint (`carousel_joint`) indexed at $60^\circ$ ($\frac{\pi}{3}\text{ rad}$) steps per release.
* **Payload Ball**: 40 mm diameter, 45 g competition sphere with realistic restitution and contact bounce dynamics.

---

## 2. System Architecture

```
                      +---------------------------------------+
                      |       Autonomy / Flight Computer      |
                      +---------------------------------------+
                                          |
                      /ball_drop/drop     |  /ball_drop/status
                     (std_srvs/Trigger)   |  (std_msgs/String JSON)
                                          v
                      +---------------------------------------+
                      |         ball_drop_controller          |
                      |            (ROS 2 Jazzy)              |
                      +---------------------------------------+
                           |                             |
      /ball_drop/carousel_cmd                            | /world/.../create
     (Float64 -> gz.msgs.Double)                         | (gz.msgs.EntityFactory)
                           v                             v
           +-------------------------------+   +-------------------+
           |    Gazebo Sim Harmonic        |   | Ballistic Payload |
           |  - JointPositionController    |   |  - Real Gravity   |
           |  - 60° Indexing Stepper       |   |  - Target Drop    |
           +-------------------------------+   +-------------------+
```

### ROS 2 Interface

| Interface | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/ball_drop/drop` | `std_srvs/srv/Trigger` | Service | Indexes carousel $60^\circ$, drops 1 ball, updates telemetry |
| `/ball_drop/reload` | `std_srvs/srv/Trigger` | Service | Resets carousel to $0^\circ$ and reloads 6 balls |
| `/ball_drop/cmd_drop` | `std_msgs/msg/Empty` | Subscriber | Instant drop trigger topic |
| `/ball_drop/cmd_carousel_angle` | `std_msgs/msg/Float64` | Subscriber | Manual angular positioning of carousel (radians) |
| `/ball_drop/status` | `std_msgs/msg/String` | Publisher | Real-time JSON telemetry (remaining, chamber, angle, ready) |
| `/model/uav/odometry` | `nav_msgs/msg/Odometry` | Subscriber | Inherits UAV speed vector for accurate flight ballistics |

---

## 3. Quickstart Guide

### Prerequisites
* Ubuntu 24.04 LTS (or 22.04)
* ROS 2 Jazzy (or Iron / Humble)
* Gazebo Sim Harmonic (v8.x)
* `ros_gz_sim`, `ros_gz_bridge`

### Build the Workspace
```bash
cd /home/paneendra/.gemini/antigravity/scratch/uav_ball_drop_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Run Automated Headless Verification
To execute the complete self-testing suite (launches Gazebo Sim headless, initializes ROS 2 node, triggers drops, verifies physics, and tests reload):
```bash
./test_simulation.sh
```

### Run Interactive Simulation (Gazebo GUI)
```bash
source install/setup.bash
ros2 launch uav_ball_drop sim_standalone.launch.py
```

### Trigger Drops from Command Line
In a second terminal:
```bash
source install/setup.bash

# Trigger a drop via ROS 2 Service:
ros2 service call /ball_drop/drop std_srvs/srv/Trigger

# Or trigger using the Python test client:
ros2 run uav_ball_drop test_drop_client 3  # Drops 3 balls in sequence

# Check live mechanism telemetry:
ros2 topic echo /ball_drop/status

# Reload mechanism:
ros2 service call /ball_drop/reload std_srvs/srv/Trigger
```

---

## 4. Models Included

1. **`uav_ball_drop_mechanism`**: Standalone mechanism model with high-resolution 3D visual meshes (`.obj` with `.mtl` materials), accurate mass properties, inertial tensors, and collision bounds.
2. **`payload_ball`**: Competition ball with restitution, contact friction, and aerodynamic properties.
3. **`uav_with_ball_drop`**: Quadrotor airframe integrated with the mechanism mounted directly to its belly, ready for PX4 SITL / autonomous waypoint missions.
4. **`ball_drop_test_world.sdf`**: Test environment with a 2.5 m suspension rig and high-visibility concentric bullseye drop target.

---

## 5. Repository Structure

```
uav_ball_drop_ws/
├── test_simulation.sh                 # End-to-end automated verification script
├── README.md                          # Documentation
└── src/
    └── uav_ball_drop/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── launch/
        │   └── sim_standalone.launch.py
        ├── models/
        │   ├── uav_ball_drop_mechanism/ # Housing, brackets, rotor, SDF & meshes
        │   ├── payload_ball/           # Ball payload model
        │   └── uav_with_ball_drop/     # Full UAV integrated model
        ├── worlds/
        │   └── ball_drop_test_world.sdf# Test world with target bullseye
        ├── urdf/
        │   └── ball_drop_mechanism.urdf.xacro # ROS 2 URDF / RViz definition
        ├── scripts/
        │   └── generate_meshes.py      # Procedural CAD mesh generator
        └── uav_ball_drop/
            ├── ball_drop_controller.py # Core ROS 2 drop controller node
            └── test_drop_client.py     # Test CLI client
```
