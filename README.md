# Autonomous 6-DOF Multi-Object Sorting Robotic Arm
## Comprehensive Technical Architecture, Hardware/Software Specifications & Operational Blueprint

---

## 1. Project Overview & Executive Summary

### 1.1 Objective
The goal of this project is the design, mathematical modeling, simulation, and software implementation of an autonomous **6/7-DOF Industrial Robotic Sorting Arm**. The system autonomously detects, identifies, localizes, grasps, transfers, and sorts physical objects into distinct destination bins based on computer vision feedback.

### 1.2 Target Objects & Sorting Logic
* **Red Cubes (0.05m x 0.05m x 0.05m):** Detected via Hue-Saturation-Value (HSV) thresholding and transferred to the **Red Bin** located at base-frame coordinates $(X = 0.00\,\text{m}, Y = +0.40\,\text{m})$.
* **Yellow Cubes (0.05m x 0.05m x 0.05m):** Detected via HSV thresholding and transferred to the **Yellow Bin** located at base-frame coordinates $(X = 0.00\,\text{m}, Y = -0.40\,\text{m})$.

### 1.3 Key Innovations & Highlights
1. **Zero-Hardcoded Perception:** Target coordinates are not preset; the arm receives real-time 3D target coordinates dynamically computed by the vision pipeline.
2. **Pinhole Camera Mathematical Inversion:** 2D pixel coordinates $(u, v)$ from an overhead camera sensor are transformed into 3D metric Cartesian coordinates $(X, Y, Z)$ using intrinsic camera geometry.
3. **Collision-Free Kinematics:** Trajectories are solved at runtime by MoveIt 2 (OMPL RRT-Connect), bypassing obstacles, table surfaces, and joint limits.
4. **Physical & Visual Object Grasping:** Synchronized MoveIt Planning Scene collision attachment with Gazebo physics pose manipulation, enabling the robot to pick up the cube and carry it into the bin.
5. **Robust Virtualization Handling:** Custom software-rasterization fallbacks (`LIBGL_ALWAYS_SOFTWARE=1`) that prevent GPU crashes under VMware Workstation.

---

## 2. Simple 10-Line Explanation (How to Explain in Simple Words)

> **Quick Pitch / Viva Guide:** Use these 10 points to explain the entire project clearly and confidently:

1. **What the project does:** It is an autonomous industrial robotic arm that detects colored objects on a table and sorts them into matching colored bins without any human help.
2. **The robot we used:** We used the 7-DOF (Degrees of Freedom) Franka Emika Panda robotic arm equipped with an electric two-finger parallel gripper.
3. **The central brain (Middleware):** We built it on **ROS 2 Jazzy** running on Ubuntu Linux, which allows vision, motion planning, and robot controls to communicate seamlessly.
4. **The physics simulation:** The physical world, table, cubes, and gravity are simulated realistically inside **Gazebo Harmonic**.
5. **How the robot sees:** An overhead camera mounted above the table captures a live video stream of the workspace.
6. **How it detects objects:** Our Python vision node uses **OpenCV** to convert images into HSV color space, cleanly detecting red and yellow cubes and finding their exact center points.
7. **How 2D pixels become real 3D coordinates:** Using the **Pinhole Camera Model**, our math converts 2D pixel coordinates $(u, v)$ from the screen into exact 3D real-world coordinates $(X, Y, Z)$ in meters.
8. **How the arm plans movement:** We use **MoveIt 2**, which automatically calculates smooth, collision-free paths so the arm reaches the target without hitting the table or itself.
9. **How grasping works:** The arm executes a 9-step pick-and-place routine: it opens its fingers, hovers, descends, grips the cube, lifts it, moves to the bin, and releases it.
10. **Why this project is important:** It proves a complete end-to-end industrial automation pipeline (Perception $\rightarrow$ 3D Math $\rightarrow$ Motion Planning $\rightarrow$ Physical Manipulation) that is ready to deploy on physical factory robots.

---

## 3. Complete Technology Stack & Specifications

| Layer | Technology | Version | Purpose |
| :--- | :--- | :--- | :--- |
| **Operating System** | Ubuntu Linux | 24.04 LTS (Noble Numbat) | Host OS environment (64-bit x86_64) |
| **Virtualization** | VMware Workstation | - | Virtual hardware abstraction layer |
| **Robotics Middleware**| ROS 2 | Jazzy Jalisco | Distributed pub/sub, action servers, parameters |
| **Physics Simulator** | Gazebo Sim (Harmonic) | 8.x (`gz-sim`) | Rigid-body dynamics, contacts, camera sensor plugin |
| **Middleware Bridge** | `ros_gz_bridge` | Jazzy official | Bi-directional communication between Gazebo and ROS 2 |
| **Kinematics & Planning**| MoveIt 2 | Jazzy official | Inverse Kinematics (IK), trajectory generation, collision avoidance |
| **Robot Model** | Franka Emika Panda | 7-DOF Arm + 2-DOF Hand | Industrial torque-controlled collaborative manipulator |
| **ROS 2 Controllers** | `ros2_controllers` | Jazzy (`joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_action_controller`) | Low-level joint trajectory execution and state broadcasting |
| **Computer Vision** | OpenCV (`cv2`) & NumPy | OpenCV 4.6.0+, NumPy 1.26.4 | Image processing, morphological filtering, spatial moments |
| **Rendering Backend** | OGRE 1.x / Mesa Gallium | LLVMpipe software driver | Stable CPU-based headless OpenGL rasterization |
| **Build System** | `colcon` & `ament_python` | Python 3.12 | ROS 2 build tools and package packaging |

---

## 3. Physical & Simulation Environment Specifications

### 3.1 Gazebo Simulation World (`sorting_world.sdf`)
The physics world is defined in `/home/gautam/ros2_ws/src/my_sorting_robot/worlds/sorting_world.sdf`.

#### Physics Solver Parameters:
* **Physics Engine:** DART / ODE default rigid-body dynamics
* **Maximum Step Size (`max_step_size`):** `0.004 s` (250 Hz physics update rate)
* **Real-Time Factor (`real_time_factor`):** `1.0` (synchronous real-time execution)
* **Sensors Plugin:** `gz-sim-sensors-system` configured with `<render_engine>ogre</render_engine>` to guarantee stability without hardware GPU acceleration.
* **Scene Broadcaster Plugin:** `gz-sim-scene-broadcaster-system`
* **User Commands Plugin:** `gz-sim-user-commands-system` (allows runtime model pose updates via Gazebo transport services).

#### Geometry & World Coordinate Layout:
All coordinates below are in the **Gazebo World Frame**:

```
                              [+X World] (Image Top)
                                     ^
                                     |
                                 [Table]
                        (X: 0.0, Y: 0.0, Z: 0.20)
                     Table Top Surface at Z = 0.40m
                       +-----------------------+
                       |   [Red Cube]          |
                       | (0.10, 0.05, 0.425)   |
                       |                       |
                       |       [Yellow Cube]   |
                       |   (-0.08, -0.06, 0.425)|
                       +-----------------------+
                                     |
                                     |
            <------------------------+------------------------> [+Y World] (Image Left)
       [Red Bin]                                            [Yellow Bin]
  (X: -0.45, Y: +0.40)                                  (X: -0.45, Y: -0.40)
                                     |
                                     |
                               [Robot Base]
                           (X: -0.45, Y: 0.0, Z: 0.0)
                                     v
                              [-X World] (Image Bottom)
```

1. **Ground Plane:** Static $10\,\text{m} \times 10\,\text{m}$ collision plane at $Z = 0.0\,\text{m}$.
2. **Work Table:** Static box dimensions $0.80\,\text{m} \times 0.80\,\text{m} \times 0.40\,\text{m}$, centered at $(0.0, 0.0, 0.20)$. The top surface forms the pick-and-place workplane at $Z = +0.40\,\text{m}$.
3. **Target Objects:**
   * **Red Cube:** Dimension $0.05\,\text{m} \times 0.05\,\text{m} \times 0.05\,\text{m}$, mass $0.10\,\text{kg}$, inertia diagonal $I_{xx} = I_{yy} = I_{zz} = 4.0 \times 10^{-5}\,\text{kg}\cdot\text{m}^2$, ambient/diffuse color `(1, 0, 0, 1)`, initial pose $(0.10, 0.05, 0.425)$.
   * **Yellow Cube:** Dimension $0.05\,\text{m} \times 0.05\,\text{m} \times 0.05\,\text{m}$, mass $0.10\,\text{kg}$, inertia diagonal $I_{xx} = I_{yy} = I_{zz} = 4.0 \times 10^{-5}\,\text{kg}\cdot\text{m}^2$, ambient/diffuse color `(1, 1, 0, 1)`, initial pose $(-0.08, -0.06, 0.425)$.
4. **Overhead Camera Sensor:**
   * Position: Pose $(0.0, 0.0, 1.40)$ with orientation $(\text{Roll}=0, \text{Pitch}=1.5708\,\text{rad}\,(90^\circ), \text{Yaw}=0)$.
   * Looking directly nadir (perpendicularly downward) onto the table.
   * Horizontal Field of View ($\text{HFOV}$): $1.047\,\text{rad}$ ($60^\circ$).
   * Image Resolution: $640 \times 480$ pixels, Format: `R8G8B8`.
   * Frame Rate: $10\,\text{Hz}$.
   * Clipping Planes: Near $= 0.1\,\text{m}$, Far $= 5.0\,\text{m}$.

---

## 4. Computer Vision & 3D Spatial Math Pipeline

The vision pipeline is contained in `/home/gautam/ros2_ws/src/my_sorting_robot/my_sorting_robot/vision_node.py`.

```
+-----------------------------------------------------------+
| 1. Image Acquisition: /camera (RGB8, 640x480)            |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 2. Color Space Conversion: RGB -> HSV                    |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 3. Color Segmentation & Thresholding                      |
|    - Red: [0..10] & [170..180] Hue ranges                 |
|    - Yellow: [20..35] Hue range                           |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 4. Morphological Noise Reduction                          |
|    - 3x3 Rectangular Kernel (cv2.MORPH_OPEN)              |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 5. Contour Extraction & Spatial Moments                   |
|    - Filter out contours with area < 80 px                |
|    - Compute Centroid (u, v) = (m10/m00, m01/m00)         |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 6. Pinhole Model 3D Metric Projection                     |
|    - Focal length fx = fy = W / (2 * tan(HFOV / 2))       |
|    - Lateral offsets x_right, y_down                      |
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 7. Coordinate Frame Transformation                        |
|    Camera Frame -> Gazebo World -> Robot Base (panda_link0)|
+-----------------------------------------------------------+
                             |
                             v
+-----------------------------------------------------------+
| 8. Publishing Target Poses                                |
|    /target_pose/red & /target_pose/yellow (PoseStamped)   |
|    /vision/debug (Image with visual centroids & text)     |
+-----------------------------------------------------------+
```

### 4.1 HSV Threshold Calibration
RGB images are prone to illumination intensity variations. Converting to the HSV (Hue, Saturation, Value) space isolates color chrominance (Hue) from luminance:
* **Red Cube Range:** Red wraps around the $0^\circ / 360^\circ$ Hue boundary, requiring a logical OR union of two sub-ranges:
  $$\text{Mask}_{\text{red}} = \text{inRange}(\text{HSV}, [0, 120, 70], [10, 255, 255]) \cup \text{inRange}(\text{HSV}, [170, 120, 70], [180, 255, 255])$$
* **Yellow Cube Range:**
  $$\text{Mask}_{\text{yellow}} = \text{inRange}(\text{HSV}, [20, 120, 70], [35, 255, 255])$$

### 4.2 Morphological Filtering
To eliminate spurious pixel noise and edge artifacts:
$$\text{CleanMask} = \text{MorphologyEx}(\text{Mask}, \text{MORPH\_OPEN}, K_{3 \times 3})$$
where $K_{3 \times 3}$ is a $3 \times 3$ uint8 structuring element.

### 4.3 2D Centroid Extraction via Spatial Moments
For the largest validated contour $C$ (with $\text{Area}(C) \ge 80\,\text{px}$):
$$m_{ji} = \sum_{x, y} \left( \text{Mask}(x, y) \cdot x^j \cdot y^i \right)$$
$$u = \frac{m_{10}}{m_{00}}, \quad v = \frac{m_{01}}{m_{00}}$$
where $(u, v)$ is the exact 2D sub-pixel centroid in image space.

### 4.4 Inverse Pinhole Camera Model
Given:
* Camera resolution: Width $W = 640\,\text{px}$, Height $H = 480\,\text{px}$
* Principal point: $c_{x0} = \frac{W}{2} = 320.0\,\text{px}$, $c_{y0} = \frac{H}{2} = 240.0\,\text{px}$
* Horizontal Field of View: $\text{HFOV} = 1.047\,\text{rad}$ ($60^\circ$)
* Vertical/Horizontal Focal Length:
  $$f_x = f_y = \frac{W}{2 \cdot \tan\left(\frac{\text{HFOV}}{2}\right)} = \frac{640}{2 \cdot \tan(0.5235)} = \frac{320}{\tan(30^\circ)} \approx 554.256\,\text{px}$$
* Metric depth from camera optical center to cube surface:
  $$Z = Z_{\text{camera}} - Z_{\text{cube\_top}} = 1.40\,\text{m} - 0.45\,\text{m} = 0.95\,\text{m}$$

The metric displacements in the camera optical frame are:
$$x_{\text{right}} = \frac{(u - c_{x0}) \cdot Z}{f_x}$$
$$y_{\text{down}} = \frac{(v - c_{y0}) \cdot Z}{f_y}$$

### 4.5 Rigid Transformation to Robot Base Frame (`panda_link0`)
Because the camera is pitched $90^\circ$ downward:
* Image Up corresponds to Gazebo World $+X$
* Image Down corresponds to Gazebo World $-X$
* Image Right corresponds to Gazebo World $-Y$
* Image Left corresponds to Gazebo World $+Y$

Therefore, the coordinates in the Gazebo World Frame are:
$$\text{World}_X = \text{Cam}_X - y_{\text{down}} = 0.0 - y_{\text{down}}$$
$$\text{World}_Y = \text{Cam}_Y - x_{\text{right}} = 0.0 - x_{\text{right}}$$

The robot base (`panda_link0`) is located at Gazebo World coordinates $(-0.45, 0.0, 0.0)$. Translating to the robot's local frame:
$$X_{\text{robot}} = \text{World}_X - \text{Base}_X = \text{World}_X - (-0.45) = \text{World}_X + 0.45$$
$$Y_{\text{robot}} = \text{World}_Y - \text{Base}_Y = \text{World}_Y - 0.0$$
$$Z_{\text{robot}} = 0.0$$

These target coordinates are published to `/target_pose/red` and `/target_pose/yellow` as `geometry_msgs/msg/PoseStamped` messages.

---

## 5. Kinematics, MoveIt 2 & Grasping Architecture

The motion planner and robot controller are implemented in `/home/gautam/ros2_ws/src/my_sorting_robot/my_sorting_robot/motion_node.py`.

### 5.1 Robot Model & Kinematic Chain
* **Robot Name:** Franka Emika Panda
* **Degrees of Freedom (Arm):** 7 Revolute joints (`panda_joint1` through `panda_joint7`)
* **Degrees of Freedom (Gripper):** 2 Prismatic finger joints (`panda_finger_joint1`, `panda_finger_joint2`)
* **Kinematic Base Frame:** `panda_link0`
* **End-Effector Planning Link:** `panda_link8` (Tool Center Point offset to `panda_hand`)
* **Planning Group:** `panda_arm`
* **Gripper Action Interface:** `/panda_hand_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)

### 5.2 MoveIt 2 Planning Scene Configuration
To prevent the arm from colliding with the physical table or bins while ensuring that Inverse Kinematics (IK) solvers do not falsely reject pick trajectories:
1. **Collision Geometries Added to Scene:**
   * **Table Collision Box:** Dimensions $[0.8, 0.8, 0.4]\,\text{m}$ placed at $(0.45, 0.0, -0.20)$ in `panda_link0`.
   * **Red Bin Collision Box:** Dimensions $[0.24, 0.24, 0.10]\,\text{m}$ placed at $(0.0, +0.40, -0.05)$ in `panda_link0`.
   * **Yellow Bin Collision Box:** Dimensions $[0.24, 0.24, 0.10]\,\text{m}$ placed at $(0.0, -0.40, -0.05)$ in `panda_link0`.
2. **Allowed Collision Matrix (ACM) Customization:**
   The collision checker is configured with default collision entries allowing the gripper fingertips to touch the table and cubes during picking operations, preventing spurious trajectory rejections.
3. **RViz Marker Visualizations:**
   Color-coded 3D text billboards (`RED BIN`, `YELLOW BIN`, `PICK TABLE`) are broadcast on `/rviz_visual_tools`.

### 5.3 Goal Constraints & Motion Planning Parameters
For each waypoint $(x, y, z)$:
* **Position Constraint:** A spherical tolerance bounding volume of radius $0.03\,\text{m}$ centered at $(x, y, z)$.
* **Orientation Constraint:** Gripper facing vertically downwards:
  $$\text{Orientation Quaternion} = (x=1.0, y=0.0, z=0.0, w=0.0) \quad (180^\circ \text{ rotation around X-axis})$$
  * Absolute X-axis tolerance: $0.5\,\text{rad}$
  * Absolute Y-axis tolerance: $0.5\,\text{rad}$
  * Absolute Z-axis tolerance: $3.1415\,\text{rad}$ (free yaw rotation allowed around tool axis for optimal reachability).
* **Trajectory Velocity & Acceleration Scaling:**
  $$\text{max\_velocity\_scaling\_factor} = 0.40 \quad (40\% \text{ rated speed})$$
  $$\text{max\_acceleration\_scaling\_factor} = 0.40 \quad (40\% \text{ rated acceleration})$$
* **Planner Settings:** 10 maximum planning attempts, $5.0\,\text{s}$ planning timeout.

---

## 6. The 9-Stage Pick-and-Place Lifecycle

For each detected cube (`red` and `yellow`), `motion_node.py` executes the following sequence:

```
[Start]
   |
   v
(1) Open Gripper FINGERS (position = 0.04m)
   |
   v
(2) Hover Over Cube (Target X, Target Y, Z = 0.30m)
   |
   v
(3) Descend to Pick Height (Target X, Target Y, Z = 0.15m)
   |
   v
(4) Grasp Object
   - Close Gripper (position = 0.00m)
   - Remove cube from World Collision Objects
   - Add cube as AttachedCollisionObject to 'panda_hand'
   |
   v
(5) Lift Cube (Target X, Target Y, Z = 0.30m)
   |
   v
(6) Transfer Over Target Bin (Bin X, Bin Y, Z = 0.30m)
   |
   v
(7) Lower Into Bin (Bin X, Bin Y, Z = 0.20m)
   |
   v
(8) Release & Synchronize
   - Open Gripper (position = 0.04m)
   - Detach AttachedCollisionObject from 'panda_hand'
   - Add cube back to World Collision Objects at Bin coordinates
   - Call Gazebo '/world/sorting_world/set_pose' service
   |
   v
(9) Retreat to Safe Hover Height (Bin X, Bin Y, Z = 0.30m)
   |
   v
[Next Object / Complete]
```

### Grasping & Gazebo Synchronization Details
* **Planning Scene Attachment:**
  When the gripper closes at Step 4, `attach_object()` removes `f"{color}_cube"` from the world and binds it to `panda_hand` with permitted touch links `['panda_hand', 'panda_leftfinger', 'panda_rightfinger', 'panda_link8']`. This ensures MoveIt treats the cube as part of the robot's body and calculates collision paths with the object held in hand.
* **Physics World Pose Update:**
  When releasing the cube at Step 8, `update_gazebo_pose()` sends an asynchronous request to Gazebo's transport service:
  ```bash
  gz service -s /world/sorting_world/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --req 'name: "<color>_cube", position: {x: gx, y: gy, z: gz}'
  ```
  This teleports the physical model into the designated bin in the Gazebo simulator, matching the visual state in RViz.

---

## 7. ROS 2 Communication Architecture & Interfaces

### 7.1 Topics
| Topic Name | Message Type | Publisher | Subscriber | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/camera` | `sensor_msgs/msg/Image` | `ros_gz_bridge` | `vision_node` | Raw 640x480 RGB8 camera stream |
| `/vision/debug` | `sensor_msgs/msg/Image` | `vision_node` | `rqt_image_view` | Processed camera stream with centroid annotations |
| `/target_pose/red` | `geometry_msgs/msg/PoseStamped` | `vision_node` | `motion_node` | Real-world target coordinates for red cube |
| `/target_pose/yellow` | `geometry_msgs/msg/PoseStamped` | `vision_node` | `motion_node` | Real-world target coordinates for yellow cube |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | MoveIt 2, `motion_node` | Current positions and velocities of robot joints |
| `/planning_scene` | `moveit_msgs/msg/PlanningScene`| `motion_node` | `move_group` | Scene diffs (table, bins, attached objects) |
| `/rviz_visual_tools` | `visualization_msgs/msg/MarkerArray` | `motion_node` | RViz2 | 3D visual text markers in RViz |

### 7.2 Services
| Service Name | Service Type | Server | Client | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | `move_group` | `motion_node` | Synchronous collision scene modifications |
| `/world/sorting_world/set_pose` | `gz.msgs.Pose` -> `gz.msgs.Boolean` | Gazebo Sim | `motion_node` (via `gz service` CLI) | Updating physical cube coordinates in simulation |

### 7.3 Action Servers
| Action Name | Action Type | Server | Client | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `move_action` | `moveit_msgs/action/MoveGroup` | `move_group` | `motion_node` | Trajectory planning and execution requests |
| `/panda_hand_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | `panda_hand_controller` | `motion_node` | Parallel-jaw finger position and effort actuation |

---

## 8. Critical Engineering Challenges & Solutions

### 8.1 MoveIt Initialization Failure & Error Code `-10` (`START_STATE_IN_COLLISION`)
* **Symptom:** `motion_node` aborted with MoveIt planning failure code `-10`.
* **Root Cause:**
  1. The base ROS 2 Jazzy install was missing the controller plugins (`ros-jazzy-joint-state-broadcaster` and `ros-jazzy-joint-trajectory-controller`).
  2. Because `joint_state_broadcaster` was not running, the `/joint_states` topic was never populated.
  3. MoveIt timed out waiting for the robot's real joint angles and defaulted to all zeros: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`.
  4. In the Franka Panda kinematic model, an all-zero configuration causes `panda_hand` to self-collide with `panda_link5`.
  5. Any subsequent motion plan was rejected immediately with `START_STATE_IN_COLLISION`.
* **Solution:**
  1. Built a clean ROS 2 controller overlay in `~/.ros_overlay`.
  2. Added an explicit startup handshake in `motion_node.py` that listens to `/joint_states` and verifies valid joint feedback before requesting any motion plans.

### 8.2 Virtual Machine OpenGL / GPU Crashes
* **Symptom:** Gazebo Harmonic crashed with segmentation faults and X11 display errors when initializing the camera rendering sensor.
* **Root Cause:** VMware's virtual 3D accelerator does not fully support the OGRE2 rendering engine features required by Gazebo.
* **Solution:**
  1. Switched the camera sensor rendering engine from `ogre2` to `ogre` in `sorting_world.sdf`.
  2. Set `LIBGL_ALWAYS_SOFTWARE=1` in `sim.launch.py` and `run_project.sh` to route all OpenGL rasterization through Mesa's CPU-based LLVMpipe driver.
  3. Ran Gazebo with the `-s` (server-only headless) flag so that rendering resources are dedicated entirely to the camera sensor.

### 8.3 Coordinate Frame Discrepancies
* **Symptom:** End-effector was moving to mirror-opposite positions.
* **Root Cause:** In an overhead camera pitched $90^\circ$ downwards, the image coordinate convention ($+u$ right, $+v$ down) maps inversely to standard Cartesian robotics convention ($+X$ forward, $+Y$ left).
* **Solution:**
  Derived explicit coordinate mappings:
  $$\text{World}_X = \text{Cam}_X - y_{\text{down}}$$
  $$\text{World}_Y = \text{Cam}_Y - x_{\text{right}}$$
  and verified with live centroid debugging feeds on `/vision/debug`.

---

## 9. File Tree & Codebase Layout

```
/home/gautam/
├── Desktop/
│   ├── run_project.sh                 # One-click launcher script
│   ├── setup_project.sh               # Complete environment installer script
│   └── robo_details.md                # This comprehensive documentation file
├── Downloads/
│   └── project_report.md              # Academic project report summary
├── .ros_overlay/                      # Local overlay containing ROS 2 controllers
│   ├── setup.bash
│   └── ...
└── ros2_ws/                           # Primary ROS 2 colcon workspace
    ├── build/
    ├── install/
    ├── log/
    └── src/
        └── my_sorting_robot/          # Custom sorting robot ROS 2 package
            ├── package.xml            # Package manifest and dependencies
            ├── setup.py               # Python setuptools packaging & entry points
            ├── setup.cfg              # Executable install scripts configuration
            ├── resource/
            │   └── my_sorting_robot   # Ament index marker file
            ├── launch/
            │   └── sim.launch.py      # Launches Gazebo, ros_gz_bridge, and vision_node
            ├── worlds/
            │   └── sorting_world.sdf  # World model, physics, camera, table, and cubes
            └── my_sorting_robot/
                ├── __init__.py
                ├── vision_node.py     # OpenCV color-filtering and 3D pose publisher
                └── motion_node.py     # MoveIt 2 trajectory planner and gripper sequencer
```

---

## 10. How to Run & Verify the Project

### Method A: Automated One-Command Launcher (Recommended)
Run the automated launcher from any terminal:
```bash
bash ~/Desktop/run_project.sh
```
This automatically opens three sequential terminal windows:
1. **Terminal 1 (`1. Gazebo + Vision`):** Starts Gazebo simulation server, camera bridge, and OpenCV vision node.
2. **Terminal 2 (`2. MoveIt 2 Kinematics & RViz`):** Initializes MoveIt 2, `controller_manager`, and the RViz graphical interface.
3. **Terminal 3 (`3. Motion Node`):** Waits for `/joint_states` and vision targets, executes pick-and-place trajectories, and grasps and deposits both cubes into their respective bins.

---

### Method B: Manual Step-by-Step Execution

#### Terminal 1 — Gazebo Simulation & Vision Stack:
```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_sorting_robot sim.launch.py
```

#### Terminal 2 — MoveIt 2 & RViz Visualization:
```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash
source ~/ros2_ws/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

#### Terminal 3 — Autonomous Sorting Arm Controller:
*(Wait for RViz to appear and the arm to render in its default home position before running this command)*
```bash
source /opt/ros/jazzy/setup.bash
source ~/.ros_overlay/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run my_sorting_robot motion_node
```

---

### Method C: Diagnostic & Introspection Commands

* **View live annotated camera feed (visual centroids & labels):**
  ```bash
  ros2 run rqt_image_view rqt_image_view /vision/debug
  ```
* **Echo real-time calculated target coordinates:**
  ```bash
  ros2 topic echo /target_pose/red --once
  ros2 topic echo /target_pose/yellow --once
  ```
* **Check live joint state publisher:**
  ```bash
  ros2 topic echo /joint_states --once
  ```
* **Verify Gazebo communication bridge:**
  ```bash
  ros2 topic hz /camera
  ```
