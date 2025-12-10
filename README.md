# 🤖 ROS 2 Wall Following (IR Sensor-Based)

This repository contains the ROS 2 packages developed for implementing an autonomous wall-following behavior on the **TurtleBot 4** platform using its built-in **Infrared (IR) proximity sensors**.

The initial design, which relied on LiDAR, was successfully adapted to use the higher-reliability IR sensor data for robust control in dynamic environments.


## Demonstration Video

The following is a link to a Youtube Shorts video demonstrating basic wall-following functionality in a lab envvironment: [Youtube Shorts Link](https://youtube.com/shorts/YgjRR1WlokQ?feature=share)

---

## 🚀 Overview and Control Strategy

The system is split into three modular ROS 2 packages:

### 1. `wf_perception` (Wall Perception Node)

This node is responsible for reading the raw sensor data and calculating the error signals.

* **Input Topic:** `/ir_intensity` (from `irobot_create_msgs/msg/IrIntensityVector`)
* **Output Topic:** `/wall_tracking` (to `std_msgs/msg/Float64MultiArray`)
* **Sensor Logic:** It performs an **initial selection** on startup to determine the closest wall (left or right) and commits to that side for the duration of the run.
* **Error Calculation:** Publishes two core values:
    1.  **Distance Error ( $$\Delta D$$ ):** Calculated from the rear-side IR sensor intensity relative to a calibrated `target_intensity`.
    2.  **Angle Error ( $$\Delta \theta$$ ):** Calculated from the difference between the **front-side** IR sensor intensity and the **rear-side** IR sensor intensity.

### 2. `wf_controller` (Wall Controller Node)

This node implements the control law based on the calculated errors.

* **Input Topic:** `/wall_tracking`
* **Output Topic:** `/cmd_vel` (to `geometry_msgs/msg/Twist`)
* **Control Law:** Uses a **Proportional-Derivative (PD) control structure** where the angular velocity ($$\omega$$) is determined by the linear combination of the two errors:
    $$\omega = k_d \cdot \Delta D + k_{\theta} \cdot \Delta \theta$$
* **Safety Logic:** Implements a **dynamic linear velocity ($$v$$) switch** that reduces forward speed during large corrective turns and ensures a minimum guaranteed speed to prevent stalling.

### 3. `wf_bringup` (Launch/Setup)

Contains launch files for starting all necessary nodes simultaneously.

---

## ⚙️ Installation and Setup

This project assumes you have a functional ROS 2 Humble/Iron workspace (e.g., `~/ros2_ws`) set up for the TurtleBot 4.

1.  **Clone the repository into your ROS 2 workspace:**
    ```bash
   
    git clone [REPO_URL]
    ```

2.  **Install Dependencies:** Ensure all required packages (`irobot_create_msgs`, `nav_msgs`, `std_msgs`, etc.) are installed on your system.

3.  **Build the Workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

4.  **Source the Environment:** **This must be done in every new terminal window.**
    ```bash
    source install/setup.bash
    ```

---

## 🏃 Running the Wall Follower

1.  **Start the TurtleBot 4 base drivers.**
2.  **Launch the control system:**
    ```bash
    ros2 launch wf_bringup wf_bringup.launch.py
    ```

## 📈 Logging Odometry

The companion package **`wf_odometry`** is included to log the robot's estimated path during the run.

* **Topic Subscribed:** `/odom` (`nav_msgs/msg/Odometry`)
* **Function:** Logs timestamped $$(x, y)$$ coordinates to a CSV file.

Odometry values will automatically write to .csv file in the /odom_logs directory.
