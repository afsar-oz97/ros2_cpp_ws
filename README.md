
---

# ROS 2 C++ Workspace — `ros2_cpp_ws`

This workspace contains multiple ROS 2 packages implemented in C++ for learning, experimenting, and demonstrating core ROS 2 concepts including publishers/subscribers, services, actions, and OpenCV integration.

Tested on **ROS 2 Humble**.

---

## 📂 Workspace Structure

```
ros2_cpp_ws/
├── src/
│   ├── calculate_robot_speed/
│   ├── robot_navigation/
│   └── udemy_ros2_pkg/
```

---

## 📦 Packages Overview

### 1️⃣ `calculate_robot_speed`

Implements publishers and services related to robot speed calculations.

**Nodes:**

* `rpm_publisher` — Publishes simulated RPM values.
* `speed_publisher` — Publishes calculated robot speed.
* `service_server` — Provides odd/even number checking service.
* `client_server` — Client for the odd/even check service.
* `image_angle_change_service_server` — Service server that processes an image to change its angle using OpenCV.
* `image_angle_change_client_server` — Client for the above image angle service.

**Dependencies:**
`rclcpp`, `std_msgs`, `sensor_msgs`, `cv_bridge`, `OpenCV`

---

### 2️⃣ `robot_navigation`

Implements ROS 2 Actions for navigation tasks.

**Nodes:**

* `navigation_action_server` — Receives navigation goals and executes them.
* `navigation_action_client` — Sends navigation goals to the server.

**Dependencies:**
`rclcpp`, `rclcpp_action`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `action_msgs`

---

### 3️⃣ `udemy_ros2_pkg`

Minimal publisher/subscriber demo (string messages).

**Nodes:**

* `publisher` — Publishes a `std_msgs/String` at a fixed rate.
* `subscriber` — Subscribes to the string topic and logs messages.

**Dependencies:**
`rclcpp`, `std_msgs`

---

## 📦 Common Dependencies

All packages use:

* `rclcpp` — ROS 2 C++ Client Library
* `std_msgs` — Standard ROS 2 message types
* ROS 2 build tools: `ament_cmake`, `rosidl_default_generators`

---

## 🛠️ Building the Workspace

```bash
# From workspace root
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Running Nodes

**Example:**

```bash
# In one terminal — start a publisher
ros2 run udemy_ros2_pkg publisher

# In another terminal — start a subscriber
ros2 run udemy_ros2_pkg subscriber
```

**Calculate Robot Speed:**

```bash
ros2 run calculate_robot_speed rpm_publisher
ros2 run calculate_robot_speed speed_publisher
```

**Navigation Actions:**

```bash
# Start server
ros2 run robot_navigation navigation_action_server

# Start client
ros2 run robot_navigation navigation_action_client
```

**Services:**

```bash
# Start odd/even check server
ros2 run calculate_robot_speed service_server

# Call service from client
ros2 run calculate_robot_speed client_server
```

---

## 📚 Notes

* Ensure you have `OpenCV` installed for the `calculate_robot_speed` image processing nodes.
* Built and tested with **ROS 2 Humble** on Ubuntu 22.04.
* All nodes are implemented in modern C++ using `rclcpp`.

---

