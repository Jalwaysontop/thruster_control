# thruster_control

A ROS2 Python package that provides camera processing, distance estimation, thruster PWM control, and supporting utilities for an underwater robotic system.

---

## 🚀 Features

- **Camera Node**  
  Captures frames, publishes image topics.

- **Pixel Width Subscriber**  
  Listens to detected pixel width and assists in distance estimation.

- **Distance Node**  
  Computes object distance using focal length and pixel width.

- **Operations Node**  
  High-level operational logic and node orchestration.

- **PWM Publisher**  
  Generates PWM values for thruster control.

- **Thruster Control Node**  
  Interfaces with motor drivers and publishes final thruster commands.


## 🛠 Installation

Place the package inside your ROS2 workspace:

```

ros2_ws/
└── src/
└── thruster_control/

````

Then build:

```bash
cd ~/ros2_ws
colcon build --packages-select thruster_control
source install/setup.bash
````

---

## ▶️ Running the Nodes

Each executable is installed as a console script.

```bash
ros2 run my_thruster webcam
ros2 run my_thruster pixel_width_subscriber
ros2 run my_thruster distance
ros2 run my_thruster operations
ros2 run my_thruster pwm_cal
ros2 run my_thruster thruster
```

---

## 📦 Dependencies

Ensure these keys are added to your `package.xml`:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
```

Add more based on your code.

---

## 🧪 Tests

Run linters and tests:

```bash
colcon test
colcon test-result --verbose
```

---

## 🤝 Contributing

Pull requests and improvements are welcome.
Follow ROS2 Python style guides and add tests for new features.

---

## 📜 License

This package is open-sourced under your chosen license (update `LICENSE` file).

```

---

If you'd like, I can also generate:

✅ A more developer-friendly README  
✅ Architecture diagram  
✅ Setup instructions for AUV deployment  
✅ GIFs / flowcharts (ASCII style, since you prefer no videos)

Just tell me!
```
