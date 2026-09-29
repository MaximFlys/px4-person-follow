# px4-person-follow

I wrote `flight_control_node` to close a vision loop on a PX4 quadcopter. An OpenCV image pipeline runs a YOLO11n model on the drone camera. My node subscribes to those detections and publishes the velocity setpoints PX4 flies.

The program I wrote is `src/flight_control_node/flight_control_node/flight_node.py`. The console script is `start_flight`.

## Data pipeline

Gazebo does not talk to my node, and my node does not talk to Gazebo. Each stage has one job:

```
x500 mono camera in Gazebo Harmonic
    → ros_gz_image image_bridge          sensor_msgs/Image
    → OpenCV (cv_bridge, BGR cv::Mat)
    → YOLO11n ONNX                       ros2_yolos_cpp /yolos_detector
    → /yolos_detector/detections         vision_msgs/Detection2DArray
    → flight_control_loop()              the code in this repo
    → /fmu/in/*                          Micro XRCE-DDS Agent, UDP 8888
    → PX4 SITL                           moves the simulated x500
```

`/fmu/out/vehicle_attitude` comes back the other way so the forward speed can be rotated into the local NED frame.

## OpenCV and the YOLO model

The model is YOLO11n, exported to ONNX (`yolo11n.onnx`) with COCO names in `coco.names`. Class 0 in that file is `person`. I run it with [ros2_yolos_cpp](https://github.com/Geekgineer/ros2_yolos_cpp), which is an OpenCV 4 detector node:

1. `ros_gz_image` turns the Gazebo camera into a `sensor_msgs/Image` on `/world/baylands/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`.
2. `cv_bridge` converts that message to an OpenCV BGR `cv::Mat`. That is the image the model sees.
3. YOLOs-CPP runs the ONNX model on that `cv::Mat` (ONNX Runtime, CPU in my sim, `use_gpu:=false`). OpenCV also draws the debug image.
4. Boxes are converted to `vision_msgs/Detection2DArray` and published on `/yolos_detector/detections`.

Each `Detection2D` carries the fields my loop reads:

- `bbox.center.position.x` and `bbox.center.position.y`, pixel center of the box
- `bbox.size_y`, box height in pixels
- `results[0].hypothesis.class_id`, the COCO name (`person`) or the numeric id if the name is empty

The detector is a lifecycle node. `configure` loads `yolo11n.onnx` and `coco.names` into memory. `activate` opens the image subscription and starts publishing. My flight node has nothing to follow until that transition finishes. The launch file remaps `~/image_raw` to the Gazebo camera topic, so the model is looking at the x500's forward camera and not at `/camera/image_raw`.

## What I wrote

`FlightSubscriber` is a `rclpy` node named `flight_node_subscriber`. A 10 Hz timer is the only thing that talks to PX4. The camera callback only updates state.

### Subscriptions and publishers

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| Subscribe | `/yolos_detector/detections` | `vision_msgs/Detection2DArray` | depth 10 |
| Subscribe | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | sensor data |
| Publish | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | depth 10 |
| Publish | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | depth 10 |
| Publish | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | depth 10 |

`listener_callback` logs how many boxes arrived and calls `flight_control_loop`. `attitude_callback` stores `msg.q` and calls `Qto3D`.

### `Qto3D`

PX4's `VehicleAttitude.q` is scalar-first `(w, x, y, z)`. I expand that quaternion into a rotation matrix and read the Euler angles:

- pitch = `arcsin(-R[2, 0])`
- roll = `arctan2(R[2, 1], R[2, 2])`
- yaw = `arctan2(R[1, 0], R[0, 0])`

Yaw is the value the velocity rotation uses. Roll and pitch are computed and stored with it.

### `flight_control_loop`

This is the connection from the YOLO message into the setpoint.

An empty `detections` array zeros `x`, `y`, `z`, and `yaw_rate` immediately. Otherwise I take `detections[0]`, the first box in the array. If that box has no `results`, I log a warning and return without touching the watchdog, so the last command stands until the one-second timer clears it.

The class check is `class_id == 'person' or class_id == '0'`. That matches a COCO name from `coco.names` and the raw class index. Any other class is ignored.

For a person I read the box and treat the image as 640×480, so the center is `(320, 240)`:

- `error_x = center_x - 320`
- `error_z = center_y - 240` (the variable is named `center_z` in the source)
- `bbox_height = bbox.size_y`

`error_x` becomes a yaw rate, `error_x * 0.01`, instead of a sideways velocity. The vehicle turns toward the person.

Forward speed is proportional to how small the box is. A height under 120 pixels means the person is far, so

```
velocity_forward = (120 - bbox_height) * 0.05
```

At 120 pixels or taller the forward speed is 0. I then rotate that body-forward speed into the PX4 local NED frame with the yaw from `Qto3D`:

```
x = velocity_forward * cos(yaw)    # North
y = velocity_forward * sin(yaw)    # East
```

The vertical command published from this loop is `velocity_up`. That field is initialized to 0 and the loop does not assign `error_z` into it, so the setpoint `z` stays 0. The vehicle yaws and moves forward. It does not climb or descend from the box position.

Every accepted person box refreshes `last_detection_time`.

### `timer_callback`

Ten hertz, in this order:

1. When `offboard_setpoint_counter_` is 10, send offboard mode and arm. The timer fires every 0.1 s and the counter increments once per tick, so this is about one second after the node starts. The setpoint stream is already running, which is what PX4 requires before it will enter offboard.
2. If `last_detection_time` is more than 1.0 s old, zero `x`, `y`, `z`, and `yaw_rate`. A stopped detector, or a first box that is not a person, therefore decays to a zero velocity instead of holding the last chase command.
3. Publish `OffboardControlMode` and `TrajectorySetpoint`.
4. Increment the counter until it reaches 11, so the arm command is sent once.

`land()` is also in the file. It publishes `VEHICLE_CMD_NAV_LAND`. The timer does not call it. The loss-of-detection response I wired up is the zero-velocity watchdog above.

### Setpoint and command messages

`publish_offboard_control_mode` sets `velocity` true and position, acceleration, attitude, and body rate false. PX4 stays in offboard only while this message keeps arriving.

`publish_trajectory_setpoint` fills position, acceleration, jerk, and yaw with NaN so PX4 ignores them. `velocity` is `[x, y, z]` in local NED (z positive down). `yawspeed` is `yaw_rate`. The timestamp is the node clock in microseconds.

`publish_vehicle_command` fills a `VehicleCommand` from companion component 191 (`from_external` true, system 1, component 1):

- `VEHICLE_CMD_DO_SET_MODE` with `param1 = 1`, `param2 = 6` selects PX4 custom mode offboard.
- `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1 = 1` arms.
- `VEHICLE_CMD_NAV_LAND` is what `land()` sends.

`main` creates the node, spins until Ctrl-C, then destroys the node and shuts rclpy down.

### Package around the node

`package.xml` depends on `rclpy`, `vision_msgs`, `geometry_msgs`, `px4_msgs`, and `python3-numpy`. The build type is `ament_python`. `setup.py` installs the entry point `start_flight = flight_control_node.flight_node:main`. The `test/` files are the standard ament copyright, flake8, and pep257 checks created with the package.

`px4_msgs` in this workspace is [PX4/px4_msgs](https://github.com/PX4/px4_msgs) at `ca9895d` (package 2.0.1, aligned with PX4 `07bac138`). The message package has to match the firmware.

## Build

ROS 2 Jazzy, with `vision_msgs` and `python3-numpy` installed:

```bash
git clone https://github.com/MaximFlys/DFC.git
cd DFC
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
git -C src/px4_msgs checkout ca9895d26b88ffc14209daf2fc8d8564e51adddd

source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If `src/px4_msgs` is already an empty directory from the committed gitlink, remove that empty directory and run the `px4_msgs` clone again.

Colcon finds packages by walking `src/` and reading each `package.xml`. `build/`, `install/`, and `log/` are generated and gitignored.

## Run

The detector has to be publishing `/yolos_detector/detections`, and the uXRCE-DDS link has to be up.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run flight_control_node start_flight
```

The node commands offboard and arms about one second later. I run it in the simulator below.

## Gazebo Harmonic

I tested this in PX4 SITL against Gazebo Harmonic (`gz-sim` 8.14), airframe `x500_mono_cam`, world `baylands`. The PX4 tree is `~/src/PX4-Autopilot` at `v1.18.0-alpha1-113-g1403709f65`.

`./scripts/run_gazebo_sim.sh` opens the seven windows I used to paste by hand, and it waits on the slow steps. The image bridge, camera view, and YOLO launch wait until PX4 prints `Ready for takeoff`. The flight window waits until window 6 has configured and activated `/yolos_detector`.

```bash
./scripts/run_gazebo_sim.sh
```

| Window | What it runs |
| --- | --- |
| 1 PX4 Gazebo | `PX4_GZ_WORLD=baylands` and `QT_QPA_PLATFORM=xcb make px4_sitl gz_x500_mono_cam` in `~/src/PX4-Autopilot` |
| 2 Image bridge | `ros_gz_image image_bridge` on the x500 mono camera topic |
| 3 uXRCE agent | `MicroXRCEAgent udp4 -p 8888` |
| 4 Camera view | `rqt_image_view` on that same camera topic |
| 5 YOLO | `ros2 launch ros2_yolos_cpp detector.launch.py` with `~/ros2_ws/models/yolo11n.onnx` and `coco.names` |
| 6 Detector lifecycle | `ros2 lifecycle set /yolos_detector configure`, then `activate` |
| 7 Flight node | `source /opt/ros/jazzy/setup.bash`, then `source ~/super_ws/install/setup.bash`, then `ros2 run flight_control_node start_flight` |

Close the PX4 window to stop the simulator. If PX4 SITL or `MicroXRCEAgent` is already running, the script exits instead of starting a second copy.

## License

Apache-2.0. Maintainer: Maxim ([MaximFlys](https://github.com/MaximFlys)).
