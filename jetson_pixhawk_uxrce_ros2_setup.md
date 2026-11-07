# Jetson Orin Nano ↔ Pixhawk 6C uXRCE-DDS / ROS 2 Bridge Setup

This document summarizes the full process to connect a Jetson Orin Nano and a Pixhawk 6C via PX4 uXRCE-DDS / ROS 2 bridge over UART.

---

## 1. Fix the Jetson Orin Nano UART issue

The Jetson was on **JetPack / L4T R36.5.0**.  
The 40-pin header UART on pins 8/10 maps to:

- `serial@3100000`
- `/dev/ttyTHS1`

There is a known UART DMA / IOMMU issue on some Orin Nano / NX R36.5 systems. To fix this issue, please run the script included in this repository.

### Script used:

```bash
chmod +x jetson_uart_dma_fix.sh
sudo ./jetson_uart_dma_fix.sh apply
sudo reboot
```

### Verify after reboot

```bash
sudo ./jetson_uart_dma_iommu_universal_v3.sh status
```

You want to see something like:

```text
serial-tegra 3100000.serial: Adding to iommu group 1
```

That means the fixed DMA path is active.

---

## 2. Wire Pixhawk TELEM2 to Jetson UART

### Jetson 40-pin header
- pin 8 = TX
- pin 10 = RX
- pin 6 = GND

### Connect to Pixhawk TELEM2
- Pixhawk **TX** → Jetson **RX**
- Pixhawk **RX** → Jetson **TX**
- Pixhawk **GND** → Jetson **GND**

Do **not** connect 5V.

---

## 3. Initial UART sanity test using MAVLink

Before switching to DDS, TELEM2 was first tested as a MAVLink companion link.

### PX4 / QGroundControl parameters for initial MAVLink UART test

```text
MAV_1_CONFIG    = TELEM2
MAV_1_MODE      = Onboard
MAV_1_FLOW_CTRL = Force Off
MAV_1_RADIO_CTL = Disabled
UXRCE_DDS_CFG   = Disabled
SER_TEL2_BAUD   = 57600
```

Then MAVProxy was used over `/dev/ttyTHS1` to confirm the UART link worked.

To test this, you may run:

```text
sudo apt update
sudo apt install python3-pip python3-dev
python3 -m pip install --user MAVProxy
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
mavproxy.py --master=/dev/ttyTHS1,57600 --show-errors
```
---

## 4. Switch TELEM2 from MAVLink to uXRCE-DDS

Once UART was proven healthy, TELEM2 was reassigned to DDS.

### PX4 / QGroundControl parameters

```text
MAV_1_CONFIG   = 0
UXRCE_DDS_CFG  = TELEM2
SER_TEL2_BAUD  = 921600
```

Then reboot the Pixhawk.

---

## 5. Install the Micro XRCE-DDS Agent on the Jetson

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

---

## 6. Create the ROS 2 workspace

### Clone PX4 ROS 2 packages

```bash
mkdir -p ~/ws_px4/src
cd ~/ws_px4/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

### Build

```bash
cd ~/ws_px4
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

---

## 7. Start the DDS bridge after each boot

### Terminal 1: start the agent

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/local_setup.bash
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

### Terminal 2: source ROS and use the bridge

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/local_setup.bash
ros2 topic list
```

---

## 8. Confirm the ROS 2 bridge is alive

A working setup should show many `/fmu/in/...` and `/fmu/out/...` topics, for example:

- `/fmu/in/sensor_optical_flow`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_command_ack`
- `/fmu/out/vehicle_control_mode`
- `/fmu/out/timesync_status`

---

## 9. Use the correct QoS for PX4 ROS 2 topics

PX4 DDS publishers use:

- `BEST_EFFORT`
- `TRANSIENT_LOCAL`

You can verify with:

```bash
ros2 topic info /fmu/out/vehicle_status_v1 -v
ros2 topic info /fmu/out/vehicle_local_position -v
```

### Recommended Python QoS profile

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

Without this, Python subscribers may show incompatible QoS warnings and receive no data.

---

## 10. Bench-test topics that are safe to use

If the Pixhawk is not connected to real sensors, some estimator-driven topics may be silent.

A good first proof-of-life topic is:

```bash
ros2 topic echo /fmu/out/timesync_status --qos-reliability best_effort --qos-durability transient_local
```

Other useful bench-test topics:

```bash
ros2 topic echo /fmu/out/failsafe_flags --qos-reliability best_effort --qos-durability transient_local
ros2 topic echo /fmu/out/vehicle_control_mode --qos-reliability best_effort --qos-durability transient_local
ros2 topic echo /fmu/out/vehicle_land_detected --qos-reliability best_effort --qos-durability transient_local
```

---

## 11. What already works with the current bridge

With the currently exposed topics, you can already:

- publish optical flow using `/fmu/in/sensor_optical_flow`
- publish offboard heartbeat using `/fmu/in/offboard_control_mode`
- publish setpoints using `/fmu/in/trajectory_setpoint`
- send commands using `/fmu/in/vehicle_command`
- read command acknowledgements on `/fmu/out/vehicle_command_ack`
- read local/global position and odometry when available

You do **not** need the `PX4-Autopilot` source repo just to write the Jetson-side optical flow node, because `/fmu/in/sensor_optical_flow` is already exposed.

---

## 12. Daily-use command sequence

### Power-on sequence
1. Power the Jetson
2. Power the Pixhawk

### Terminal 1
```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/local_setup.bash
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

### Terminal 2
```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/local_setup.bash
ros2 topic list
```

### Optional quick test
```bash
ros2 topic echo /fmu/out/timesync_status --qos-reliability best_effort --qos-durability transient_local
```

### Then run your own node
```bash
python3 your_node.py
```

---

## 13. Notes

- `ws_px4` is the **ROS 2 side**.
- `PX4-Autopilot` is the **firmware source repo** and is only needed if you want to edit `dds_topics.yaml` and rebuild firmware.
- If you need a topic not currently exposed by the DDS bridge, then clone `PX4-Autopilot` and edit:

```text
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

---

## 14. Useful reminders

### PX4 side for DDS over TELEM2
```text
MAV_1_CONFIG   = 0
UXRCE_DDS_CFG  = TELEM2
SER_TEL2_BAUD  = 921600
```

### Jetson side
```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/local_setup.bash
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

### Quick bridge check
```bash
ros2 topic list
ros2 topic echo /fmu/out/timesync_status --qos-reliability best_effort --qos-durability transient_local
```
