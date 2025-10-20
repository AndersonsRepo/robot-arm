# Setup Guide

This guide covers the complete setup process for the Telearm robot system with dual-mode WiFi/Bluetooth teleoperation support.

## Prerequisites

- Python 3.8 or higher
- Arduino IDE or compatible programming environment
- USB cable for Arduino programming
- External 5V power supply (6A+ recommended for servos)
- ESP32-WROOM-32 development board
- 3× MPU-9250 IMU sensors
- Raspberry Pi 4B (with WiFi/Bluetooth)
- Arduino Uno R3
- 5× MG995 servos

## Software Installation

### 1. Install Telearm Package

```bash
# Clone the repository
git clone https://github.com/your-org/robot-arm.git
cd robot-arm

# Install in development mode
make install
# or
pip install -e .

# Install with hardware support
pip install -e ".[hardware]"

# Install development dependencies
make dev
```

### 2. Verify Installation

```bash
# Test the package
python -c "import telearm; print(telearm.__version__)"

# Run smoke test
python -m examples.smoke_test

# Test CLI
python -m telearm.cli --help
```

## Hardware Setup

### 1. ESP32 Operator Tracker Assembly

**Required Components:**
- ESP32-WROOM-32 development board
- 3× MPU-9250 IMU sensors
- Breadboard and jumper wires
- 3.3V power supply

**Wiring:**
```
ESP32 Pin    →    MPU-9250 Pin
21 (SDA)     →    SDA
22 (SCL)     →    SCL
3.3V         →    VCC
GND          →    GND
```

**IMU Placement:**
- IMU 0 (0x68): Upper arm
- IMU 1 (0x69): Forearm  
- IMU 2 (0x68): Hand

### 2. Arduino Servo Driver Assembly

**Required Components:**
- Arduino Uno R3
- 5× MG995 servos
- External 5V power supply (6A+)
- Servo mounting hardware

**Wiring:**
```
Arduino Pin  →    Servo Signal
Pin 3        →    Servo 1 (Base)
Pin 5        →    Servo 2 (Shoulder)
Pin 6        →    Servo 3 (Elbow)
Pin 9        →    Servo 4 (Wrist)
Pin 10       →    Servo 5 (Gripper)

Power Supply →    All Servo VCC/GND
```

### 3. Raspberry Pi Setup

**Required Components:**
- Raspberry Pi 4B
- MicroSD card (32GB+)
- Power supply
- USB cable for Arduino connection

**Initial Setup:**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y python3-pip python3-venv git

# Install Bluetooth utilities
sudo apt install -y bluetooth bluez-utils rfcomm
```

## Firmware Setup

### 1. ESP32 Firmware Configuration

**File**: `firmware/esp32/operator_tracker/operator_tracker.ino`

**Mode Selection:**
```cpp
// Choose communication mode (uncomment ONE):
#define USE_BLUETOOTH true
// #define USE_WIFI true
```

**Network Configuration (WiFi Mode):**
```cpp
const char* ssid = "YourWiFiNetwork";        // Replace with your WiFi name
const char* password = "YourWiFiPassword";    // Replace with your WiFi password
const char* targetIP = "192.168.1.100";       // Replace with Raspberry Pi IP
```

**Bluetooth Configuration (Bluetooth Mode):**
```cpp
const char* bluetoothDeviceName = "TelearmOperator";  // Device name for pairing
```

**Upload Instructions:**
1. Open Arduino IDE
2. Install ESP32 board package
3. Select "ESP32 Dev Module" board
4. Open `firmware/esp32/operator_tracker/operator_tracker.ino`
5. Update network settings above
6. Upload to ESP32

### 2. Arduino Firmware Setup

**File**: `firmware/arduino/telearm_driver/telearm_driver.ino`

**Upload Instructions:**
1. Connect Arduino to computer via USB
2. Open Arduino IDE
3. Open `firmware/arduino/telearm_driver/telearm_driver.ino`
4. Select "Arduino Uno" board and correct port
5. Upload firmware

**Verification:**
- Open Serial Monitor (115200 baud)
- Should see "READY" message
- Test with: `M,0,0.0` (should return "OK")

## Configuration Setup

### 1. Update Configuration Files

**File**: `config/teleop.yaml`

**Replace Placeholders:**
```yaml
teleoperation:
  mode: "wifi"  # or "bluetooth"
  velocity_scale: 0.6
  update_rate_hz: 100
  timeout_ms: 200
  
  # WiFi settings (for mode: "wifi")
  port: 5000
  
  # Bluetooth settings (for mode: "bluetooth")
  bluetooth:
    port: "/dev/rfcomm0"  # Bluetooth serial port
    baud_rate: 115200     # Serial baud rate
```

**File**: `config/robot.yaml`

**Update Robot Parameters:**
```yaml
robot:
  name: "telearm-5dof"
  dof: 5

joints:
  - name: "base_yaw"
    home: 0.0
    limit_min: -180.0  # Adjust based on your hardware
    limit_max: 180.0
    max_velocity: 1.0
    max_acceleration: 2.0
  # ... repeat for all 5 joints
```

**File**: `config/pins.yaml`

**Update Hardware Pins:**
```yaml
hardware:
  arduino:
    baud_rate: 115200
    default_port: "/dev/ttyUSB0"  # Update with actual Arduino port

servos:
  pins: [3, 5, 6, 9, 10]  # Update if using different pins
```

### 2. Raspberry Pi Network Configuration

**For WiFi Mode:**
```bash
# Get Raspberry Pi IP address
hostname -I

# Configure static IP (optional)
sudo nano /etc/dhcpcd.conf
# Add:
# interface wlan0
# static ip_address=192.168.1.100/24
# static routers=192.168.1.1
# static domain_name_servers=8.8.8.8

# Restart networking
sudo systemctl restart dhcpcd
```

**For Bluetooth Mode:**
```bash
# Run Bluetooth setup script
chmod +x scripts/setup_bluetooth_pi.sh
sudo ./scripts/setup_bluetooth_pi.sh

# Follow prompts to pair with ESP32
# Script will create /dev/rfcomm0 device
```

## Step-by-Step Setup Process

### Phase 1: Hardware Assembly

1. **Assemble ESP32 Operator Tracker:**
   - Mount ESP32 on breadboard
   - Connect 3× MPU-9250 sensors to ESP32 I2C pins
   - Verify connections with multimeter
   - Power on and check Serial Monitor (115200 baud)

2. **Assemble Arduino Servo Driver:**
   - Mount Arduino Uno
   - Connect 5× servos to specified pins
   - Connect external 5V power supply
   - Verify servo power (should be 5V ±0.1V)
   - Upload Arduino firmware

3. **Setup Raspberry Pi:**
   - Flash Raspberry Pi OS to microSD card
   - Enable SSH and WiFi during first boot
   - Connect to network and update system

### Phase 2: Software Configuration

1. **Install Telearm Package:**
   ```bash
   git clone https://github.com/your-org/robot-arm.git
   cd robot-arm
   pip install -e ".[hardware]"
   ```

2. **Update Configuration Files:**
   - Edit `config/teleop.yaml` with your network settings
   - Edit `config/robot.yaml` with your hardware parameters
   - Edit `config/pins.yaml` with your Arduino pins

3. **Configure ESP32 Firmware:**
   - Edit `firmware/esp32/operator_tracker/operator_tracker.ino`
   - Update WiFi credentials or select Bluetooth mode
   - Upload firmware to ESP32

### Phase 3: Network Setup

**For WiFi Mode:**
1. Ensure ESP32 and Raspberry Pi are on same network
2. Update ESP32 firmware with correct WiFi credentials
3. Update ESP32 firmware with Raspberry Pi IP address
4. Test connection: ESP32 should show "WiFi connected!"

**For Bluetooth Mode:**
1. Run Bluetooth setup script on Raspberry Pi
2. Pair ESP32 with Raspberry Pi
3. Verify `/dev/rfcomm0` device exists
4. Test connection: ESP32 should show "Bluetooth device ready for pairing"

### Phase 4: Testing and Calibration

1. **Basic Hardware Test:**
   ```bash
   # Test Arduino connection
   python -m telearm.cli status
   
   # Test servos
   python -m telearm.cli home
   ```

2. **Network Test:**
   ```bash
   # Test WiFi mode
   python -m telearm.cli teleop --mode wifi
   
   # Test Bluetooth mode
   python -m telearm.cli teleop --mode bluetooth
   ```

3. **Calibration:**
   ```bash
   # Run calibration routine
   python -m telearm.cli calibrate
   
   # Test joint limits
   python -m telearm.cli joints 0.1 0.0 0.0 0.0 0.0
   ```

## Communication Mode Setup

### WiFi Mode Setup

1. **Configure WiFi Network:**
   ```bash
   # On Raspberry Pi, get IP address
   hostname -I
   # Example output: 192.168.1.100
   ```

2. **Update ESP32 Firmware:**
   ```cpp
   const char* ssid = "YourWiFiNetwork";
   const char* password = "YourWiFiPassword";
   const char* targetIP = "192.168.1.100";  // Raspberry Pi IP
   ```

3. **Test Connection:**
   ```bash
   # Start teleoperation
   python -m telearm.cli teleop --mode wifi
   
   # Should see: "UDP receiver started on port 5000"
   # ESP32 Serial Monitor should show: "WiFi connected!"
   ```

### Bluetooth Mode Setup

1. **Run Bluetooth Setup Script:**
   ```bash
   sudo ./scripts/setup_bluetooth_pi.sh
   ```

2. **Follow Script Prompts:**
   - Script will scan for Bluetooth devices
   - Look for "TelearmOperator" device
   - Enter ESP32 MAC address when prompted
   - Script will pair and create `/dev/rfcomm0`

3. **Test Connection:**
   ```bash
   # Start teleoperation
   python -m telearm.cli teleop --mode bluetooth
   
   # Should see: "Bluetooth serial receiver started on /dev/rfcomm0"
   # ESP32 Serial Monitor should show: "Bluetooth device ready for pairing"
   ```

## Systemd Service Setup (Optional)

**For Automatic Bluetooth Binding:**

1. **Edit Service File:**
   ```bash
   sudo nano scripts/telearm-bluetooth.service
   ```

2. **Replace Placeholder:**
   ```ini
   ExecStart=/usr/bin/rfcomm bind 0 <ESP32_MAC_ADDRESS> 1
   ```
   Replace `<ESP32_MAC_ADDRESS>` with actual ESP32 MAC address

3. **Install Service:**
   ```bash
   sudo cp scripts/telearm-bluetooth.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable telearm-bluetooth.service
   sudo systemctl start telearm-bluetooth.service
   ```

## Testing

### 1. Basic Functionality

```bash
# Test in simulation mode
python -m telearm.cli --sim home
python -m telearm.cli --sim status

# Test with hardware
python -m telearm.cli home
python -m telearm.cli status
```

### 2. Communication Testing

```bash
# Test WiFi mode
python -m telearm.cli teleop --mode wifi

# Test Bluetooth mode
python -m telearm.cli teleop --mode bluetooth

# Test with custom Bluetooth port
python -m telearm.cli teleop --mode bluetooth --bt-port /dev/rfcomm1
```

### 3. Full System Test

```bash
# Run comprehensive tests
make test

# Test examples
make examples
```

## Troubleshooting

### Common Issues

1. **ESP32 Connection Issues:**
   - Check WiFi credentials in firmware
   - Verify Raspberry Pi IP address
   - Check Serial Monitor for error messages

2. **Bluetooth Pairing Issues:**
   - Ensure ESP32 is in Bluetooth mode
   - Check if `/dev/rfcomm0` exists
   - Re-run Bluetooth setup script

3. **Arduino Communication Issues:**
   - Check USB cable and port
   - Verify baud rate (115200)
   - Check power supply connections

4. **Servo Issues:**
   - Verify 5V power supply (6A+)
   - Check servo wiring
   - Test individual servos

### Debug Commands

```bash
# Check Python installation
python -c "import telearm; print(telearm.__version__)"

# Test serial communication
python -c "import serial; print('Serial available')"

# Check config files
python -c "from telearm import load_from_config; print(load_from_config())"

# Test CLI without hardware
python -m telearm.cli --sim status

# Check Bluetooth devices
sudo bluetoothctl devices

# Check rfcomm devices
ls -la /dev/rfcomm*
```

### Network Debugging

```bash
# Check WiFi connection
iwconfig

# Test network connectivity
ping <ESP32_IP>

# Monitor UDP traffic
sudo tcpdump -i wlan0 port 5000

# Check Bluetooth status
sudo bluetoothctl info <ESP32_MAC>
```

## Performance Optimization

### WiFi Mode
- **Latency**: 5-15ms typical
- **Range**: 30-50m indoor
- **Setup**: Requires network configuration
- **Use Case**: Longer range, network integration

### Bluetooth Mode
- **Latency**: 10-20ms typical
- **Range**: 10-30m indoor
- **Setup**: Direct pairing, no network required
- **Use Case**: Simple setup, point-to-point connection

## Safety Considerations

1. **Power Supply**: Use 6A+ supply for servos
2. **Emergency Stop**: Always have emergency stop accessible
3. **Joint Limits**: Verify physical limits before operation
4. **Calibration**: Run calibration before first use
5. **Testing**: Start with small movements

## Next Steps

- Review [Safety Guide](safety.md) before operation
- Practice with small movements first
- Experiment with different trajectories
- Consider adding sensors for feedback control
- Review [Bluetooth Setup Guide](bluetooth-setup.md) for detailed Bluetooth configuration
- Review [Program Overview](program-overview.md) for system architecture details