# Bluetooth Setup Guide for Telearm

This guide covers setting up Bluetooth communication between the ESP32 operator tracker and Raspberry Pi controller as an alternative to WiFi.

## Overview

The Bluetooth implementation uses ESP32's built-in Bluetooth Classic Serial Port Profile (SPP) to transmit operator pose data. This provides a direct wireless connection without requiring WiFi network setup.

### Architecture
```
ESP32 (Operator) → Bluetooth SPP → Raspberry Pi → USB Serial → Arduino (Robot)
```

### Communication Modes
- **WiFi Mode**: ESP32 → WiFi UDP → Raspberry Pi (existing)
- **Bluetooth Mode**: ESP32 → Bluetooth SPP → Raspberry Pi (new)

## Hardware Requirements

### ESP32 Side
- ESP32-WROOM-32 (has built-in Bluetooth Classic)
- 3× MPU-9250 IMU sensors
- Same wiring as WiFi mode

### Raspberry Pi Side
- Raspberry Pi 4B (has built-in Bluetooth 5.0)
- No additional hardware needed

### Robot Side
- Arduino Uno R3 + servos (unchanged)

## ESP32 Firmware Setup

### Mode Selection

The ESP32 firmware supports three mode selection methods:

#### Option 1: Compile-time Flag (Recommended)
Edit `firmware/esp32/operator_tracker/operator_tracker.ino`:

```cpp
// At the top of the file, uncomment one of these:
#define USE_BLUETOOTH true
// #define USE_WIFI true
```

#### Option 2: GPIO Button Selection
Add a button between GPIO 0 and GND. Press and hold during boot:
- Button pressed → Bluetooth mode
- No button → WiFi mode (default)

#### Option 3: Auto-fallback
Try WiFi first, fall back to Bluetooth if no connection after 10 seconds.

### Bluetooth Implementation

The firmware automatically includes Bluetooth support when `USE_BLUETOOTH` is defined:

```cpp
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setupBluetooth() {
    SerialBT.begin("TelearmOperator");  // Device name for pairing
    Serial.println("Bluetooth device ready for pairing");
}

void transmitOperatorPose() {
    // Same packet structure as WiFi (40 bytes)
    SerialBT.write((uint8_t*)&pose, sizeof(pose));
}
```

### Key Features
- Same 40-byte packet format as WiFi
- Same 100Hz transmission rate
- Same IMU fusion and joint angle estimation
- Connection status monitoring
- Automatic reconnection on disconnect

## Raspberry Pi Setup

### 1. Install Bluetooth Packages

```bash
sudo apt-get update
sudo apt-get install bluez bluez-utils python3-serial
```

### 2. Enable Bluetooth

```bash
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

### 3. Pair with ESP32

Use the automated setup script:

```bash
cd /path/to/robot-arm
chmod +x scripts/setup_bluetooth_pi.sh
./scripts/setup_bluetooth_pi.sh
```

Or manually:

```bash
# Scan for devices
sudo bluetoothctl
> power on
> agent on
> scan on
# Wait for "TelearmOperator" device
> scan off
> pair <ESP32_MAC_ADDRESS>
> trust <ESP32_MAC_ADDRESS>
> connect <ESP32_MAC_ADDRESS>
> quit

# Bind to serial port
sudo rfcomm bind /dev/rfcomm0 <ESP32_MAC_ADDRESS> 1
sudo chmod 666 /dev/rfcomm0
```

### 4. Test Connection

```bash
# Check if device is connected
sudo bluetoothctl info <ESP32_MAC_ADDRESS>

# Test serial port
ls -la /dev/rfcomm0
```

## Configuration

### Teleoperation Config

Update `config/teleop.yaml`:

```yaml
teleoperation:
  mode: "bluetooth"  # or "wifi"
  velocity_scale: 0.6
  update_rate_hz: 100
  timeout_ms: 200
  
  # Bluetooth settings
  bluetooth:
    port: "/dev/rfcomm0"
    baud_rate: 115200
```

### CLI Usage

```bash
# Use Bluetooth mode (overrides config)
python3 -m telearm.cli teleop --mode bluetooth

# Use specific Bluetooth port
python3 -m telearm.cli teleop --mode bluetooth --bt-port /dev/rfcomm0

# Use WiFi mode
python3 -m telearm.cli teleop --mode wifi
```

## Automatic Startup (Optional)

### Systemd Service

To automatically bind Bluetooth on boot:

1. Copy the service file:
```bash
sudo cp scripts/telearm-bluetooth.service /etc/systemd/system/
```

2. Edit the service file to include your ESP32 MAC address:
```bash
sudo nano /etc/systemd/system/telearm-bluetooth.service
# Replace %i with your ESP32 MAC address
```

3. Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable telearm-bluetooth.service
sudo systemctl start telearm-bluetooth.service
```

## Performance Comparison

### Latency
- **WiFi UDP**: 5-15ms typical, <25ms worst case
- **Bluetooth SPP**: 10-20ms typical, <35ms worst case

### Range
- **WiFi**: 30-50m indoor, 100m+ outdoor
- **Bluetooth Classic**: 10-30m indoor, 50-100m outdoor

### Reliability
- **WiFi**: Higher throughput, better range, needs network setup
- **Bluetooth**: Simpler pairing, direct connection, shorter range

### Data Rate
- Both support 100Hz control rate with 40-byte packets
- Bluetooth: 40 bytes × 100Hz = 32,000 bps (1.6% of 2 Mbps capacity)
- WiFi: Same data rate, higher overhead

## Troubleshooting

### ESP32 Issues

#### Bluetooth Not Starting
```cpp
// Check in Serial Monitor:
// "Bluetooth device ready for pairing"
```
- Verify `USE_BLUETOOTH` is defined
- Check ESP32 has Bluetooth enabled in Arduino IDE
- Ensure no conflicting WiFi/Bluetooth libraries

#### Connection Failures
- ESP32 Bluetooth range is limited (~10m)
- Check for 2.4GHz interference (WiFi, microwave, etc.)
- Verify ESP32 is powered adequately

### Raspberry Pi Issues

#### Device Not Found
```bash
# Check Bluetooth is enabled
sudo systemctl status bluetooth

# Scan for devices
sudo bluetoothctl scan on
```

#### Pairing Fails
```bash
# Remove old pairing
sudo bluetoothctl remove <ESP32_MAC>

# Reset Bluetooth
sudo systemctl restart bluetooth
```

#### Serial Port Issues
```bash
# Check rfcomm binding
sudo rfcomm -a

# Release and rebind
sudo rfcomm release /dev/rfcomm0
sudo rfcomm bind /dev/rfcomm0 <ESP32_MAC> 1
```

#### Permission Denied
```bash
# Fix permissions
sudo chmod 666 /dev/rfcomm0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Python Issues

#### Serial Port Not Found
```python
# Check if port exists
import os
print(os.path.exists('/dev/rfcomm0'))

# List available serial ports
import serial.tools.list_ports
print(serial.tools.list_ports.comports())
```

#### Connection Timeout
- Check ESP32 is transmitting data
- Verify baud rate matches (115200)
- Test with simple serial monitor first

## Testing

### 1. ESP32 Firmware Test
```cpp
// In Serial Monitor, look for:
// "Bluetooth device ready for pairing"
// "Sent packet X: angles=[...] vel=[...]"
```

### 2. Raspberry Pi Connection Test
```bash
# Test serial communication
sudo cat /dev/rfcomm0
# Should see binary data if ESP32 is transmitting
```

### 3. Python Integration Test
```python
# Test Bluetooth receiver
from telearm.network.bluetooth_receiver import BluetoothOperatorDataReceiver

receiver = BluetoothOperatorDataReceiver()
receiver.start()
packet = receiver.get_latest()
print(f"Received: {packet}")
receiver.stop()
```

### 4. Full System Test
```bash
# Start teleoperation with Bluetooth
python3 -m telearm.cli teleop --mode bluetooth

# Move ESP32 and verify robot responds
# Check status output for packet reception
```

## When to Use Bluetooth vs WiFi

### Use Bluetooth When:
- No WiFi network available
- Simple point-to-point connection needed
- Shorter range is acceptable (<30m)
- Quick setup required
- Avoiding network configuration

### Use WiFi When:
- Longer range needed (>30m)
- Multiple devices need to connect
- Network infrastructure available
- Higher reliability required
- Integration with existing network

## Security Considerations

### Bluetooth Security
- ESP32 uses default PIN (usually "1234" or "0000")
- Connection is not encrypted by default
- Range limitation provides some physical security
- Consider changing default PIN in ESP32 firmware

### Recommendations
- Use in controlled environment
- Monitor for unauthorized connections
- Consider implementing authentication if needed
- Keep firmware updated

## Advanced Configuration

### Custom Baud Rate
```cpp
// In ESP32 firmware (if needed)
SerialBT.begin("TelearmOperator", true);  // Master mode
```

### Multiple Devices
```bash
# Bind multiple ESP32s to different ports
sudo rfcomm bind /dev/rfcomm0 <ESP32_1_MAC> 1
sudo rfcomm bind /dev/rfcomm1 <ESP32_2_MAC> 1
```

### Debugging
```bash
# Monitor Bluetooth traffic
sudo hcidump -i hci0

# Check Bluetooth logs
journalctl -u bluetooth -f
```

## Support

For issues not covered in this guide:
1. Check ESP32 Serial Monitor for error messages
2. Verify Raspberry Pi Bluetooth logs: `journalctl -u bluetooth`
3. Test with mock data first: `python3 -m telearm.cli teleop --mock`
4. Compare with WiFi mode to isolate Bluetooth-specific issues
