# WiFi Setup Instructions for Telearm

This guide covers setting up WiFi communication between the ESP32 operator tracker and Raspberry Pi controller for teleoperation.

## Overview

The WiFi implementation uses ESP32's built-in WiFi capabilities to transmit operator pose data via UDP packets. This provides wireless communication with longer range than Bluetooth but requires network configuration.

### Architecture
```
ESP32 (Operator) → WiFi UDP → Raspberry Pi → USB Serial → Arduino (Robot)
```

## Hardware Requirements

### ESP32 Side
- ESP32-WROOM-32 (has built-in WiFi)
- 3× MPU-9250 IMU sensors
- Same wiring as Bluetooth mode

### Raspberry Pi Side
- Raspberry Pi 4B (has built-in WiFi)
- No additional hardware needed

### Robot Side
- Arduino Uno R3 + servos (unchanged)

## Network Configuration Options

### Option 1: Dedicated WiFi Network (Recommended)

Create a dedicated WiFi network for the teleoperation system:

#### Using Raspberry Pi as Access Point
```bash
# Install hostapd and dnsmasq
sudo apt-get update
sudo apt-get install hostapd dnsmasq

# Configure hostapd
sudo nano /etc/hostapd/hostapd.conf
```

Add to `/etc/hostapd/hostapd.conf`:
```ini
interface=wlan0
driver=nl80211
ssid=TelearmNetwork
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=telearm123
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

Configure dnsmasq:
```bash
sudo nano /etc/dnsmasq.conf
```

Add to `/etc/dnsmasq.conf`:
```ini
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
```

Configure static IP for Pi:
```bash
sudo nano /etc/dhcpcd.conf
```

Add to `/etc/dhcpcd.conf`:
```ini
interface wlan0
static ip_address=192.168.4.1/24
nohook wpa_supplicant
```

Enable services:
```bash
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo systemctl start hostapd
sudo systemctl start dnsmasq
```

#### ESP32 Configuration
Update `firmware/esp32/operator_tracker/operator_tracker.ino`:
```cpp
const char* ssid = "TelearmNetwork";
const char* password = "telearm123";
const char* targetIP = "192.168.4.1";  // Pi's static IP
```

### Option 2: Existing WiFi Network

Connect both devices to an existing WiFi network:

#### Raspberry Pi Setup
```bash
# Connect to WiFi
sudo raspi-config
# Navigate to: System Options → Wireless LAN
# Enter SSID and password

# Get Pi's IP address
hostname -I
```

#### ESP32 Configuration
Update firmware with network details:
```cpp
const char* ssid = "YourWiFiNetwork";
const char* password = "YourWiFiPassword";
const char* targetIP = "192.168.1.100";  // Pi's IP from hostname -I
```

## ESP32 Firmware Configuration

### WiFi Mode Selection
In `firmware/esp32/operator_tracker/operator_tracker.ino`:

```cpp
// Mode selection - uncomment ONE of these:
// #define USE_BLUETOOTH true
#define USE_WIFI true
```

### Network Settings
Update these constants in the firmware:
```cpp
const char* ssid = "TelearmNetwork";        // WiFi network name
const char* password = "telearm123";         // WiFi password
const int udpPort = 5000;                    // UDP port (must match Pi)
const char* targetIP = "192.168.4.1";       // Raspberry Pi IP address
```

### Compile and Upload
1. Open Arduino IDE
2. Select ESP32 board
3. Install required libraries:
   - MPU9250 (for IMU sensors)
   - WiFi (built-in)
4. Compile and upload to ESP32

## Raspberry Pi Configuration

### Python Dependencies
```bash
# Install required packages
pip install numpy scipy pyyaml pyserial

# For teleoperation
pip install -e ".[hardware]"
```

### Network Configuration
Ensure the Pi has a static IP or consistent IP assignment:

#### Static IP Setup
```bash
sudo nano /etc/dhcpcd.conf
```

Add:
```ini
interface wlan0
static ip_address=192.168.4.1/24
static routers=192.168.4.1
static domain_name_servers=8.8.8.8
```

Restart networking:
```bash
sudo systemctl restart dhcpcd
```

### Firewall Configuration
```bash
# Allow UDP port 5000
sudo ufw allow 5000/udp

# Check firewall status
sudo ufw status
```

## Testing WiFi Connection

### 1. ESP32 Serial Monitor
Open Arduino IDE Serial Monitor (115200 baud) and look for:
```
ESP32 Operator Tracker Starting...
Mode: WiFi UDP
Connecting to WiFi: TelearmNetwork
........
WiFi connected!
IP address: 192.168.4.2
Operator tracker ready!
```

### 2. Raspberry Pi Network Test
```bash
# Check WiFi connection
iwconfig

# Test connectivity to ESP32
ping 192.168.4.2

# Monitor UDP traffic
sudo tcpdump -i wlan0 port 5000
```

### 3. Python Integration Test
```bash
# Test WiFi teleoperation
python3 -m telearm.cli teleop --mode wifi

# Should see:
# Starting teleoperation mode...
# Mode override: wifi
# UDP receiver started on port 5000
# Control loop started at 100 Hz
```

## Troubleshooting WiFi Issues

### ESP32 Connection Problems

#### WiFi Not Connecting
```cpp
// Check in Serial Monitor for error messages
// Common issues:
// - Wrong SSID/password
// - WiFi network not in range
// - Network security settings
```

#### Connection Drops
- Check signal strength: Move ESP32 closer to router
- Verify network stability: Test with other devices
- Check for interference: 2.4GHz devices (microwave, etc.)

### Raspberry Pi Network Issues

#### Cannot Receive UDP Packets
```bash
# Check if Pi is listening on port 5000
sudo netstat -ulnp | grep 5000

# Check firewall rules
sudo ufw status

# Test UDP reception
nc -ul 5000
```

#### IP Address Conflicts
```bash
# Check current IP
hostname -I

# Release and renew IP
sudo dhcpcd -k
sudo dhcpcd
```

### Network Performance Issues

#### High Latency
- Check WiFi signal strength
- Reduce network congestion
- Use dedicated 2.4GHz channel
- Consider 5GHz if supported

#### Packet Loss
- Monitor with: `sudo tcpdump -i wlan0 port 5000`
- Check for interference
- Verify ESP32 and Pi are on same network

## Performance Optimization

### WiFi Settings
```cpp
// In ESP32 firmware, optimize WiFi settings:
WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Maximum power
WiFi.setSleep(false);                 // Disable power saving
```

### Network Optimization
```bash
# On Raspberry Pi, optimize network settings:
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### UDP Buffer Sizes
```python
# In Python receiver, increase buffer sizes:
receiver = OperatorDataReceiver(
    port=5000,
    buffer_size=20,  # Increase from default 10
    timeout_seconds=0.2
)
```

## Security Considerations

### Network Security
- Use WPA2/WPA3 encryption
- Change default passwords
- Consider MAC address filtering
- Use dedicated network for teleoperation

### Access Control
```bash
# Restrict network access
sudo ufw deny from 192.168.4.0/24 to any port 22  # SSH
sudo ufw allow from 192.168.4.0/24 to any port 5000  # Teleoperation
```

## Advanced Configuration

### Multiple ESP32s
To support multiple operator devices:

```cpp
// Each ESP32 needs unique identifier
const char* deviceID = "Operator1";  // or "Operator2", etc.
```

```python
# Python can handle multiple UDP sources
# Each ESP32 sends to same port, Python receives from all
```

### Network Monitoring
```bash
# Monitor network performance
sudo iftop -i wlan0

# Check WiFi signal quality
sudo iwconfig wlan0

# Monitor UDP traffic
sudo tcpdump -i wlan0 -n port 5000
```

## When to Use WiFi vs Bluetooth

### Use WiFi When:
- Longer range needed (>30m)
- Multiple devices need to connect
- Network infrastructure available
- Higher reliability required
- Integration with existing network

### Use Bluetooth When:
- No WiFi network available
- Simple point-to-point connection needed
- Shorter range is acceptable (<30m)
- Quick setup required
- Avoiding network configuration

## Support and Troubleshooting

### Common Error Messages

#### "WiFi connection failed!"
- Check SSID and password
- Verify network is in range
- Check network security settings

#### "UDP receiver started on port 5000"
- Normal startup message
- If followed by connection timeout, check network

#### "No packets received"
- ESP32 not transmitting
- Network connectivity issues
- Firewall blocking UDP packets

### Debug Commands
```bash
# Check WiFi status
iwconfig

# Test network connectivity
ping <ESP32_IP>

# Monitor UDP traffic
sudo tcpdump -i wlan0 port 5000

# Check Python process
ps aux | grep telearm
```

### Getting Help
1. Check ESP32 Serial Monitor for error messages
2. Verify network connectivity with ping
3. Test with mock data: `python3 -m telearm.cli teleop --mock`
4. Compare with Bluetooth mode to isolate WiFi-specific issues
