# WiFi Implementation Knowledge & Bluetooth Adaptation Guide

## WiFi Implementation Learnings

### Network Architecture Decisions

#### Why WiFi Over Bluetooth?
**Advantages**:
- **Lower Latency**: WiFi UDP can achieve <10ms latency vs Bluetooth's 20-50ms
- **Higher Throughput**: WiFi supports higher data rates for future expansion
- **Network Flexibility**: Can connect to existing network infrastructure
- **Multiple Devices**: Support for multiple operators or monitoring devices
- **Debugging**: Easier network debugging and monitoring tools

**Trade-offs**:
- **Power Consumption**: WiFi uses more power than Bluetooth
- **Setup Complexity**: Requires network configuration vs Bluetooth pairing
- **Interference**: WiFi can be affected by network congestion

#### Network Configuration
**ESP32 WiFi Settings**:
```cpp
const char* ssid = "TelearmNetwork";
const char* password = "telearm123";
const char* targetIP = "192.168.1.100";  // Raspberry Pi IP
const int udpPort = 5000;
```

**Key Learnings**:
1. **Static IP Assignment**: Raspberry Pi needs static IP for reliable connection
2. **Network Isolation**: Dedicated WiFi network prevents interference
3. **UDP vs TCP**: UDP chosen for lower latency, accepting occasional packet loss
4. **Port Selection**: Port 5000+ avoids system port conflicts

### Data Transmission Strategy

#### Packet Format Optimization
**40-byte Fixed Packet**:
```
| seq (4) | timestamp (4) | joints (12) | velocities (12) | confidence (4) | checksum (4) |
```

**Design Decisions**:
- **Fixed Size**: Predictable network behavior, easier buffering
- **Sequence Numbers**: Detect packet loss and reordering
- **Timestamp**: Measure network latency
- **Checksum**: Detect corruption (simple sum, fast to compute)
- **Minimal Data**: Only essential information to reduce latency

#### Transmission Timing
**ESP32 Firmware**:
- **IMU Fusion**: 200 Hz (5ms) for accurate orientation
- **UDP Transmission**: 100 Hz (10ms) to balance accuracy vs network load
- **Buffering**: 10-packet buffer to handle network jitter

**Performance Results**:
- **Typical Latency**: 5-15ms over local WiFi
- **Packet Loss**: <1% on stable network
- **Jitter**: ±2ms variation

### Network Reliability Features

#### Connection Monitoring
**Python Receiver**:
```python
def is_connection_alive(self) -> bool:
    return (time.time() - self.stats.last_packet_time) < self.timeout_seconds
```

**Features Implemented**:
- **Watchdog Timer**: 200ms timeout triggers emergency stop
- **Packet Loss Detection**: Sequence number monitoring
- **Latency Tracking**: Real-time performance measurement
- **Automatic Recovery**: Reconnection without restart

#### Error Handling
**Network Error Types**:
1. **Connection Loss**: WiFi disconnection
2. **Packet Corruption**: Checksum failures
3. **High Latency**: Network congestion
4. **Packet Loss**: Sequence gaps

**Recovery Strategies**:
- **Graceful Degradation**: Continue with last known good data
- **Emergency Stop**: Safety shutdown on critical failures
- **Status Reporting**: Real-time error notifications

## Bluetooth Adaptation Requirements

### Hardware Changes

#### ESP32 Bluetooth Configuration
**Current WiFi Code**:
```cpp
WiFi.begin(ssid, password);
WiFi.localIP();
udp.beginPacket(targetIP, udpPort);
```

**Required Bluetooth Changes**:
```cpp
// Replace WiFi with Bluetooth Classic
BluetoothSerial SerialBT;
SerialBT.begin("TelearmOperator");
SerialBT.connect("TelearmController");
```

#### Connection Management
**WiFi Approach**:
- Automatic IP assignment
- UDP socket connection
- Network discovery via IP

**Bluetooth Approach**:
- Device pairing required
- Serial connection management
- Device discovery via Bluetooth

### Software Architecture Changes

#### Network Layer Modifications
**File**: `telearm/network/receiver.py`

**Current WiFi Implementation**:
```python
class OperatorDataReceiver:
    def __init__(self, port: int = 5000, buffer_size: int = 10):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', port))
```

**Required Bluetooth Changes**:
```python
import bluetooth

class OperatorDataReceiver:
    def __init__(self, device_mac: str, buffer_size: int = 10):
        self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.socket.connect((device_mac, 1))  # RFCOMM port 1
```

#### Protocol Adaptations
**Current UDP Protocol**:
- Connectionless communication
- Packet-based transmission
- Built-in error detection

**Bluetooth Protocol**:
- Connection-oriented communication
- Stream-based transmission
- Need explicit packet framing

**Required Changes**:
```python
# Add packet framing for Bluetooth stream
def send_packet_bluetooth(self, packet: TeleopPacket):
    data = packet.pack()
    # Add packet length header
    length_header = struct.pack('>I', len(data))
    self.socket.send(length_header + data)

def receive_packet_bluetooth(self) -> Optional[TeleopPacket]:
    # Read packet length
    length_data = self.socket.recv(4)
    length = struct.unpack('>I', length_data)[0]
    
    # Read packet data
    data = self.socket.recv(length)
    return TeleopPacket.unpack(data)
```

### Configuration Changes

#### Network Configuration
**WiFi Config** (`config/teleop.yaml`):
```yaml
teleoperation:
  network:
    type: "wifi"
    port: 5000
    timeout_ms: 200
```

**Bluetooth Config** (Required):
```yaml
teleoperation:
  network:
    type: "bluetooth"
    device_mac: "AA:BB:CC:DD:EE:FF"  # ESP32 MAC address
    timeout_ms: 200
```

#### ESP32 Firmware Changes
**WiFi Firmware** (`firmware/esp32/operator_tracker/operator_tracker.ino`):
```cpp
void connectToWiFi() {
    WiFi.begin(ssid, password);
    // ... WiFi connection logic
}

void transmitOperatorPose() {
    udp.beginPacket(targetIP, udpPort);
    udp.write((uint8_t*)&pose, sizeof(pose));
    udp.endPacket();
}
```

**Required Bluetooth Firmware**:
```cpp
void connectToBluetooth() {
    SerialBT.begin("TelearmOperator");
    // Wait for connection
    while (!SerialBT.connected()) {
        delay(100);
    }
}

void transmitOperatorPose() {
    // Send packet length header
    uint32_t packetSize = sizeof(pose);
    SerialBT.write((uint8_t*)&packetSize, 4);
    
    // Send packet data
    SerialBT.write((uint8_t*)&pose, sizeof(pose));
}
```

### Performance Considerations

#### Latency Comparison
**WiFi Performance**:
- **Typical Latency**: 5-15ms
- **Jitter**: ±2ms
- **Reliability**: High on stable network

**Bluetooth Performance** (Expected):
- **Typical Latency**: 20-50ms
- **Jitter**: ±10ms
- **Reliability**: Good for short range

#### Bandwidth Requirements
**Current Data Rate**:
- **Packet Size**: 40 bytes
- **Transmission Rate**: 100 Hz
- **Total Bandwidth**: 4 KB/s

**Bluetooth Suitability**:
- **Bluetooth Classic**: 1 Mbps (more than sufficient)
- **Bluetooth LE**: 2 Mbps (sufficient but higher latency)

### Implementation Steps for Bluetooth

#### Phase 1: Basic Bluetooth Support
1. **Replace WiFi with Bluetooth Classic** in ESP32 firmware
2. **Update Python receiver** to use Bluetooth socket
3. **Add packet framing** for stream-based communication
4. **Update configuration** for Bluetooth parameters

#### Phase 2: Connection Management
1. **Implement device discovery** and pairing
2. **Add connection monitoring** and auto-reconnect
3. **Handle Bluetooth-specific errors** (pairing loss, range issues)
4. **Update watchdog logic** for Bluetooth timeouts

#### Phase 3: Performance Optimization
1. **Optimize packet format** for Bluetooth characteristics
2. **Implement adaptive transmission rate** based on connection quality
3. **Add Bluetooth-specific debugging** tools
4. **Performance testing** and latency measurement

### Code Changes Required

#### Files to Modify
1. **`firmware/esp32/operator_tracker/operator_tracker.ino`**:
   - Replace WiFi library with BluetoothSerial
   - Update connection and transmission functions
   - Add packet framing for stream communication

2. **`telearm/network/receiver.py`**:
   - Replace UDP socket with Bluetooth socket
   - Add packet framing for stream-based communication
   - Update connection monitoring for Bluetooth

3. **`config/teleop.yaml`**:
   - Add Bluetooth configuration section
   - Update network parameters

4. **`telearm/cli.py`**:
   - Add Bluetooth connection options
   - Update help text and examples

#### New Dependencies
**Python**:
```toml
[project.optional-dependencies]
teleop = ["pyserial>=3.5", "scipy>=1.7.0", "pybluez>=0.23"]  # Add pybluez
```

**ESP32**:
```cpp
#include <BluetoothSerial.h>  // Replace WiFi.h
```

### Testing Strategy for Bluetooth

#### Development Testing
1. **Mock Bluetooth Connection**: Test packet framing without hardware
2. **Bluetooth Simulator**: Use Android/iOS app to simulate ESP32
3. **Range Testing**: Verify connection stability at different distances
4. **Interference Testing**: Test with other Bluetooth devices

#### Performance Testing
1. **Latency Measurement**: Compare WiFi vs Bluetooth performance
2. **Reliability Testing**: Long-term connection stability
3. **Battery Life**: Power consumption comparison
4. **Pairing Testing**: Connection establishment and reconnection

### Migration Path

#### Backward Compatibility
- **Dual Support**: Support both WiFi and Bluetooth modes
- **Configuration Switch**: Easy switching between modes
- **Fallback Options**: Automatic fallback on connection failure

#### Deployment Strategy
1. **Phase 1**: Implement Bluetooth alongside WiFi
2. **Phase 2**: Test and validate Bluetooth performance
3. **Phase 3**: Deprecate WiFi in favor of Bluetooth (if desired)
4. **Phase 4**: Remove WiFi code (optional)

This knowledge base provides a comprehensive guide for adapting the teleoperation system from WiFi to Bluetooth while maintaining the same safety features and real-time performance characteristics.
