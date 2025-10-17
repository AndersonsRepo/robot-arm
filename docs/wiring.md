# Wiring Guide

This document describes how to wire the 5-DOF robot arm servos to the Arduino.

## Hardware Requirements

- Arduino Uno or compatible microcontroller
- 5x SG90 or similar hobby servos
- External 5V power supply (6A+ recommended)
- Jumper wires
- Breadboard (optional, for prototyping)

## Servo Connections

Connect the servos to the Arduino pins as specified in `config/pins.yaml`:

| Joint | Servo Function | Arduino Pin | Color Code |
|-------|----------------|-------------|------------|
| Base Yaw | Joint 1 | Pin 3 | Orange (Signal) |
| Base Pitch | Joint 2 | Pin 5 | Orange (Signal) |
| Elbow | Joint 3 | Pin 6 | Orange (Signal) |
| Wrist Pitch | Joint 4 | Pin 9 | Orange (Signal) |
| Wrist Roll | Joint 5 | Pin 10 | Orange (Signal) |

### Servo Wire Colors (SG90)
- **Red**: Power (5V)
- **Black/Brown**: Ground
- **Orange/Yellow**: Signal

## Power Distribution

**Important**: Do not power the servos directly from the Arduino's 5V pin. Use an external power supply.

### Recommended Setup:
1. Connect external 5V power supply positive to servo power rails
2. Connect external 5V power supply negative to Arduino GND
3. Connect Arduino GND to servo ground rails
4. Connect only servo signal wires to Arduino pins

### Power Supply Requirements:
- **Voltage**: 5V (4.8V - 6V acceptable)
- **Current**: 6A or higher (servos can draw 1A+ each under load)
- **Connector**: Barrel jack or screw terminals

## Wiring Diagram

```
External 5V Supply
    │
    ├─ 5V ────┐
    │         │
    └─ GND ───┼─── Arduino GND
              │
              ├─ Servo 1 (Pin 3) ── Signal
              ├─ Servo 2 (Pin 5) ── Signal  
              ├─ Servo 3 (Pin 6) ── Signal
              ├─ Servo 4 (Pin 9) ── Signal
              └─ Servo 5 (Pin 10) ── Signal
```

## Mechanical Assembly

1. **Base Yaw (Joint 1)**: Mount to base plate, rotates entire arm
2. **Base Pitch (Joint 2)**: Mounts to yaw joint, controls arm elevation
3. **Elbow (Joint 3)**: Mid-arm joint for reach extension
4. **Wrist Pitch (Joint 4)**: Controls end-effector angle
5. **Wrist Roll (Joint 5)**: Controls end-effector rotation

## Calibration Notes

- Each servo may have slightly different center positions
- Use the Arduino serial monitor to test individual servo movements
- Adjust `theta_offset` values in `config/robot.yaml` if needed
- Verify joint limits match physical constraints

## Troubleshooting

### Common Issues:

1. **Servos not moving**: Check power connections and signal wires
2. **Jittery movement**: Ensure adequate power supply capacity
3. **Incorrect positioning**: Verify pin assignments and servo mounting
4. **Serial communication errors**: Check baud rate (115200) and port

### Testing Individual Servos:
```bash
# Test servo on pin 3
telearm --sim joints 0 0 0 0 0  # All at center
```

### Power Supply Issues:
- Use multimeter to verify 5V output under load
- Check for voltage drop when servos move simultaneously
- Consider using capacitors for power smoothing

## Safety Notes

- Always power off when making connections
- Double-check polarity before connecting power
- Use appropriate wire gauge for current capacity
- Secure all connections to prevent shorts
- Keep power supply away from moisture and heat
