#!/bin/bash
# Raspberry Pi Bluetooth Setup Script for Telearm
# 
# This script automates the pairing process between Raspberry Pi and ESP32
# Bluetooth device for teleoperation communication.
#
# Usage: ./setup_bluetooth_pi.sh [ESP32_MAC_ADDRESS]
#
# If MAC address is not provided, the script will scan for devices.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
ESP32_MAC=""
DEVICE_NAME="TelearmOperator"
SERIAL_PORT="/dev/rfcomm0"

echo -e "${BLUE}=== Telearm Bluetooth Setup ===${NC}"
echo "This script will pair your Raspberry Pi with the ESP32 operator tracker."
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo -e "${RED}This script should not be run as root. Please run as regular user.${NC}"
   exit 1
fi

# Check if bluetoothctl is available
if ! command -v bluetoothctl &> /dev/null; then
    echo -e "${RED}bluetoothctl not found. Please install bluez:${NC}"
    echo "sudo apt-get update && sudo apt-get install bluez"
    exit 1
fi

# Check if rfcomm is available
if ! command -v rfcomm &> /dev/null; then
    echo -e "${RED}rfcomm not found. Please install bluez:${NC}"
    echo "sudo apt-get update && sudo apt-get install bluez"
    exit 1
fi

# Function to scan for devices
scan_for_devices() {
    echo -e "${YELLOW}Scanning for Bluetooth devices...${NC}"
    echo "Looking for device named: $DEVICE_NAME"
    echo ""
    
    # Start scanning
    sudo bluetoothctl << EOF
power on
agent on
scan on
EOF
    
    echo "Scanning for 10 seconds..."
    sleep 10
    
    # Stop scanning and get discovered devices
    echo "Getting discovered devices..."
    devices=$(sudo bluetoothctl << EOF
scan off
devices
EOF
)
    
    echo "$devices"
    
    # Look for our device
    esp32_device=$(echo "$devices" | grep -i "$DEVICE_NAME" || true)
    
    if [ -n "$esp32_device" ]; then
        # Extract MAC address
        ESP32_MAC=$(echo "$esp32_device" | awk '{print $2}')
        echo -e "${GREEN}Found ESP32 device: $esp32_device${NC}"
        echo -e "${GREEN}MAC Address: $ESP32_MAC${NC}"
    else
        echo -e "${RED}Could not find device named '$DEVICE_NAME'${NC}"
        echo "Make sure your ESP32 is powered on and in Bluetooth mode."
        echo "Available devices:"
        echo "$devices"
        return 1
    fi
}

# Get MAC address from command line or scan
if [ $# -eq 1 ]; then
    ESP32_MAC="$1"
    echo -e "${BLUE}Using provided MAC address: $ESP32_MAC${NC}"
else
    echo "No MAC address provided. Scanning for devices..."
    scan_for_devices
fi

if [ -z "$ESP32_MAC" ]; then
    echo -e "${RED}No MAC address available. Exiting.${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Starting pairing process...${NC}"

# Pair with the device
echo "Pairing with $ESP32_MAC..."
sudo bluetoothctl << EOF
pair $ESP32_MAC
trust $ESP32_MAC
connect $ESP32_MAC
EOF

# Check if connection was successful
sleep 2
connection_status=$(sudo bluetoothctl << EOF
info $ESP32_MAC
EOF
)

if echo "$connection_status" | grep -q "Connected: yes"; then
    echo -e "${GREEN}Successfully connected to ESP32!${NC}"
else
    echo -e "${RED}Failed to connect to ESP32.${NC}"
    echo "Connection status:"
    echo "$connection_status"
    exit 1
fi

echo ""
echo -e "${YELLOW}Setting up serial port binding...${NC}"

# Unbind any existing rfcomm binding
sudo rfcomm release $SERIAL_PORT 2>/dev/null || true

# Bind the device to serial port
echo "Binding $ESP32_MAC to $SERIAL_PORT..."
sudo rfcomm bind $SERIAL_PORT $ESP32_MAC 1

# Check if binding was successful
if [ -e "$SERIAL_PORT" ]; then
    echo -e "${GREEN}Serial port binding successful!${NC}"
    echo "Device is now available at: $SERIAL_PORT"
    
    # Set permissions
    sudo chmod 666 $SERIAL_PORT
    echo "Set permissions for $SERIAL_PORT"
else
    echo -e "${RED}Serial port binding failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}=== Bluetooth Setup Complete ===${NC}"
echo "ESP32 MAC Address: $ESP32_MAC"
echo "Serial Port: $SERIAL_PORT"
echo ""
echo "You can now run teleoperation with Bluetooth mode:"
echo "python3 -m telearm.cli teleop --mode bluetooth"
echo ""
echo -e "${YELLOW}Note: This binding will be lost on reboot.${NC}"
echo "To make it permanent, run:"
echo "sudo ./setup_bluetooth_pi.sh $ESP32_MAC"
echo ""

# Test the connection
echo -e "${BLUE}Testing connection...${NC}"
if timeout 5 cat $SERIAL_PORT > /dev/null 2>&1; then
    echo -e "${GREEN}Connection test successful!${NC}"
else
    echo -e "${YELLOW}Connection test inconclusive (no data received).${NC}"
    echo "This is normal if the ESP32 is not currently transmitting data."
fi

echo ""
echo -e "${GREEN}Setup complete! You can now use Bluetooth teleoperation.${NC}"
