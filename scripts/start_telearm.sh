#!/bin/bash
# Telearm Raspberry Pi Startup Script
# 
# This script provides a unified way to start the Telearm robot system
# with automatic mode detection and configuration validation.
#
# Usage: ./start_telearm.sh [mode] [options]
# Modes: wifi, bluetooth, sim
# Options: --rviz (enable ROS2 visualization), --config <file>

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
MODE="wifi"
ENABLE_RVIZ=false
CONFIG_FILE=""
VERBOSE=false

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}=== Telearm Robot Startup ===${NC}"
echo "Project Directory: $PROJECT_DIR"
echo ""

# Function to show usage
show_usage() {
    echo "Usage: $0 [mode] [options]"
    echo ""
    echo "Modes:"
    echo "  wifi       - WiFi teleoperation (default)"
    echo "  bluetooth  - Bluetooth teleoperation"
    echo "  sim        - Simulation mode (no hardware)"
    echo ""
    echo "Options:"
    echo "  --rviz     - Enable ROS2 visualization"
    echo "  --config   - Use custom config file"
    echo "  --verbose  - Enable verbose output"
    echo "  --help     - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 wifi --rviz"
    echo "  $0 bluetooth --verbose"
    echo "  $0 sim"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        wifi|bluetooth|sim)
            MODE="$1"
            shift
            ;;
        --rviz)
            ENABLE_RVIZ=true
            shift
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_usage
            exit 1
            ;;
    esac
done

# Function to log messages
log() {
    if [ "$VERBOSE" = true ]; then
        echo -e "${BLUE}[$(date '+%H:%M:%S')]${NC} $1"
    fi
}

# Function to check if running on Raspberry Pi
check_pi() {
    if [ -f /proc/cpuinfo ] && grep -q "Raspberry Pi" /proc/cpuinfo; then
        log "Running on Raspberry Pi"
        return 0
    else
        log "Not running on Raspberry Pi"
        return 1
    fi
}

# Function to check dependencies
check_dependencies() {
    log "Checking dependencies..."
    
    # Check Python
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}Python3 not found. Please install Python 3.8+${NC}"
        exit 1
    fi
    
    # Check telearm package
    if ! python3 -c "import telearm" 2>/dev/null; then
        echo -e "${YELLOW}Telearm package not found. Installing...${NC}"
        cd "$PROJECT_DIR"
        pip3 install -e .
    fi
    
    # Check hardware dependencies for non-sim modes
    if [ "$MODE" != "sim" ]; then
        # Check pyserial
        if ! python3 -c "import serial" 2>/dev/null; then
            echo -e "${YELLOW}Installing pyserial...${NC}"
            pip3 install pyserial
        fi
        
        # Check Arduino connection
        if [ -e "/dev/ttyUSB0" ] || [ -e "/dev/ttyACM0" ]; then
            log "Arduino detected"
        else
            echo -e "${YELLOW}Warning: No Arduino detected on /dev/ttyUSB0 or /dev/ttyACM0${NC}"
        fi
    fi
    
    log "Dependencies check complete"
}

# Function to validate configuration
validate_config() {
    log "Validating configuration..."
    
    # Check robot config
    if [ -f "$PROJECT_DIR/config/robot.yaml" ]; then
        log "Robot configuration found"
    else
        echo -e "${RED}Robot configuration not found: config/robot.yaml${NC}"
        exit 1
    fi
    
    # Check pins config
    if [ -f "$PROJECT_DIR/config/pins.yaml" ]; then
        log "Pins configuration found"
    else
        echo -e "${RED}Pins configuration not found: config/pins.yaml${NC}"
        exit 1
    fi
    
    # Check teleop config
    if [ -f "$PROJECT_DIR/config/teleop.yaml" ]; then
        log "Teleoperation configuration found"
    else
        echo -e "${RED}Teleoperation configuration not found: config/teleop.yaml${NC}"
        exit 1
    fi
    
    log "Configuration validation complete"
}

# Function to setup Bluetooth
setup_bluetooth() {
    if [ "$MODE" = "bluetooth" ]; then
        log "Setting up Bluetooth..."
        
        # Check if Bluetooth is available
        if ! command -v bluetoothctl &> /dev/null; then
            echo -e "${RED}Bluetooth not available. Please install bluez:${NC}"
            echo "sudo apt-get update && sudo apt-get install bluez"
            exit 1
        fi
        
        # Check if rfcomm device exists
        if [ ! -e "/dev/rfcomm0" ]; then
            echo -e "${YELLOW}Bluetooth device not bound. Running setup...${NC}"
            if [ -f "$SCRIPT_DIR/setup_bluetooth_pi.sh" ]; then
                sudo "$SCRIPT_DIR/setup_bluetooth_pi.sh"
            else
                echo -e "${RED}Bluetooth setup script not found${NC}"
                exit 1
            fi
        else
            log "Bluetooth device already bound"
        fi
    fi
}

# Function to start ROS2 visualization
start_rviz() {
    if [ "$ENABLE_RVIZ" = true ]; then
        log "Starting ROS2 visualization..."
        
        # Check if ROS2 is available
        if ! command -v ros2 &> /dev/null; then
            echo -e "${YELLOW}ROS2 not found. Skipping visualization.${NC}"
            return 0
        fi
        
        # Start RViz in background
        cd "$PROJECT_DIR"
        ros2 launch telearm_ros2 telearm_rviz.launch.py &
        RVIZ_PID=$!
        echo -e "${GREEN}ROS2 visualization started (PID: $RVIZ_PID)${NC}"
        sleep 2
    fi
}

# Function to start teleoperation
start_teleoperation() {
    log "Starting teleoperation in $MODE mode..."
    
    cd "$PROJECT_DIR"
    
    # Build command
    CMD="python3 -m telearm.cli teleop --mode $MODE"
    
    if [ "$VERBOSE" = true ]; then
        CMD="$CMD --verbose"
    fi
    
    if [ -n "$CONFIG_FILE" ]; then
        CMD="$CMD --config $CONFIG_FILE"
    fi
    
    echo -e "${GREEN}Starting teleoperation...${NC}"
    echo "Command: $CMD"
    echo ""
    
    # Start teleoperation
    exec $CMD
}

# Function to cleanup on exit
cleanup() {
    log "Cleaning up..."
    
    if [ "$ENABLE_RVIZ" = true ] && [ -n "$RVIZ_PID" ]; then
        echo -e "${YELLOW}Stopping ROS2 visualization...${NC}"
        kill $RVIZ_PID 2>/dev/null || true
    fi
    
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Main execution
main() {
    echo -e "${BLUE}Mode: $MODE${NC}"
    echo -e "${BLUE}ROS2 Visualization: $ENABLE_RVIZ${NC}"
    echo -e "${BLUE}Verbose: $VERBOSE${NC}"
    echo ""
    
    # Run checks
    check_dependencies
    validate_config
    setup_bluetooth
    
    # Start services
    start_rviz
    start_teleoperation
}

# Run main function
main "$@"
