# Safety Guide

**⚠️ IMPORTANT: Read this safety guide completely before operating the robot arm.**

## General Safety Principles

1. **Never operate the robot without proper supervision**
2. **Always power off when making adjustments or connections**
3. **Keep hands and body parts away from moving parts during operation**
4. **Ensure adequate workspace clearance**
5. **Test movements with small amplitudes first**

## Electrical Safety

### Power Supply
- Use only the specified 5V power supply with adequate current capacity (6A+)
- Never exceed the rated voltage for servos (typically 4.8V - 6V)
- Ensure proper grounding of all electrical components
- Use appropriate wire gauge for current capacity

### Connections
- Always power off before making or changing connections
- Double-check polarity before connecting power
- Secure all connections to prevent shorts or loose connections
- Keep power cables away from moving parts

### Arduino Safety
- Do not exceed the Arduino's input voltage limits
- Use external power supply for servos, not Arduino's 5V pin
- Ensure proper USB connection for communication
- Protect Arduino from mechanical stress

## Mechanical Safety

### Workspace Requirements
- Maintain at least 30cm clearance in all directions around the robot
- Ensure no obstacles in the robot's workspace
- Keep the work area well-lit and free of clutter
- Use a stable, level mounting surface

### Joint Limits
- Respect the joint limits specified in `config/robot.yaml`
- Never force joints beyond their physical limits
- Test joint limits manually before programming
- Consider adding physical limit switches for safety

### Servo Safety
- Do not stall servos by forcing them against obstacles
- Allow servos to complete movements before issuing new commands
- Monitor servo temperature during extended operation
- Replace damaged or overheating servos immediately

## Software Safety

### Movement Validation
- Always validate target positions before execution
- Use simulation mode (`--sim`) for testing new programs
- Implement position limits in your control software
- Check for singularities in inverse kinematics

### Error Handling
- Implement proper error checking in your code
- Monitor for communication errors with hardware
- Have emergency stop procedures ready
- Log all robot operations for debugging

### Configuration Safety
- Verify `config/robot.yaml` matches your physical hardware
- Test configuration changes in simulation first
- Keep backups of working configurations
- Document any hardware modifications

## Emergency Procedures

### Emergency Stop
1. **Immediate**: Disconnect power supply
2. **Secondary**: Unplug USB cable to stop communication
3. **Tertiary**: Use physical emergency stop if available

### Recovery Procedures
1. Power off the system completely
2. Check for any mechanical damage or obstructions
3. Verify all connections are secure
4. Power on and run calibration routine
5. Test with small movements before normal operation

### Troubleshooting Safety Issues
- If servos are not responding: Check power and connections
- If movement is erratic: Power off and check for loose connections
- If robot moves unexpectedly: Disconnect power immediately
- If smoke or burning smell: Disconnect power immediately and inspect

## Operational Guidelines

### Pre-Operation Checklist
- [ ] Power supply voltage verified (5V ± 0.2V)
- [ ] All connections secure and correct
- [ ] Workspace clear and adequate
- [ ] Joint limits verified and set
- [ ] Emergency stop procedure understood
- [ ] Software tested in simulation mode

### During Operation
- Monitor robot movement continuously
- Keep emergency stop accessible
- Do not leave robot unattended during operation
- Stop immediately if any unusual behavior is observed
- Maintain safe distance from moving parts

### Post-Operation
- Return robot to home position
- Power off the system
- Clean and inspect the robot for any issues
- Document any problems or observations

## Risk Assessment

### Low Risk Operations
- Small amplitude movements (< 5cm)
- Slow speed operations (< 30 deg/s)
- Simulation mode testing
- Status checking and calibration

### Medium Risk Operations
- Normal workspace movements
- Standard speed operations
- Automated sequences
- ROS 2 visualization

### High Risk Operations
- Large amplitude movements (> 10cm)
- High speed operations (> 60 deg/s)
- Complex multi-joint motions
- Operation near obstacles

## Maintenance Safety

### Regular Maintenance
- Check all connections monthly
- Inspect servo mounting and alignment
- Clean robot surfaces as needed
- Verify joint limit switches (if installed)

### Servo Maintenance
- Do not disassemble servos unless absolutely necessary
- Replace servos showing signs of wear or damage
- Keep servos dry and clean
- Monitor for unusual noise or vibration

### Software Maintenance
- Keep software and firmware updated
- Test all updates in simulation first
- Backup working configurations
- Document any changes or modifications

## Legal and Liability

- This robot is for educational and research purposes
- Users assume all responsibility for safe operation
- Follow all applicable local safety regulations
- Ensure proper supervision for inexperienced users
- Document all safety procedures and incidents

## Contact and Support

For safety-related questions or incidents:
1. Immediately stop operation if safety is compromised
2. Document the issue thoroughly
3. Contact technical support with detailed information
4. Do not attempt to operate until safety issues are resolved

---

**Remember: Safety is everyone's responsibility. When in doubt, stop and ask for help.**
