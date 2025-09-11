# Tank Robot Implementation Guide

## Overview
This document provides a comprehensive guide to the enhanced tank robot implementation with command-based framework, CTRE integration, and vision capabilities.

## Architecture

### Subsystems
1. **DriveSubsystem2** - Enhanced tank drive with TalonFX motors
2. **NavigationSubsystem** - Pigeon 2.0 IMU integration for positioning
3. **VisionSubsystem** - Limelight camera for AprilTag tracking

### Commands
1. **Drive Commands**
   - `DriveCommand2` - Teleop drive control with exponential scaling
   - `DriveDistanceCommand` - PID-controlled distance driving
   - `TurnToAngleCommand` - Precise heading control using gyro

2. **Autonomous Commands**
   - `AutoBalanceCommand` - Charging station balance using IMU
   - `AutoAimCommand` - Vision-based target aiming
   - `AprilTagTrackingCommand` - Advanced target tracking with positioning
   - `FieldNavigationCommand` - Complex autonomous sequences

3. **Utility Classes**
   - `RobotMath` - Mathematical utilities for robot control
   - Enhanced `Constants` - Organized configuration parameters

## Key Features Implemented

### 1. Motor Control Enhancement
- Proper TalonFX configuration with Phoenix 6
- Current limiting and ramp rates
- Follower motor setup
- Brake mode for safety

### 2. Odometry Integration
- Real-time distance tracking using encoders
- Integration with navigation subsystem
- Dashboard telemetry display

### 3. Navigation Capabilities
- Pigeon 2.0 IMU integration
- Heading control with wrap-around
- Balance detection and correction
- Field-relative positioning

### 4. Vision Processing
- Limelight NetworkTable integration
- AprilTag detection and tracking
- Distance calculation using camera geometry
- PID-controlled aiming and positioning

### 5. Control Enhancements
- Exponential input scaling
- Deadband filtering
- Rate limiting for smooth control
- Multiple control modes (arcade/tank)

## Testing Recommendations

### 1. Basic Functionality Tests
```java
// Test drive motors
Command motorTest = Commands.sequence(
    new DriveDistanceCommand(drive, 0.5),
    new WaitCommand(1.0),
    new DriveDistanceCommand(drive, -0.5)
);

// Test navigation
Command navTest = Commands.sequence(
    Commands.runOnce(navigation::resetHeading),
    new TurnToAngleCommand(drive, navigation, 90.0),
    new TurnToAngleCommand(drive, navigation, 0.0)
);
```

### 2. Vision System Tests
```java
// Test Limelight connectivity
SmartDashboard.putBoolean("Limelight Connected", 
    NetworkTableInstance.getDefault().getTable("limelight").isConnected());

// Test AprilTag detection
Command visionTest = new AutoAimCommand(drive, vision).withTimeout(5.0);
```

### 3. Balance System Tests
```java
// Test balance detection
SmartDashboard.putBoolean("Is Balanced", navigation.isBalanced());
SmartDashboard.putNumber("Pitch", navigation.getPitch());
SmartDashboard.putNumber("Roll", navigation.getRoll());
```

## Configuration Steps

### 1. Hardware Setup
- Configure TalonFX motor controllers with IDs 1-4
- Install Pigeon 2.0 with ID 10
- Set up Limelight with default network settings
- Connect Xbox controllers to ports 0 and 1

### 2. Software Configuration
- Deploy vendor dependencies (Phoenix 6, Limelight)
- Calibrate Pigeon 2.0 on level surface
- Configure Limelight pipeline for AprilTag detection
- Test all subsystems individually before integration

### 3. Tuning Parameters
Adjust constants in `Constants.java`:
- Drive PID values
- Vision PID values
- Motor current limits
- Input scaling factors

## Troubleshooting

### Common Issues
1. **Motors not responding**
   - Check CAN bus connections
   - Verify motor controller IDs
   - Ensure proper power distribution

2. **Navigation drift**
   - Recalibrate Pigeon 2.0
   - Check mounting orientation
   - Verify steady power supply

3. **Vision targeting issues**
   - Check Limelight network connection
   - Verify AprilTag pipeline configuration
   - Adjust camera exposure and gain

### Debugging Tools
- SmartDashboard telemetry
- Driver Station diagnostics
- Phoenix Tuner X for CTRE devices
- Limelight web interface

## Performance Optimization

### 1. Loop Timing
- Minimize processing in periodic methods
- Use appropriate command timeouts
- Implement rate limiting where needed

### 2. Network Traffic
- Limit dashboard updates to essential data
- Use efficient data types for NetworkTables
- Consider update rates for vision data

### 3. Power Management
- Monitor current consumption
- Use appropriate ramp rates
- Implement brownout protection

## Future Enhancements

### 1. Advanced Autonomous
- Pathfinding algorithms
- Multi-target AprilTag triangulation
- Dynamic obstacle avoidance

### 2. Enhanced Vision
- Multiple camera support
- Game piece detection
- Machine learning integration

### 3. Improved Control
- Swerve drive conversion
- Field-relative driving
- Advanced trajectory following

## Code Quality

### Best Practices Implemented
- Modular subsystem design
- Command composition patterns
- Comprehensive error handling
- Extensive documentation
- Type safety and null checks

### Testing Strategy
- Unit tests for utility functions
- Integration tests for subsystems
- Simulation testing support
- Hardware-in-loop validation

---

This implementation provides a solid foundation for an FRC tank robot with modern capabilities. The modular design allows for easy expansion and modification as requirements evolve.