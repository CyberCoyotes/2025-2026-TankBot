# 2025-2026-TankRobot

## Overview
FRC robot code in Java using a comprehensive command-based framework with advanced capabilities including navigation, vision processing, and autonomous operations.

## Hardware Configuration
- **Drive System**: Tank treads with four TalonFX motors (two per side)
- **Navigation**: Pigeon 2.0 IMU for heading, pitch/roll, and odometry
- **Vision**: Limelight camera for AprilTag detection and tracking
- **Control**: Xbox controllers for driver and operator input

## Key Features

### Drive System
- Arcade-style driving with Xbox controller
- Enhanced motor configuration with current limiting and ramp rates
- Real-time odometry tracking
- Exponential input scaling for precise control
- Integrated safety features

### Navigation Subsystem
- Pigeon 2.0 IMU integration for precise heading control
- Automatic balance detection and correction
- Field-relative positioning with odometry
- Dashboard telemetry display

### Vision Subsystem
- Limelight integration for AprilTag detection
- Automatic target tracking and aiming
- Distance calculation using target geometry
- PID-controlled positioning
- Multiple camera modes (vision/driver)

### Autonomous Capabilities
- **Basic Movements**: Drive distance, turn to angle
- **Balance Operations**: Auto-balance on charging station
- **Vision Operations**: Auto-aim, AprilTag tracking
- **Complex Sequences**: Multi-phase autonomous routines
- **Safety Features**: Timeouts and error handling

## Command Structure

### Drive Commands
- `DriveCommand2`: Enhanced teleop drive control
- `DriveDistanceCommand`: Precise distance driving
- `TurnToAngleCommand`: Accurate heading control

### Autonomous Commands
- `AutoBalanceCommand`: Charging station balance
- `AutoAimCommand`: Vision-based aiming
- `AprilTagTrackingCommand`: Advanced target tracking
- `FieldNavigationCommand`: Complex autonomous sequences

### Utility Classes
- `RobotMath`: Mathematical utilities
- Enhanced constants organization
- Comprehensive error handling

## Control Mapping

### Driver Controller (Xbox Controller Port 0)
- **Left Stick Y**: Forward/backward drive
- **Right Stick X**: Rotation
- **A Button**: Reset gyro heading
- **B Button**: Reset drive encoders
- **X Button**: Auto-balance mode
- **Y Button**: Auto-aim mode
- **Left Bumper**: Turn 90° left
- **Right Bumper**: Turn 90° right
- **Start Button**: AprilTag tracking mode

### Operator Controller (Xbox Controller Port 1)
- **A Button**: Turn Limelight LEDs on
- **B Button**: Turn Limelight LEDs off
- **X Button**: Set camera to vision mode
- **Y Button**: Set camera to driver mode
- **Start Button**: Take Limelight snapshot

## Autonomous Options
1. **Do Nothing**: Stationary (default)
2. **Mobility**: Simple drive out of community
3. **Drive and Balance**: Drive to charging station and balance
4. **Vision Autonomous**: AprilTag-based positioning
5. **Complex Navigation**: Multi-phase field navigation
6. **Defensive**: Minimal movement positioning
7. **System Test**: Validate all subsystems

## Installation & Setup

### Dependencies
- CTRE Phoenix 6 (TalonFX motor controllers, Pigeon 2.0)
- Limelight vision library
- WPILib 2025 command framework

### Hardware IDs
```java
// Drive Motors
RIGHT_LEADER_ID = 1
RIGHT_FOLLOWER_ID = 2  
LEFT_LEADER_ID = 3
LEFT_FOLLOWER_ID = 4

// Navigation
PIGEON_ID = 10

// Controllers
DRIVER_CONTROLLER_PORT = 0
OPERATOR_CONTROLLER_PORT = 1
```

## Safety Features
- Current limiting on all motors
- Ramp rates for smooth acceleration
- Command timeouts for autonomous safety
- Deadband filtering for controller inputs
- Comprehensive error handling

## Development Notes
- Modular subsystem design for easy testing
- Extensive telemetry for debugging
- Configurable constants for easy tuning
- Command composition for complex behaviors
- Simulation support for development

## Future Enhancements
- Path following with trajectory generation
- Advanced field positioning algorithms
- Multi-target AprilTag triangulation
- Machine learning-based vision processing
- Automated game piece manipulation

---
*This codebase demonstrates best practices for FRC command-based programming with integration of modern CTRE and vision systems.*
