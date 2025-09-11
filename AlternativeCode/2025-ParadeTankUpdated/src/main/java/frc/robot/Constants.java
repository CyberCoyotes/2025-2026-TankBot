// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Controller Constants
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Drive Constants
    public static final class DriveConstants {
        public static final int RIGHT_LEADER_ID = 1;    // Right Leader Motor ID - TalonFX
        public static final int RIGHT_FOLLOWER_ID = 2;  // Right Follower Motor ID - TalonFX
        public static final int LEFT_LEADER_ID = 3;     // Left Leader Motor ID - TalonFX
        public static final int LEFT_FOLLOWER_ID = 4;   // Left Follower Motor ID - TalonFX
        
        // Drive characteristics
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0);
        public static final double GEAR_RATIO = 10.71; // L1 gearing
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.0);
        
        // Drive limits
        public static final double MAX_VELOCITY_MPS = 4.0; // meters per second
        public static final double MAX_ACCELERATION_MPSPS = 3.0; // meters per second squared
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI; // radians per second
        
        // Motor configuration
        public static final double CURRENT_LIMIT_AMPS = 40.0;
        public static final double RAMP_RATE_SECONDS = 0.5;
        
        // Control constants
        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final double DRIVE_EXPO = 2.0; // Exponential scaling for finer control
        public static final double TURN_EXPO = 2.0;
    }

    // Navigation Constants
    public static final class NavigationConstants {
        public static final int PIGEON_ID = 10; // Pigeon 2.0 CAN ID
        public static final double BALANCE_THRESHOLD_DEGREES = 2.5;
        public static final double BALANCE_SPEED = 0.2;
    }

    // Vision Constants  
    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24.0);
        public static final double CAMERA_PITCH_DEGREES = 15.0; // Angle of camera mount
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104.0); // AprilTag height
        
        // Vision processing
        public static final double VALID_TARGET_THRESHOLD = 0.1; // Minimum area for valid target
        public static final double AIM_TOLERANCE_DEGREES = 2.0;
        public static final double RANGE_TOLERANCE_METERS = 0.5;
        
        // Auto-aim PID constants
        public static final double AIM_KP = 0.02;
        public static final double AIM_KI = 0.001;
        public static final double AIM_KD = 0.005;
        
        // Distance PID constants  
        public static final double RANGE_KP = 3.0;
        public static final double RANGE_KI = 0.0;
        public static final double RANGE_KD = 0.0;
    }

    // Autonomous Constants
    public static final class AutoConstants {
        public static final double AUTO_DRIVE_SPEED = 0.5;
        public static final double AUTO_TURN_SPEED = 0.3;
        public static final double AUTO_BALANCE_SPEED = 0.2;
        
        // Path following
        public static final double MAX_VELOCITY_MPS = 2.0;
        public static final double MAX_ACCELERATION_MPSPS = 2.0;
        
        // PID Constants for autonomous driving
        public static final double K_P_DRIVE = 3.0;
        public static final double K_I_DRIVE = 0.0; 
        public static final double K_D_DRIVE = 0.0;
        
        public static final double K_P_TURN = 5.0;
        public static final double K_I_TURN = 0.0;
        public static final double K_D_TURN = 0.0;
    }
}
