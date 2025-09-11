package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Enhanced drive subsystem for tank robot with improved motor configuration,
 * odometry integration, and telemetry. Supports both arcade and tank drive modes.
 */
public class DriveSubsystem2 extends SubsystemBase {
    private final DifferentialDrive drive;
    private final TalonFX rightLeaderMotor;
    private final TalonFX rightFollowerMotor;
    private final TalonFX leftLeaderMotor;
    private final TalonFX leftFollowerMotor;
    
    // Odometry tracking
    private double leftDistanceMeters = 0.0;
    private double rightDistanceMeters = 0.0;
    private double lastLeftPosition = 0.0;
    private double lastRightPosition = 0.0;

    public DriveSubsystem2() {
        // Initialize motors
        rightLeaderMotor = new TalonFX(DriveConstants.RIGHT_LEADER_ID);
        rightFollowerMotor = new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID);
        leftLeaderMotor = new TalonFX(DriveConstants.LEFT_LEADER_ID);
        leftFollowerMotor = new TalonFX(DriveConstants.LEFT_FOLLOWER_ID);

        // Configure all motors
        configureMotor(rightLeaderMotor, false);  // Right side not inverted
        configureMotor(rightFollowerMotor, false);
        configureMotor(leftLeaderMotor, true);   // Left side inverted
        configureMotor(leftFollowerMotor, true);

        // Set up follower relationships
        rightFollowerMotor.setControl(new Follower(rightLeaderMotor.getDeviceID(), false));
        leftFollowerMotor.setControl(new Follower(leftLeaderMotor.getDeviceID(), false));

        // Create differential drive with leader motors
        drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);
        
        // Configure differential drive safety
        drive.setSafetyEnabled(false);
        drive.setMaxOutput(1.0);
        
        // Reset encoders
        resetEncoders();
    }
    
    @Override
    public void periodic() {
        // Update odometry
        updateOdometry();
        
        // Update telemetry
        SmartDashboard.putNumber("Left Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Distance (m)", getRightDistanceMeters());
        SmartDashboard.putNumber("Left Velocity (mps)", getLeftVelocityMPS());
        SmartDashboard.putNumber("Right Velocity (mps)", getRightVelocityMPS());
        SmartDashboard.putNumber("Average Distance (m)", getAverageDistanceMeters());
    }

    /**
     * Configures a TalonFX motor with standard settings.
     * @param motor The motor to configure
     * @param inverted Whether the motor should be inverted
     */
    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output configuration
        config.MotorOutput.Inverted = inverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limiting
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = DriveConstants.CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentThreshold = DriveConstants.CURRENT_LIMIT_AMPS + 10;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        
        // Ramp rate for smooth acceleration
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DriveConstants.RAMP_RATE_SECONDS;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.RAMP_RATE_SECONDS;
        
        // Apply configuration
        motor.getConfigurator().apply(config);
        
        // Reset position to zero
        motor.setPosition(0);
    }

    /**
     * Arcade drive method with deadband and exponential scaling.
     * @param forward Forward speed (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     */
    public void arcadeDrive(double forward, double rotation) {
        // Apply deadband
        forward = MathUtil.applyDeadband(forward, DriveConstants.JOYSTICK_DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, DriveConstants.JOYSTICK_DEADBAND);
        
        // Apply exponential scaling for finer control
        forward = Math.copySign(Math.pow(Math.abs(forward), DriveConstants.DRIVE_EXPO), forward);
        rotation = Math.copySign(Math.pow(Math.abs(rotation), DriveConstants.TURN_EXPO), rotation);
        
        drive.arcadeDrive(forward, rotation);
    }

    /**
     * Tank drive method for direct wheel control.
     * @param leftSpeed Left side speed (-1.0 to 1.0)
     * @param rightSpeed Right side speed (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Sets the wheel speeds directly for autonomous control.
     * @param wheelSpeeds Target wheel speeds
     */
    public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double leftOutput = wheelSpeeds.leftMetersPerSecond / DriveConstants.MAX_VELOCITY_MPS;
        double rightOutput = wheelSpeeds.rightMetersPerSecond / DriveConstants.MAX_VELOCITY_MPS;
        
        tankDrive(leftOutput, rightOutput);
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        drive.stopMotor();
    }

    /**
     * Updates odometry calculations.
     */
    private void updateOdometry() {
        // Get current encoder positions in rotations
        double leftRotations = leftLeaderMotor.getPosition().getValueAsDouble();
        double rightRotations = rightLeaderMotor.getPosition().getValueAsDouble();
        
        // Calculate distance traveled since last update
        double leftDelta = (leftRotations - lastLeftPosition) * 
                          (DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / DriveConstants.GEAR_RATIO;
        double rightDelta = (rightRotations - lastRightPosition) * 
                           (DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / DriveConstants.GEAR_RATIO;
        
        // Update total distances
        leftDistanceMeters += leftDelta;
        rightDistanceMeters += rightDelta;
        
        // Update last positions
        lastLeftPosition = leftRotations;
        lastRightPosition = rightRotations;
    }

    /**
     * Gets the left encoder distance in meters.
     * @return Left distance in meters
     */
    public double getLeftDistanceMeters() {
        return leftDistanceMeters;
    }

    /**
     * Gets the right encoder distance in meters.
     * @return Right distance in meters
     */
    public double getRightDistanceMeters() {
        return rightDistanceMeters;
    }

    /**
     * Gets the average encoder distance in meters.
     * @return Average distance in meters
     */
    public double getAverageDistanceMeters() {
        return (leftDistanceMeters + rightDistanceMeters) / 2.0;
    }

    /**
     * Gets the left wheel velocity in meters per second.
     * @return Left velocity in mps
     */
    public double getLeftVelocityMPS() {
        double rps = leftLeaderMotor.getVelocity().getValueAsDouble();
        return (rps * DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / DriveConstants.GEAR_RATIO;
    }

    /**
     * Gets the right wheel velocity in meters per second.
     * @return Right velocity in mps
     */
    public double getRightVelocityMPS() {
        double rps = rightLeaderMotor.getVelocity().getValueAsDouble();
        return (rps * DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / DriveConstants.GEAR_RATIO;
    }

    /**
     * Gets the current wheel speeds.
     * @return Current wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMPS(), getRightVelocityMPS());
    }

    /**
     * Resets the encoder distances to zero.
     */
    public void resetEncoders() {
        leftLeaderMotor.setPosition(0);
        rightLeaderMotor.setPosition(0);
        leftDistanceMeters = 0.0;
        rightDistanceMeters = 0.0;
        lastLeftPosition = 0.0;
        lastRightPosition = 0.0;
    }

    /**
     * Legacy drive method for compatibility.
     * @param forward Forward speed
     * @param rotation Rotation speed
     */
    public void drive(double forward, double rotation) {
        arcadeDrive(forward, rotation);
    }
}