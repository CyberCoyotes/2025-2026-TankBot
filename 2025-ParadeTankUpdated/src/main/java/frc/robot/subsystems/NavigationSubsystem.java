package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NavigationConstants;

/**
 * Navigation subsystem that handles robot positioning and orientation using Pigeon2.
 * Provides odometry, heading control, and balance detection for autonomous operations.
 */
public class NavigationSubsystem extends SubsystemBase {
    private final Pigeon2 pigeon;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field2d;
    
    private double leftDistanceMeters = 0;
    private double rightDistanceMeters = 0;
    
    public NavigationSubsystem() {
        pigeon = new Pigeon2(NavigationConstants.PIGEON_ID);
        
        // Reset the Pigeon2 to ensure clean startup
        pigeon.reset();
        
        // Initialize odometry with starting pose
        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            leftDistanceMeters,
            rightDistanceMeters
        );
        
        // Create field display for dashboard
        field2d = new Field2d();
        SmartDashboard.putData("Field", field2d);
    }
    
    @Override
    public void periodic() {
        // Update odometry with current sensor readings
        odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            leftDistanceMeters,
            rightDistanceMeters
        );
        
        // Update field display
        field2d.setRobotPose(getPose());
        
        // Put telemetry data on dashboard
        SmartDashboard.putNumber("Heading (degrees)", getHeading());
        SmartDashboard.putNumber("Roll (degrees)", getRoll());
        SmartDashboard.putNumber("Pitch (degrees)", getPitch());
        SmartDashboard.putBoolean("Is Balanced", isBalanced());
        SmartDashboard.putString("Robot Pose", getPose().toString());
    }
    
    /**
     * Gets the current heading of the robot in degrees.
     * @return Heading in degrees (-180 to 180)
     */
    public double getHeading() {
        return pigeon.getYaw().getValueAsDouble();
    }
    
    /**
     * Gets the current roll of the robot in degrees.
     * @return Roll in degrees
     */
    public double getRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }
    
    /**
     * Gets the current pitch of the robot in degrees.
     * @return Pitch in degrees
     */
    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }
    
    /**
     * Gets the current rotation rate in degrees per second.
     * @return Rotation rate in degrees/second
     */
    public double getTurnRate() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }
    
    /**
     * Resets the heading to zero degrees.
     */
    public void resetHeading() {
        pigeon.reset();
    }
    
    /**
     * Sets the heading to a specific angle.
     * @param degrees The angle to set in degrees
     */
    public void setHeading(double degrees) {
        pigeon.setYaw(degrees);
    }
    
    /**
     * Gets the current robot pose.
     * @return The robot's current pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    /**
     * Resets the odometry to a specific pose.
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getHeading()),
            leftDistanceMeters,
            rightDistanceMeters,
            pose
        );
    }
    
    /**
     * Updates the odometry with encoder distances.
     * This should be called from the drive subsystem.
     * @param leftMeters Left side distance in meters
     * @param rightMeters Right side distance in meters
     */
    public void updateOdometry(double leftMeters, double rightMeters) {
        this.leftDistanceMeters = leftMeters;
        this.rightDistanceMeters = rightMeters;
    }
    
    /**
     * Checks if the robot is balanced on the charging station.
     * @return True if robot is balanced within threshold
     */
    public boolean isBalanced() {
        return Math.abs(getPitch()) < NavigationConstants.BALANCE_THRESHOLD_DEGREES &&
               Math.abs(getRoll()) < NavigationConstants.BALANCE_THRESHOLD_DEGREES;
    }
    
    /**
     * Gets the tilt angle for balance correction.
     * @return Tilt angle for driving direction
     */
    public double getTiltAngle() {
        return getPitch(); // Use pitch for forward/backward balance
    }
    
    /**
     * Gets the rotation for field-relative driving.
     * @return Current rotation as Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
}