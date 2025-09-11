package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem that handles Limelight camera operations and AprilTag tracking.
 * Provides target detection, distance calculation, and auto-aim functionality.
 */
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final PIDController aimController;
    private final PIDController rangeController;
    
    // Vision data
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetArea = 0.0;
    private boolean hasValidTarget = false;
    private int targetID = -1;
    
    public VisionSubsystem() {
        // Get Limelight NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
        
        // Initialize PID controllers
        aimController = new PIDController(
            VisionConstants.AIM_KP,
            VisionConstants.AIM_KI, 
            VisionConstants.AIM_KD
        );
        aimController.setTolerance(VisionConstants.AIM_TOLERANCE_DEGREES);
        
        rangeController = new PIDController(
            VisionConstants.RANGE_KP,
            VisionConstants.RANGE_KI,
            VisionConstants.RANGE_KD
        );
        rangeController.setTolerance(VisionConstants.RANGE_TOLERANCE_METERS);
        
        // Set LED mode to pipeline default
        setLEDMode(LEDMode.PIPELINE);
        
        // Set camera mode to vision processing
        setCameraMode(CameraMode.VISION);
        
        // Set pipeline to AprilTag detection
        setPipeline(0);
    }
    
    @Override
    public void periodic() {
        // Update vision data from Limelight
        updateVisionData();
        
        // Update dashboard
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);
        SmartDashboard.putNumber("Target X Offset", targetX);
        SmartDashboard.putNumber("Target Y Offset", targetY);
        SmartDashboard.putNumber("Target Area", targetArea);
        SmartDashboard.putNumber("Target ID", targetID);
        SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
        SmartDashboard.putBoolean("On Target", isOnTarget());
        SmartDashboard.putBoolean("In Range", isInRange());
    }
    
    /**
     * Updates vision data from the Limelight NetworkTable.
     */
    private void updateVisionData() {
        // Check if we have a valid target
        hasValidTarget = limelightTable.getEntry("tv").getDouble(0) == 1.0;
        
        if (hasValidTarget) {
            // Get target data
            targetX = limelightTable.getEntry("tx").getDouble(0.0);
            targetY = limelightTable.getEntry("ty").getDouble(0.0);
            targetArea = limelightTable.getEntry("ta").getDouble(0.0);
            targetID = (int) limelightTable.getEntry("tid").getDouble(-1.0);
        } else {
            // Reset values when no target
            targetX = 0.0;
            targetY = 0.0;
            targetArea = 0.0;
            targetID = -1;
        }
    }
    
    /**
     * Checks if we have a valid target.
     * @return True if a valid target is detected
     */
    public boolean hasValidTarget() {
        return hasValidTarget && targetArea > VisionConstants.VALID_TARGET_THRESHOLD;
    }
    
    /**
     * Gets the horizontal offset to the target.
     * @return X offset in degrees (-27 to 27)
     */
    public double getTargetX() {
        return targetX;
    }
    
    /**
     * Gets the vertical offset to the target.
     * @return Y offset in degrees (-20.5 to 20.5)
     */
    public double getTargetY() {
        return targetY;
    }
    
    /**
     * Gets the target area percentage.
     * @return Area as percentage of image (0-100)
     */
    public double getTargetArea() {
        return targetArea;
    }
    
    /**
     * Gets the detected AprilTag ID.
     * @return AprilTag ID, or -1 if no tag detected
     */
    public int getTargetID() {
        return targetID;
    }
    
    /**
     * Calculates distance to target using trigonometry.
     * @return Distance to target in meters
     */
    public double getDistanceToTarget() {
        if (!hasValidTarget()) {
            return 0.0;
        }
        
        double heightDifference = VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
        double angleToTarget = VisionConstants.CAMERA_PITCH_DEGREES + targetY;
        
        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }
    
    /**
     * Gets the turn output for auto-aiming.
     * @return Turn speed (-1.0 to 1.0)
     */
    public double getAimOutput() {
        if (!hasValidTarget()) {
            return 0.0;
        }
        
        return aimController.calculate(targetX, 0.0);
    }
    
    /**
     * Gets the drive output for range control.
     * @param desiredDistance Desired distance to target in meters
     * @return Drive speed (-1.0 to 1.0)
     */
    public double getRangeOutput(double desiredDistance) {
        if (!hasValidTarget()) {
            return 0.0;
        }
        
        return rangeController.calculate(getDistanceToTarget(), desiredDistance);
    }
    
    /**
     * Checks if the robot is aimed at the target.
     * @return True if aimed within tolerance
     */
    public boolean isOnTarget() {
        return hasValidTarget() && aimController.atSetpoint();
    }
    
    /**
     * Checks if the robot is at the desired range.
     * @return True if at desired range within tolerance
     */
    public boolean isInRange() {
        return hasValidTarget() && rangeController.atSetpoint();
    }
    
    /**
     * Sets the LED mode.
     * @param mode LED mode to set
     */
    public void setLEDMode(LEDMode mode) {
        limelightTable.getEntry("ledMode").setNumber(mode.value);
    }
    
    /**
     * Sets the camera mode.
     * @param mode Camera mode to set
     */
    public void setCameraMode(CameraMode mode) {
        limelightTable.getEntry("camMode").setNumber(mode.value);
    }
    
    /**
     * Sets the active pipeline.
     * @param pipeline Pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Takes a snapshot with the Limelight.
     */
    public void takeSnapshot() {
        limelightTable.getEntry("snapshot").setNumber(1);
    }
    
    /**
     * Resets PID controllers.
     */
    public void resetPID() {
        aimController.reset();
        rangeController.reset();
    }
    
    // Enum for LED modes
    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);
        
        public final int value;
        
        LEDMode(int value) {
            this.value = value;
        }
    }
    
    // Enum for camera modes
    public enum CameraMode {
        VISION(0),
        DRIVER(1);
        
        public final int value;
        
        CameraMode(int value) {
            this.value = value;
        }
    }
}