package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Advanced command for tracking and positioning relative to AprilTags.
 * Combines aiming and range control for precise robot positioning.
 */
public class AprilTagTrackingCommand extends Command {
    private final DriveSubsystem2 driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final double targetDistance;
    private final int targetID;
    
    private Timer stableTimer = new Timer();
    private boolean wasOnTarget = false;
    
    /**
     * Creates a new AprilTag tracking command.
     * @param driveSubsystem Drive subsystem
     * @param visionSubsystem Vision subsystem
     * @param targetDistance Desired distance to target in meters
     * @param targetID Specific AprilTag ID to track (use -1 for any tag)
     */
    public AprilTagTrackingCommand(DriveSubsystem2 driveSubsystem, 
                                  VisionSubsystem visionSubsystem,
                                  double targetDistance,
                                  int targetID) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetDistance = targetDistance;
        this.targetID = targetID;
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Creates a command that tracks any AprilTag at the specified distance.
     * @param driveSubsystem Drive subsystem
     * @param visionSubsystem Vision subsystem
     * @param targetDistance Desired distance to target in meters
     */
    public AprilTagTrackingCommand(DriveSubsystem2 driveSubsystem, 
                                  VisionSubsystem visionSubsystem,
                                  double targetDistance) {
        this(driveSubsystem, visionSubsystem, targetDistance, -1);
    }
    
    @Override
    public void initialize() {
        // Reset vision PID controllers
        visionSubsystem.resetPID();
        
        // Turn on LEDs for better detection
        visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.ON);
        
        // Reset stable timer
        stableTimer.reset();
        wasOnTarget = false;
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        if (!visionSubsystem.hasValidTarget()) {
            // No target - stop and search
            driveSubsystem.arcadeDrive(0, 0.2); // Slow turn to search
            resetStableTimer();
            return;
        }
        
        // Check if we care about specific tag ID
        if (targetID >= 0 && visionSubsystem.getTargetID() != targetID) {
            // Wrong tag - keep searching
            driveSubsystem.arcadeDrive(0, 0.2);
            resetStableTimer();
            return;
        }
        
        // Calculate aim output
        double aimOutput = visionSubsystem.getAimOutput();
        
        // Calculate range output
        double rangeOutput = visionSubsystem.getRangeOutput(targetDistance);
        
        // Limit outputs for safety
        aimOutput = Math.max(-0.4, Math.min(0.4, aimOutput));
        rangeOutput = Math.max(-0.6, Math.min(0.6, rangeOutput));
        
        // Apply differential scaling - prioritize aiming over range when far from target
        double currentDistance = visionSubsystem.getDistanceToTarget();
        if (currentDistance > targetDistance * 1.5) {
            // Far away - focus more on aiming
            rangeOutput *= 0.7;
        }
        
        // Drive the robot
        driveSubsystem.arcadeDrive(rangeOutput, aimOutput);
        
        // Check if we're on target and in range
        boolean onTarget = visionSubsystem.isOnTarget() && 
                          Math.abs(currentDistance - targetDistance) < VisionConstants.RANGE_TOLERANCE_METERS;
        
        if (onTarget && !wasOnTarget) {
            // Just got on target - start timer
            stableTimer.restart();
        } else if (!onTarget) {
            // Not on target - reset timer
            resetStableTimer();
        }
        
        wasOnTarget = onTarget;
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        
        // Return LED control to pipeline
        visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.PIPELINE);
        
        if (!interrupted) {
            // Take a snapshot when successfully positioned
            visionSubsystem.takeSnapshot();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when stable on target for specified time
        return stableTimer.hasElapsed(1.0); // 1 second stable time
    }
    
    /**
     * Resets the stable timer and tracking state.
     */
    private void resetStableTimer() {
        stableTimer.stop();
        stableTimer.reset();
        wasOnTarget = false;
    }
    
    /**
     * Gets the current distance to the target.
     * @return Distance in meters, or 0 if no target
     */
    public double getCurrentDistance() {
        return visionSubsystem.getDistanceToTarget();
    }
    
    /**
     * Gets the current target ID being tracked.
     * @return AprilTag ID, or -1 if no target
     */
    public int getCurrentTargetID() {
        return visionSubsystem.getTargetID();
    }
    
    /**
     * Checks if the robot is currently tracking a target.
     * @return True if actively tracking
     */
    public boolean isTracking() {
        return visionSubsystem.hasValidTarget() && 
               (targetID < 0 || visionSubsystem.getTargetID() == targetID);
    }
}