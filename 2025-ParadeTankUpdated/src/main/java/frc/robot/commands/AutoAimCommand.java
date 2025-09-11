package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to automatically aim at an AprilTag target using Limelight.
 * Rotates the robot to center the target in the camera's field of view.
 */
public class AutoAimCommand extends Command {
    private final DriveSubsystem2 driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    
    public AutoAimCommand(DriveSubsystem2 driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Reset vision PID controllers
        visionSubsystem.resetPID();
        
        // Set LED mode to on for better target detection
        visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.ON);
    }
    
    @Override
    public void execute() {
        if (visionSubsystem.hasValidTarget()) {
            // Get aim output from vision subsystem
            double turnOutput = visionSubsystem.getAimOutput();
            
            // Limit turn speed for safety
            turnOutput = Math.max(-0.5, Math.min(0.5, turnOutput));
            
            // Drive with turn output only (no forward/backward movement)
            driveSubsystem.arcadeDrive(0, turnOutput);
        } else {
            // No target found, stop the robot
            driveSubsystem.stop();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        
        // Return LED mode to pipeline control
        visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.PIPELINE);
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when target is centered or no target is available
        return visionSubsystem.isOnTarget() || !visionSubsystem.hasValidTarget();
    }
}