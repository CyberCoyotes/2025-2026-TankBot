package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NavigationConstants;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.NavigationSubsystem;

/**
 * Command to automatically balance the robot on the charging station.
 * Uses Pigeon2 gyro data to maintain balance by driving forward/backward.
 */
public class AutoBalanceCommand extends Command {
    private final DriveSubsystem2 driveSubsystem;
    private final NavigationSubsystem navigationSubsystem;
    
    private boolean isBalanced = false;
    
    public AutoBalanceCommand(DriveSubsystem2 driveSubsystem, NavigationSubsystem navigationSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        isBalanced = false;
    }
    
    @Override
    public void execute() {
        // Get the tilt angle (pitch) from the gyro
        double tiltAngle = navigationSubsystem.getTiltAngle();
        
        // Calculate drive speed based on tilt angle
        double driveSpeed = 0.0;
        
        if (Math.abs(tiltAngle) > NavigationConstants.BALANCE_THRESHOLD_DEGREES) {
            // Calculate proportional drive speed
            driveSpeed = tiltAngle * 0.02; // Proportional constant
            
            // Limit the speed
            driveSpeed = Math.max(-NavigationConstants.BALANCE_SPEED, 
                                Math.min(NavigationConstants.BALANCE_SPEED, driveSpeed));
        }
        
        // Drive the robot
        driveSubsystem.arcadeDrive(-driveSpeed, 0); // Negative to correct for tilt
        
        // Update balance status
        isBalanced = navigationSubsystem.isBalanced();
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return isBalanced;
    }
}