package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem2;

/**
 * Command to drive a specific distance using encoder feedback.
 * Uses PID control for accurate positioning.
 */
public class DriveDistanceCommand extends Command {
    private final DriveSubsystem2 driveSubsystem;
    private final double targetDistanceMeters;
    private final PIDController pidController;
    
    private double startDistance;
    
    public DriveDistanceCommand(DriveSubsystem2 driveSubsystem, double distanceMeters) {
        this.driveSubsystem = driveSubsystem;
        this.targetDistanceMeters = distanceMeters;
        
        // Initialize PID controller
        pidController = new PIDController(
            AutoConstants.K_P_DRIVE,
            AutoConstants.K_I_DRIVE,
            AutoConstants.K_D_DRIVE
        );
        pidController.setTolerance(0.05); // 5cm tolerance
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Record starting position
        startDistance = driveSubsystem.getAverageDistanceMeters();
        
        // Reset PID controller
        pidController.reset();
        
        // Set PID setpoint to target distance from start
        pidController.setSetpoint(startDistance + targetDistanceMeters);
    }
    
    @Override
    public void execute() {
        // Get current average distance
        double currentDistance = driveSubsystem.getAverageDistanceMeters();
        
        // Calculate drive output using PID
        double driveOutput = pidController.calculate(currentDistance);
        
        // Limit output speed for safety
        driveOutput = Math.max(-AutoConstants.AUTO_DRIVE_SPEED, 
                              Math.min(AutoConstants.AUTO_DRIVE_SPEED, driveOutput));
        
        // Drive forward/backward only
        driveSubsystem.arcadeDrive(driveOutput, 0);
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}