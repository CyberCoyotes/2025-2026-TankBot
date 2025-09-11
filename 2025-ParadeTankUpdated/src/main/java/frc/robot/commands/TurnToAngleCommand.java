package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.NavigationSubsystem;

/**
 * Command to turn the robot to a specific heading using Pigeon2 gyro.
 * Uses PID control for accurate turning.
 */
public class TurnToAngleCommand extends Command {
    private final DriveSubsystem2 driveSubsystem;
    private final NavigationSubsystem navigationSubsystem;
    private final double targetAngleDegrees;
    private final PIDController pidController;
    
    public TurnToAngleCommand(DriveSubsystem2 driveSubsystem, 
                             NavigationSubsystem navigationSubsystem, 
                             double targetAngleDegrees) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.targetAngleDegrees = targetAngleDegrees;
        
        // Initialize PID controller
        pidController = new PIDController(
            AutoConstants.K_P_TURN,
            AutoConstants.K_I_TURN,
            AutoConstants.K_D_TURN
        );
        
        // Configure PID for continuous input (angle wrapping)
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(2.0); // 2 degree tolerance
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Set PID setpoint to target angle
        pidController.setSetpoint(targetAngleDegrees);
        
        // Reset PID controller
        pidController.reset();
    }
    
    @Override
    public void execute() {
        // Get current heading
        double currentHeading = navigationSubsystem.getHeading();
        
        // Calculate turn output using PID
        double turnOutput = pidController.calculate(currentHeading);
        
        // Limit output speed for safety
        turnOutput = MathUtil.clamp(turnOutput, -AutoConstants.AUTO_TURN_SPEED, AutoConstants.AUTO_TURN_SPEED);
        
        // Turn in place (no forward movement)
        driveSubsystem.arcadeDrive(0, turnOutput);
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
    
    /**
     * Gets the current error from target angle.
     * @return Error in degrees
     */
    public double getError() {
        return pidController.getPositionError();
    }
}