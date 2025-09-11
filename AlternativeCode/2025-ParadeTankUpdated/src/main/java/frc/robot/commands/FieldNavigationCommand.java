package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Comprehensive autonomous command that demonstrates full robot capabilities.
 * Combines navigation, vision tracking, and balance operations.
 */
public class FieldNavigationCommand {
    
    /**
     * Creates a simple autonomous sequence: drive forward and balance.
     * @param drive Drive subsystem
     * @param navigation Navigation subsystem
     * @return Command sequence
     */
    public static Command driveAndBalance(DriveSubsystem2 drive, NavigationSubsystem navigation) {
        return Commands.sequence(
            // Reset systems
            Commands.runOnce(() -> {
                drive.resetEncoders();
                navigation.resetHeading();
            }),
            
            // Drive forward to charging station
            new DriveDistanceCommand(drive, 3.0),
            
            // Wait briefly
            new WaitCommand(0.5),
            
            // Auto-balance
            new AutoBalanceCommand(drive, navigation)
        ).withName("Drive and Balance");
    }
    
    /**
     * Creates a vision-based autonomous sequence.
     * @param drive Drive subsystem
     * @param navigation Navigation subsystem  
     * @param vision Vision subsystem
     * @return Command sequence
     */
    public static Command visionAutonomous(DriveSubsystem2 drive, 
                                          NavigationSubsystem navigation,
                                          VisionSubsystem vision) {
        return Commands.sequence(
            // Initialize
            Commands.runOnce(() -> {
                drive.resetEncoders();
                navigation.resetHeading();
                vision.setLEDMode(VisionSubsystem.LEDMode.ON);
            }),
            
            // Search and aim at target
            new AutoAimCommand(drive, vision).withTimeout(5.0),
            
            // Drive to optimal distance
            new AprilTagTrackingCommand(drive, vision, 2.0).withTimeout(8.0),
            
            // Take final snapshot
            Commands.runOnce(vision::takeSnapshot),
            
            // Wait and finish
            new WaitCommand(1.0)
        ).withName("Vision Autonomous");
    }
    
    /**
     * Creates a complex navigation pattern demonstrating multiple capabilities.
     * @param drive Drive subsystem
     * @param navigation Navigation subsystem
     * @param vision Vision subsystem
     * @return Command sequence
     */
    public static Command complexNavigation(DriveSubsystem2 drive,
                                           NavigationSubsystem navigation,
                                           VisionSubsystem vision) {
        return Commands.sequence(
            // Initialize systems
            Commands.runOnce(() -> {
                drive.resetEncoders();
                navigation.resetHeading();
            }),
            
            // Phase 1: Basic navigation
            Commands.parallel(
                Commands.sequence(
                    new DriveDistanceCommand(drive, 2.0),
                    new TurnToAngleCommand(drive, navigation, 90.0),
                    new DriveDistanceCommand(drive, 1.5),
                    new TurnToAngleCommand(drive, navigation, 0.0)
                )
            ),
            
            // Phase 2: Vision targeting
            Commands.race(
                new AprilTagTrackingCommand(drive, vision, 1.5),
                new WaitCommand(10.0) // Timeout
            ),
            
            // Phase 3: Return and balance
            Commands.sequence(
                new TurnToAngleCommand(drive, navigation, 180.0),
                new DriveDistanceCommand(drive, 2.0),
                new AutoBalanceCommand(drive, navigation)
            )
        ).withName("Complex Navigation");
    }
    
    /**
     * Creates a defensive autonomous that focuses on positioning.
     * @param drive Drive subsystem
     * @param navigation Navigation subsystem
     * @return Command sequence
     */
    public static Command defensiveAutonomous(DriveSubsystem2 drive, NavigationSubsystem navigation) {
        return Commands.sequence(
            // Leave starting area
            new DriveDistanceCommand(drive, 1.0),
            
            // Position defensively
            new TurnToAngleCommand(drive, navigation, 180.0),
            
            // Wait out the autonomous period
            new WaitCommand(12.0)
        ).withName("Defensive Autonomous");
    }
    
    /**
     * Creates a mobility autonomous (just drive out of community).
     * @param drive Drive subsystem
     * @return Command sequence
     */
    public static Command mobilityAutonomous(DriveSubsystem2 drive) {
        return Commands.sequence(
            Commands.runOnce(drive::resetEncoders),
            new DriveDistanceCommand(drive, 4.0)
        ).withName("Mobility Autonomous");
    }
    
    /**
     * Creates a test sequence for validating robot systems.
     * @param drive Drive subsystem
     * @param navigation Navigation subsystem
     * @param vision Vision subsystem
     * @return Command sequence
     */
    public static Command systemTest(DriveSubsystem2 drive,
                                    NavigationSubsystem navigation,
                                    VisionSubsystem vision) {
        return Commands.sequence(
            // Test drive system
            Commands.runOnce(() -> System.out.println("Testing drive system...")),
            new DriveDistanceCommand(drive, 0.5),
            new WaitCommand(0.5),
            new DriveDistanceCommand(drive, -0.5),
            
            // Test navigation system
            Commands.runOnce(() -> System.out.println("Testing navigation system...")),
            new TurnToAngleCommand(drive, navigation, 90.0),
            new WaitCommand(0.5),
            new TurnToAngleCommand(drive, navigation, 0.0),
            
            // Test vision system
            Commands.runOnce(() -> System.out.println("Testing vision system...")),
            Commands.runOnce(() -> vision.setLEDMode(VisionSubsystem.LEDMode.ON)),
            new WaitCommand(2.0),
            Commands.runOnce(() -> vision.setLEDMode(VisionSubsystem.LEDMode.OFF)),
            
            Commands.runOnce(() -> System.out.println("System test complete!"))
        ).withName("System Test");
    }
}