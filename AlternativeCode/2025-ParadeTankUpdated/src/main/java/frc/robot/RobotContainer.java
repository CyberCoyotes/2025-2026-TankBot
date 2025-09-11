// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AprilTagTrackingCommand;
import frc.robot.commands.DriveCommand2;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.FieldNavigationCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DriveSubsystem2;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    
    // Subsystems
    private final DriveSubsystem2 driveSubsystem = new DriveSubsystem2();
    private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    
    // Autonomous chooser
    private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    public RobotContainer() {
        // Configure the default commands
        configureDefaultCommands();
        
        // Configure button bindings
        configureBindings();
        
        // Configure autonomous options
        configureAutonomous();
    }
    
    /**
     * Configures the default commands for subsystems.
     */
    private void configureDefaultCommands() {
        // Set default drive command
        driveSubsystem.setDefaultCommand(new DriveCommand2(driveSubsystem, driverController));
        
        // Navigation subsystem handles its own periodic updates
        // Vision subsystem handles its own periodic updates
    }

    /**
     * Configures button bindings for manual control.
     */
    private void configureBindings() {
        // Driver controller bindings
        new JoystickButton(driverController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(navigationSubsystem::resetHeading));
            
        new JoystickButton(driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(driveSubsystem::resetEncoders));
            
        new JoystickButton(driverController, XboxController.Button.kX.value)
            .whileTrue(new AutoBalanceCommand(driveSubsystem, navigationSubsystem));
            
        new JoystickButton(driverController, XboxController.Button.kY.value)
            .whileTrue(new AutoAimCommand(driveSubsystem, visionSubsystem));
            
        // Left and right bumpers for precise turns
        new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 
                       navigationSubsystem.getHeading() - 90.0));
                       
        new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 
                       navigationSubsystem.getHeading() + 90.0));
                       
        // Start button for AprilTag tracking
        new JoystickButton(driverController, XboxController.Button.kStart.value)
            .whileTrue(new AprilTagTrackingCommand(driveSubsystem, visionSubsystem, 1.5));
        
        // Operator controller bindings
        new JoystickButton(operatorController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(() -> visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.ON)));
            
        new JoystickButton(operatorController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> visionSubsystem.setLEDMode(VisionSubsystem.LEDMode.OFF)));
            
        new JoystickButton(operatorController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> visionSubsystem.setCameraMode(VisionSubsystem.CameraMode.VISION)));
            
        new JoystickButton(operatorController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> visionSubsystem.setCameraMode(VisionSubsystem.CameraMode.DRIVER)));
            
        new JoystickButton(operatorController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(visionSubsystem::takeSnapshot));
    }
    
    /**
     * Configures autonomous command options.
     */
    private void configureAutonomous() {
        // Create autonomous commands
        Command driveForward2m = new DriveDistanceCommand(driveSubsystem, 2.0);
        Command driveBackward1m = new DriveDistanceCommand(driveSubsystem, -1.0);
        Command turn90Left = new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 90.0);
        Command turn90Right = new TurnToAngleCommand(driveSubsystem, navigationSubsystem, -90.0);
        Command turn180 = new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 180.0);
        Command autoBalance = new AutoBalanceCommand(driveSubsystem, navigationSubsystem);
        
        // Complex autonomous sequences
        Command driveAndBalance = Commands.sequence(
            new DriveDistanceCommand(driveSubsystem, 3.0),
            new WaitCommand(0.5),
            new AutoBalanceCommand(driveSubsystem, navigationSubsystem)
        );
        
        Command squarePattern = Commands.sequence(
            new DriveDistanceCommand(driveSubsystem, 2.0),
            new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 90.0),
            new DriveDistanceCommand(driveSubsystem, 2.0),
            new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 180.0),
            new DriveDistanceCommand(driveSubsystem, 2.0),
            new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 270.0),
            new DriveDistanceCommand(driveSubsystem, 2.0),
            new TurnToAngleCommand(driveSubsystem, navigationSubsystem, 0.0)
        );
        
        Command visionAim = Commands.sequence(
            new AutoAimCommand(driveSubsystem, visionSubsystem),
            new WaitCommand(1.0)
        );
        
        Command aprilTagTrack = new AprilTagTrackingCommand(driveSubsystem, visionSubsystem, 2.0);
        
        // Get pre-built complex sequences
        Command driveAndBalance = FieldNavigationCommand.driveAndBalance(driveSubsystem, navigationSubsystem);
        Command visionAutonomous = FieldNavigationCommand.visionAutonomous(driveSubsystem, navigationSubsystem, visionSubsystem);
        Command complexNavigation = FieldNavigationCommand.complexNavigation(driveSubsystem, navigationSubsystem, visionSubsystem);
        Command defensiveAuto = FieldNavigationCommand.defensiveAutonomous(driveSubsystem, navigationSubsystem);
        Command mobilityAuto = FieldNavigationCommand.mobilityAutonomous(driveSubsystem);
        Command systemTest = FieldNavigationCommand.systemTest(driveSubsystem, navigationSubsystem, visionSubsystem);
        
        // Add options to chooser
        autonomousChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
        autonomousChooser.addOption("Drive Forward 2m", driveForward2m);
        autonomousChooser.addOption("Drive Backward 1m", driveBackward1m);
        autonomousChooser.addOption("Turn 90° Left", turn90Left);
        autonomousChooser.addOption("Turn 90° Right", turn90Right);
        autonomousChooser.addOption("Turn 180°", turn180);
        autonomousChooser.addOption("Auto Balance", autoBalance);
        autonomousChooser.addOption("Drive and Balance", driveAndBalance);
        autonomousChooser.addOption("Square Pattern", squarePattern);
        autonomousChooser.addOption("Vision Aim Test", visionAim);
        autonomousChooser.addOption("AprilTag Tracking", aprilTagTrack);
        autonomousChooser.addOption("Vision Autonomous", visionAutonomous);
        autonomousChooser.addOption("Complex Navigation", complexNavigation);
        autonomousChooser.addOption("Defensive Auto", defensiveAuto);
        autonomousChooser.addOption("Mobility Auto", mobilityAuto);
        autonomousChooser.addOption("System Test", systemTest);
        
        // Put chooser on dashboard
        SmartDashboard.putData("Auto Chooser", autonomousChooser);
        
        // Put individual commands on dashboard for testing
        SmartDashboard.putData("Reset Heading", new InstantCommand(navigationSubsystem::resetHeading));
        SmartDashboard.putData("Reset Encoders", new InstantCommand(driveSubsystem::resetEncoders));
        SmartDashboard.putData("Auto Balance", autoBalance);
        SmartDashboard.putData("Auto Aim", new AutoAimCommand(driveSubsystem, visionSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
    
    /**
     * Gets the drive subsystem for use in other classes if needed.
     * @return The drive subsystem
     */
    public DriveSubsystem2 getDriveSubsystem() {
        return driveSubsystem;
    }
    
    /**
     * Gets the navigation subsystem for use in other classes if needed.
     * @return The navigation subsystem
     */
    public NavigationSubsystem getNavigationSubsystem() {
        return navigationSubsystem;
    }
    
    /**
     * Gets the vision subsystem for use in other classes if needed.
     * @return The vision subsystem
     */
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }
    
    /**
     * Updates navigation odometry with drive subsystem data.
     * This should be called from Robot.robotPeriodic().
     */
    public void updateOdometry() {
        navigationSubsystem.updateOdometry(
            driveSubsystem.getLeftDistanceMeters(),
            driveSubsystem.getRightDistanceMeters()
        );
    }
}
