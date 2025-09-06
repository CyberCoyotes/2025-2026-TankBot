// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand2;
import frc.robot.subsystems.DriveSubsystem2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private XboxController driveController = new XboxController(Constants.driver_controller_port);
  

  private final DriveSubsystem2 m_driveSubsystem = new DriveSubsystem2();


 
  public RobotContainer() {
    
  
    m_driveSubsystem.setDefaultCommand(new DriveCommand2(m_driveSubsystem, driveController));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }
}
