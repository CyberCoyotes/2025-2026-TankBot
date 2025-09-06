/*
 * This command will drive the robot using the drive and turn power values from the joysticks.
 * The left thumbstick will control the forward and backward movement of the robot.
 * The right thumbstick will control the rotation of the robot. 
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem2;

public class DriveCommand2 extends Command {

    private final DriveSubsystem2 subsystem;
    private final XboxController joystick;

    public DriveCommand2(DriveSubsystem2 subsystem, XboxController driveController) {
        this.subsystem = subsystem;
        this.joystick = driveController;
        addRequirements(this.subsystem);
    }

    @Override
    public void execute() {
        // Apply a deadband to filter out small, unintended joystick movements
        double drivePower = applyDeadband(-joystick.getRawAxis(1), 0.1);
        double turnPower = applyDeadband(joystick.getRawAxis(4), 0.1);

        // Optional: Apply exponential scaling for finer control
        drivePower = Math.copySign(Math.pow(Math.abs(drivePower), 2), drivePower);
        turnPower = Math.copySign(Math.pow(Math.abs(turnPower), 2), turnPower);

        subsystem.drive(drivePower, turnPower);
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }

}