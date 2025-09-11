/*
 * Enhanced drive command with improved controls and deadband handling.
 * The left thumbstick controls forward and backward movement.
 * The right thumbstick controls rotation of the robot.
 * Includes exponential scaling for finer control at low speeds.
 */

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
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
        // Get joystick inputs (inverted Y axis for forward/backward)
        double drivePower = -joystick.getLeftY();
        double turnPower = joystick.getRightX();

        // Apply deadband using WPILib utility
        drivePower = MathUtil.applyDeadband(drivePower, DriveConstants.JOYSTICK_DEADBAND);
        turnPower = MathUtil.applyDeadband(turnPower, DriveConstants.JOYSTICK_DEADBAND);

        // Apply exponential scaling for finer control
        drivePower = Math.copySign(Math.pow(Math.abs(drivePower), DriveConstants.DRIVE_EXPO), drivePower);
        turnPower = Math.copySign(Math.pow(Math.abs(turnPower), DriveConstants.TURN_EXPO), turnPower);

        // Drive the robot using arcade drive
        subsystem.arcadeDrive(drivePower, turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs indefinitely
    }
}