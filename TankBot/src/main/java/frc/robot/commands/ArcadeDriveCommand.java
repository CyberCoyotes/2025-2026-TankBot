package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class ArcadeDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;

    public ArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(forwardSupplier.getAsDouble(), rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
