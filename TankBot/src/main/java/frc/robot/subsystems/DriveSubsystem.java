package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX leftLead = new TalonFX(Constants.Drive.LEFT_PRIMARY_ID);
    private final TalonFX leftFollow = new TalonFX(Constants.Drive.LEFT_SECONDARY_ID);
    private final TalonFX rightLead = new TalonFX(Constants.Drive.RIGHT_PRIMARY_ID);
    private final TalonFX rightFollow = new TalonFX(Constants.Drive.RIGHT_SECONDARY_ID);

    public DriveSubsystem() {
        leftFollow.setControl(Constants.Drive.FOLLOW(leftLead));
        rightFollow.setControl(Constants.Drive.FOLLOW(rightLead));
    }

    public void arcadeDrive(double forward, double rotation) {
        // Apply deadband to eliminate controller drift
        forward = MathUtil.applyDeadband(forward, Constants.Drive.JOYSTICK_DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, Constants.Drive.JOYSTICK_DEADBAND);
        
        double leftOutput = forward + rotation;
        double rightOutput = forward - rotation;
        leftLead.set(leftOutput);
        rightLead.set(rightOutput);
    }

    public void stop() {
        leftLead.set(0);
        rightLead.set(0);
    }
}
