package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX leftLeader = new TalonFX(Constants.Drive.LEFT_LEADER_ID);
    private final TalonFX leftFollower = new TalonFX(Constants.Drive.LEFT_FOLLOWER_ID);
    private final TalonFX rightLeader = new TalonFX(Constants.Drive.RIGHT_LEADER_ID);
    private final TalonFX rightFollower = new TalonFX(Constants.Drive.RIGHT_FOLLOWER_ID);

    public DriveSubsystem() {
        leftFollower.setControl(Constants.Drive.FOLLOW(leftLeader));
        rightFollower.setControl(Constants.Drive.FOLLOW(rightLeader));
    }

    public void arcadeDrive(double forward, double rotation) {
        double leftOutput = forward + rotation;
        double rightOutput = forward - rotation;
        leftLeader.set(leftOutput);
        rightLeader.set(rightOutput);
    }

    public void stop() {
        leftLeader.set(0);
        rightLeader.set(0);
    }
}
