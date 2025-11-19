package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    // Left side motors (3 Falcon 500s for tank treads)
    private final TalonFX leftLeader = new TalonFX(Constants.Drive.LEFT_LEADER_ID);
    private final TalonFX leftFollower1 = new TalonFX(Constants.Drive.LEFT_FOLLOWER_1_ID);
    private final TalonFX leftFollower2 = new TalonFX(Constants.Drive.LEFT_FOLLOWER_2_ID);

    // Right side motors (3 Falcon 500s for tank treads)
    private final TalonFX rightLeader = new TalonFX(Constants.Drive.RIGHT_LEADER_ID);
    private final TalonFX rightFollower1 = new TalonFX(Constants.Drive.RIGHT_FOLLOWER_1_ID);
    private final TalonFX rightFollower2 = new TalonFX(Constants.Drive.RIGHT_FOLLOWER_2_ID);

    public DriveSubsystem() {
        // Configure left side followers
        leftFollower1.setControl(Constants.Drive.FOLLOW(leftLeader));
        leftFollower2.setControl(Constants.Drive.FOLLOW(leftLeader));

        // Configure right side followers
        rightFollower1.setControl(Constants.Drive.FOLLOW(rightLeader));
        rightFollower2.setControl(Constants.Drive.FOLLOW(rightLeader));
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
