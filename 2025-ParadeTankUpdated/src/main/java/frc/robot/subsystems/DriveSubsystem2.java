/********************** 
Subsystem for the drive train. 
This is the second version of the drive train subsystem. 
It should setup a right side and left side with two motors each.
It is using SIM motors and Talon SRX controllers. 
The right side Leader motor and left side follow motor should not inverted.
The left side Leader motor and right side follow motor should be inverted.
The left thumbstick will control the forward and backward movement of the robot.
The right thumbstick will control the rotation of the robot.
***********************/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents the drive subsystem of the robot.
 * This subsystem controls the robot's drive motors and allows it to move and rotate.
 */
public class DriveSubsystem2 extends SubsystemBase {
    private final DifferentialDrive drive;
    private final TalonFX rightLeaderMotor;
    private final TalonFX rightFollowerMotor;
    private final TalonFX leftLeaderMotor;
    private final TalonFX leftFollowerMotor;

    public DriveSubsystem2() {
        rightLeaderMotor = new TalonFX(Constants.RIGHT_LEADER_ID);
        rightFollowerMotor = new TalonFX(Constants.RIGHT_FOLLOWER_ID);
        leftLeaderMotor = new TalonFX(Constants.LEFT_LEADER_ID);
        leftFollowerMotor = new TalonFX(Constants.LEFT_FOLLOWER_ID);

        rightLeaderMotor.setInverted(true);
        rightFollowerMotor.setInverted(false);
        leftLeaderMotor.setInverted(false);
        leftFollowerMotor.setInverted(true);

        // rightLeaderMotor.setNeutralMode(NeutralMode.Coast);
        // rightFollowerMotor.setNeutralMode(NeutralMode.Coast);
        // leftLeaderMotor.setNeutralMode(NeutralMode.Coast);
        // leftFollowerMotor.setNeutralMode(NeutralMode.Coast);

     
        // rightFollowerMotor.follow(rightLeaderMotor);
        rightFollowerMotor.setControl(new Follower(rightLeaderMotor.getDeviceID(), false));
        leftFollowerMotor.setControl(new Follower(leftLeaderMotor.getDeviceID(), false));
        // leftFollowerMotor.follow(leftLeaderMotor);

        drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);
    }

    public void drive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}