package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    /**
     * The DifferentialDrive class represents a drivetrain with two motors, allowing for differential steering.
     * It provides methods for controlling the drivetrain's movement and rotation.
     */
    private final DifferentialDrive drive;
    private final Talon rightLeaderDrive;
    private final Talon rightFollowerDrive;
    private final Talon leftLeaderDrive;
    private final Talon leftFollowerDrive;
    private final Talon[] motors;

    public DriveSubsystem() {
        rightLeaderDrive = new Talon(Constants.RIGHT_LEADER_ID);
        rightFollowerDrive = new Talon(Constants.RIGHT_FOLLOWER_ID);
        leftLeaderDrive = new Talon(Constants.LEFT_LEADER_ID);
        leftFollowerDrive = new Talon(Constants.LEFT_FOLLOWER_ID);

        
        motors = new Talon[]{
            rightLeaderDrive, 
            rightFollowerDrive,
            leftLeaderDrive,
            leftFollowerDrive
        };
        

        configureMotors();
        drive = new DifferentialDrive(rightLeaderDrive, leftLeaderDrive);
    }

    /* These were set in Tuner X */
    // Comment out for now
    private void configureMotors() {
        rightLeaderDrive.setInverted(true);
        rightFollowerDrive.setInverted(false);
        leftLeaderDrive.setInverted(false);
        leftFollowerDrive.setInverted(true);

        /*
        for (TalonFX motor : motors) {
            motor.configFactoryDefault();
        }
        */

        // rightFollowerDrive.set(TalonSRXControlMode.Follower, rightLeaderDrive.getDeviceID());
        // leftFollowerDrive.set(TalonSRXControlMode.Follower, leftLeaderDrive.getDeviceID());
    }

    public void drive(double drivePower, double turnPower) {
        drive.arcadeDrive(drivePower, turnPower);
    }
}