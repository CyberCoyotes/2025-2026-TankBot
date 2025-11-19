package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    // Drive modes for different speed control
    public enum DriveMode {
        NORMAL("Normal", Constants.Drive.NORMAL_SPEED_MULTIPLIER),
        PRECISION("Precision", Constants.Drive.PRECISION_SPEED_MULTIPLIER),
        TURTLE("Turtle", Constants.Drive.TURTLE_SPEED_MULTIPLIER);

        private final String name;
        private final double speedMultiplier;

        DriveMode(String name, double speedMultiplier) {
            this.name = name;
            this.speedMultiplier = speedMultiplier;
        }

        public String getName() { return name; }
        public double getSpeedMultiplier() { return speedMultiplier; }
    }

    // Left side motors (3 Falcon 500s for tank treads)
    private final TalonFX leftLeader = new TalonFX(Constants.Drive.LEFT_LEADER_ID);
    private final TalonFX leftFollower1 = new TalonFX(Constants.Drive.LEFT_FOLLOWER_1_ID);
    private final TalonFX leftFollower2 = new TalonFX(Constants.Drive.LEFT_FOLLOWER_2_ID);

    // Right side motors (3 Falcon 500s for tank treads)
    private final TalonFX rightLeader = new TalonFX(Constants.Drive.RIGHT_LEADER_ID);
    private final TalonFX rightFollower1 = new TalonFX(Constants.Drive.RIGHT_FOLLOWER_1_ID);
    private final TalonFX rightFollower2 = new TalonFX(Constants.Drive.RIGHT_FOLLOWER_2_ID);

    // Pigeon2 Gyro for heading/rotation tracking
    private final Pigeon2 pigeon2 = new Pigeon2(Constants.Drive.PIGEON2_ID);

    // Current drive mode
    private DriveMode currentMode = DriveMode.NORMAL;

    public DriveSubsystem() {
        // Configure left side followers
        leftFollower1.setControl(Constants.Drive.FOLLOW(leftLeader));
        leftFollower2.setControl(Constants.Drive.FOLLOW(leftLeader));

        // Configure right side followers
        rightFollower1.setControl(Constants.Drive.FOLLOW(rightLeader));
        rightFollower2.setControl(Constants.Drive.FOLLOW(rightLeader));

        // Reset gyro to zero on startup
        resetGyro();
    }

    @Override
    public void periodic() {
        // Display current drive mode on SmartDashboard
        SmartDashboard.putString("Drive Mode", currentMode.getName());
        SmartDashboard.putNumber("Speed Multiplier", currentMode.getSpeedMultiplier());

        // Display gyro heading
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Yaw", getYaw());
    }

    /**
     * Arcade drive with speed multiplier based on current mode
     * @param forward Forward/backward speed (-1.0 to 1.0)
     * @param rotation Left/right rotation speed (-1.0 to 1.0)
     */
    public void arcadeDrive(double forward, double rotation) {
        // Apply speed multiplier based on current mode
        double multiplier = currentMode.getSpeedMultiplier();
        forward *= multiplier;
        rotation *= multiplier;

        double leftOutput = forward + rotation;
        double rightOutput = forward - rotation;
        leftLeader.set(leftOutput);
        rightLeader.set(rightOutput);
    }

    /**
     * Set the drive mode (Normal, Precision, or Turtle)
     * @param mode The desired drive mode
     */
    public void setDriveMode(DriveMode mode) {
        currentMode = mode;
        System.out.println("Drive mode changed to: " + mode.getName());
    }

    /**
     * Get the current drive mode
     * @return The current DriveMode
     */
    public DriveMode getDriveMode() {
        return currentMode;
    }

    /**
     * Toggle to the next drive mode (cycles through Normal -> Precision -> Turtle -> Normal)
     */
    public void toggleDriveMode() {
        switch (currentMode) {
            case NORMAL:
                setDriveMode(DriveMode.PRECISION);
                break;
            case PRECISION:
                setDriveMode(DriveMode.TURTLE);
                break;
            case TURTLE:
                setDriveMode(DriveMode.NORMAL);
                break;
        }
    }

    public void stop() {
        leftLeader.set(0);
        rightLeader.set(0);
    }

    // ==================== Gyro Methods ====================

    /**
     * Reset the gyro heading to zero
     * Use this to set the current robot direction as "forward" (0 degrees)
     */
    public void resetGyro() {
        pigeon2.reset();
        System.out.println("Gyro reset to 0 degrees");
    }

    /**
     * Get the current heading in degrees (0-360)
     * @return Heading in degrees, continuous (can be > 360 or < 0)
     */
    public double getHeading() {
        return pigeon2.getYaw().getValueAsDouble();
    }

    /**
     * Get the current yaw angle in degrees
     * @return Yaw in degrees (-180 to 180)
     */
    public double getYaw() {
        return pigeon2.getYaw().getValueAsDouble();
    }

    /**
     * Get the current pitch (tilt forward/backward) in degrees
     * @return Pitch in degrees
     */
    public double getPitch() {
        return pigeon2.getPitch().getValueAsDouble();
    }

    /**
     * Get the current roll (tilt left/right) in degrees
     * @return Roll in degrees
     */
    public double getRoll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    /**
     * Get the rotation rate in degrees per second
     * @return Rotation rate in deg/s
     */
    public double getTurnRate() {
        return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
    }
}
