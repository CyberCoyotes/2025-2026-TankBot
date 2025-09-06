// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    static final int driver_controller_port = 0;

    public static final int RIGHT_LEADER_ID = 1;    // Right Leader Motor ID - Talon for SIM Motor
    public static final int RIGHT_FOLLOWER_ID = 2;  // Right Follower Motor ID - Talon for SIM Motor
    public static final int LEFT_LEADER_ID = 3;    // Left Leader Motor ID - Talon for SIM Motor
    public static final int LEFT_FOLLOWER_ID = 4;   // Left Follower Motor ID - Talon for SIM Motor
    public static final int CENTER_WHEEL_ID = 5;    // Center Wheel Motor ID - Talon
}
