// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Drive {
    // Left side motors (3 Falcon 500s)
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_1_ID = 2;
    public static final int LEFT_FOLLOWER_2_ID = 3;

    // Right side motors (3 Falcon 500s)
    public static final int RIGHT_LEADER_ID = 4;
    public static final int RIGHT_FOLLOWER_1_ID = 5;
    public static final int RIGHT_FOLLOWER_2_ID = 6;

    // Drive mode speed multipliers
    public static final double NORMAL_SPEED_MULTIPLIER = 1.0;    // 100% speed
    public static final double PRECISION_SPEED_MULTIPLIER = 0.5;  // 50% speed for fine control
    public static final double TURTLE_SPEED_MULTIPLIER = 0.25;    // 25% speed for very slow movement

    // Helper for TalonFX follower setup (Phoenix6 API)
    public static Follower FOLLOW(TalonFX leader) {
      return new Follower(leader.getDeviceID(), false);
    }
  }

}
