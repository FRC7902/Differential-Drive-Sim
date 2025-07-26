// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // TODO: Insert DriveConstants here...
  public static class DriveConstants {
  }

  public static class ArmConstants {
    public final static int canID = 1;
    public final static double gearRatio = 100; //100:1 gear ratio
    public final static double kP = 10;
    public final static double kI = 0;
    public final static double kD = 0;
    public final static double maxVelocity = 1; // rad/s
    public final static double maxAcceleration = 1; // rad/s²
    public final static boolean brakeMode = true;
    public final static boolean enableStatorLimit = true;
    public final static double statorCurrentLimit = 40;
    public final static boolean enableSupplyLimit = false;
    public final static double supplyCurrentLimit = 40;
    public final static double armLength = 0.5; // meters
    public final static double minAngleDeg = 0;
    public final static double maxAngleDeg = 90;
    public final static double kS = 0;
    public final static double kG = 0;
    public final static double kA = 0;
    public final static double kV = 0;
  } 
}
