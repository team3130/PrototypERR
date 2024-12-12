// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean debugMode = false;
  public static final boolean pitMode = false;

  public static class CAN {
    public static final int PCM = 0;
    public static final int PDP = 1;

    public static final int Talon1 = 1;
    public static final int Talon2 = 2;
    public static final int Talon3 = 3;
    public static final int Victor4 = 4;
    public static final int Talon5 = 5;
  }

  public static class Swerve {
    public static double maxSteerVoltage = 4d;
    public static double maxDriveVoltage = 10d;

    public static double tuningDesiredVelocity = 2d;



    public static double slot0_kS = 0; // DONT USE KS
    public static double slot0_kV = 0; // measured 3/19/24
    public static double slot0_kP = 0; // measured 3/14/24
    public static double slot0_kI = 0;
    public static double slot0_kD = 0;


    // SWERVE CAN NUMBERED LIKE CARTESIAN COORDIANTE QUADRANTS (dont think this is true anymore ngl)
    // front right
    public final static int MOD_ONE_STEER = 6;
    public final static int MOD_ONE_DRIVE = 7;

    // back right
    public final static int MOD_TWO_STEER = 9;
    public final static int MOD_TWO_DRIVE = 8;

    // back left
    public final static int MOD_THREE_STEER = 11;
    public final static int MOD_THREE_DRIVE = 10;

    public final static int MOD_FOUR_STEER = 13;
    public final static int MOD_FOUR_DRIVE = 12;
    // front left
    public final static int MOD_ONE_CANCODER = 1;
    public final static int MOD_TWO_CANCODER = 2;
    public final static int MOD_THREE_CANCODER = 3;
    public final static int MOD_FOUR_CANCODER = 4;

    // Order should match side
    public static final int[] turningID = new int[] {MOD_ONE_STEER, MOD_TWO_STEER, MOD_THREE_STEER, MOD_FOUR_STEER};
    public static final int[] spinningID = new int[] {MOD_ONE_DRIVE, MOD_TWO_DRIVE, MOD_THREE_DRIVE, MOD_FOUR_DRIVE};
    public final static int[] CANCoders = new int[] {MOD_ONE_CANCODER, MOD_TWO_CANCODER, MOD_THREE_CANCODER, MOD_FOUR_CANCODER};

    /* Length and width as measured as distances between center of wheels */
    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
    public static final double trackWidth = 0.629;
    // the front-to-back distance between the drivetrain wheels, should be measured from center to center
    public static final double wheelBase = 0.629;

    public static final Translation2d[] moduleTranslations = {
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // 1 pos neg
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0) // 4, pos pos
    };

    // / 3.54 with 8 volts of voltage compensation and 4.19 with 10 volts
    // 4.8 max speed, 5 acceleration, drops to 9.6
    public final static double kPhysicalMaxSpeedMetersPerSecond = 1;
    public final static double kDeadband = 0.055;
    public final static double kMaxAccelerationDrive = 1.25;
    public final static double kMaxAccelerationAngularDrive = 1.0*Math.PI;

    public final static double kP_FrontRight = 0;
    public final static double kI_FrontRight = 0;
    public final static double kD_FrontRight = 0;
    public final static double kF_FrontRight = 0;

    public final static double kP_FrontLeft = 0;
    public final static double kI_FrontLeft = 0;
    public final static double kD_FrontLeft = 0;
    public final static double kF_FrontLeft = 0;

    public final static double kP_BackLeft = 0;
    public final static double kI_BackLeft = 0;
    public final static double kD_BackLeft = 0;
    public final static double kF_BackLeft = 0;

    public final static double kP_BackRight = 0;
    public final static double kI_BackRight = 0;
    public final static double kD_BackRight = 0;
    public final static double kF_BackRight = 0;
    public final static double[] kP_Swerve = new double[] {kP_FrontLeft, kP_BackLeft, kP_FrontRight, kP_BackRight};
    public final static double[] kI_Swerve = new double[] {kI_FrontLeft, kI_BackLeft, kI_FrontRight, kI_BackRight};
    public final static double[] kD_Swerve = new double[] {kD_FrontLeft, kD_BackLeft, kD_FrontRight, kD_BackRight};
    public final static double[] kF_Swerve = new double[] {kF_FrontLeft, kF_BackLeft, kF_FrontRight, kF_BackRight};
  }

  public static class SwerveModules {
    public static final int one = 0;
    public static final int two = 1;
    public static final int three = 2;
    public static final int four = 3;
  }

  public static class SwerveEncoderOffsets {
    public static final double MOD_ONE_OFFSET = 0.111084;
    public static final double MOD_TWO_OFFSET = 0.068604;
    public static final double MOD_THREE_OFFSET = -0.378174;
    public static final double MOD_FOUR_OFFSET = 0.427979;
    public static final double[] kCANCoderOffsets = new double[] {MOD_ONE_OFFSET, MOD_TWO_OFFSET, MOD_THREE_OFFSET, MOD_FOUR_OFFSET};
  }

  // gear ratios and/or ticks per rev, etc.
  public static class SwerveConversions {
    public final static double driveGearRatio = 6.75; // Checked 2/2/24 //6.75  checked 1/19/23
    public final static double steerGearRatio = 21.42857; // Checked 2/2/24 //150d/7d = 21.42857  checked 1/19
    public static final double wheelDiameter = Units.inchesToMeters(3.9);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public final static double driveRotToMeters = wheelDiameter * Math.PI * (1/(driveGearRatio)); // multiply by
    public static final double steerRotToRads = 1/(steerGearRatio) * Math.PI * 2; // multiply by position
    public static final double driveRotToMetersPerSecond = driveRotToMeters * 10; // multiply by velocity
    public static final double steerRotToRadsPerSecond = steerRotToRads * 10; // multiply by velocity
  }

  public static class PS5 {
    public static final int BTN_SQUARE = 1;
    public static final int BTN_X = 2;
    public static final int BTN_CIRCLE = 3;
    public static final int BTN_TRIANGLE = 4;
    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;
  
    public static final int BTN_LJOYSTICK_PRESS = 11;
    public static final int BTN_RJOYSTICK_PRESS = 12;
  
    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;
  
    // Gamepad Axis List
    public static final int AXS_LJOYSTICKX = 0;
    public static final int AXS_LJOYSTICKY = 1;
    public static final int AXS_LTRIGGER = 3;
    public static final int AXS_RTRIGGER = 4;
    public static final int AXS_RJOYSTICK_X = 2;
    public static final int AXS_RJOYSTICK_Y = 5;
    }
  
  public static class XBox {
    // Gamepad Button List
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
  
    public static final int BTN_LBUMPER = 5;
    public static final int BTN_RBUMPER = 6;
    public static final int BTN_WINDOW = 7;
    public static final int BTN_MENU = 8;
    public static final int BTN_LJOYSTICK_PRESS = 9;
    public static final int BTN_RJOYSTICK_PRESS = 10;
  
    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;
  
    // Gamepad Axis List
    public static final int AXS_LJOYSTICK_X = 0;
    public static final int AXS_LJOYSTICK_Y = 1;
    public static final int AXS_LTRIGGER = 2;
    public static final int AXS_RTRIGGER = 3;
    public static final int AXS_RJOYSTICK_X = 4;
    public static final int AXS_RJOYSTICK_Y = 5;
  }
  public static final boolean navxReversed = false;
}
