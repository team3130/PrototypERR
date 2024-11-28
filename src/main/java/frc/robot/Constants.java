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
  public static final boolean debugMode = false;
  public static final boolean pitMode = false; 

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
