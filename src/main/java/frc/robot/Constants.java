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
  public static class DriveTrain {
    public static final int canMotorControllerPort00 = 9;
    public static final int canMotorControllerPort01 = 10;
    public static final int canMotorControllerPort02 = 11;
    public static final int canMotorControllerPort03 = 12;

    public static final double kMaxSpeed = 3.0;
    public static final boolean speedLimiterEnabled = true;

    /******
     * PID
     */
    public static final double kP = 5e-5;
    public static final double kI = 1e-6;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;

    /******
     * Self Drive Train
     */
    // Used to low levels of input from the joysticks
    public static final double kDefaultDeadband = 0.02;

    // Used to change max value if needed.
    public static final double kDefaultMaxOutput = 1.0;

    /******
     * Auto-balance Constants
     */
    public static final double wheelDiameter = 8.0;
    public static final double platformWidth = 48.0;
    public static final double robotLength = 28.0;
    public static final double minAngle = 0.0;
    public static final double maxAngle = 15.0;
    public static final double minMovementSpeed = 0.3;
    public static final double maxMovementSpeed = 0.7;
    public static final double brakeAdjustment = 0.5;
  }

  public static class Joysticks {
    public static final int driveControllerLeftUsbPort = 0;
    public static final int driveControllerRightUsbPort = 1;
    public static final int secondaryDriveControlerUsbPort = 2;

    /********************************
     * Driver Left
     ********************************/
    public static class DriverLeft {
      public static final int trigger = 1;
      public static final int button02 = 2;
      public static final int button03 = 3;
      public static final int button04 = 4;
      public static final int button05 = 5;
      public static final int button06 = 6;
      public static final int button07 = 7;
      public static final int button08 = 8;
      public static final int button09 = 9;
      public static final int button10 = 10;
      public static final int button11 = 11;

      public static final int rateLimit = 3;
    }

    /********************************
     * Driver Right
     ********************************/
    public static class DriverRight {
      public static final int trigger = 1;
      public static final int button02 = 2;
      public static final int button03 = 3;
      public static final int button04 = 4;
      public static final int button05 = 5;
      public static final int button06 = 6;
      public static final int button07 = 7;
      public static final int button08 = 8;
      public static final int button09 = 9;
      public static final int button10 = 10;
      public static final int button11 = 11;

      public static final int rateLimit = 3;
    }

    /********************************
     * Secondary Driver
     ********************************/
    public static class SecondaryDriver {
      public static final int trigger = 1;
      public static final int button02 = 2;
      public static final int button03 = 3;
      public static final int button04 = 4;
      public static final int button05 = 5;
      public static final int button06 = 6;
      public static final int autoBalanceForce = 7;
      public static final int autoBalanceDisable = 8;
      public static final int button09 = 9;
      public static final int button10 = 10;
      public static final int button11 = 11;
      public static final int button12 = 12;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Robot {
    
  }
}
