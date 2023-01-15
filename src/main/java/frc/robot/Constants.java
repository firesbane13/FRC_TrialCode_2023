// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

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
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;
    public static final double kTrackWidth = 0.381 * 2;
    public static final double kWheelRadius = 0.0500;
    public static final int kEncoderResolution = -4096;

    public static final double staticGain = 1;
    public static final double velocityGain = 3;

    public static final double kVLinear = 1.98;
    public static final double kALinear = 0.2;
    public static final double kVAngular = 1.5;
    public static final double kAAngular = 0.3;

    public static final int numOfMotors = 4;
    public static final int gearing = 8;
    public static final Matrix<N7, N1> measurementStdDevs = null;
  }

  public static class Simulation {
    public static final int leftEncoderChannelA = 0;
    public static final int leftEncoderChannelB = 1;
    public static final int rightEncoderChannelA = 2;
    public static final int rightEncoderChannelB = 3;

    public static final int gyroPort = 0;
  }
}
