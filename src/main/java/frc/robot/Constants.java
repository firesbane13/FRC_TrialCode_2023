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
    public final class Robot {
    }

    public final class DriveTrain {
        // 3 meters per second.
        public static final double kMaxSpeed = 3.0;

        // 1/2 rotation per second.
        public static final double kMaxAngularSpeed = Math.PI;

        public static final double kTrackWidth = 0.381 * 2;
        public static final double kWheelRadius = 0.0508;
        public static final int kEncoderResolution = -4096;

        // Motor Controller Ports
        public static final int controller01 = 1;
        public static final int controller02 = 2;
        public static final int controller03 = 3;
        public static final int controller04 = 4;

        // Encoder Channels
        public static final int encoderChannel00 = 0;
        public static final int encoderChannel01 = 1;
        public static final int encoderChannel02 = 2;
        public static final int encoderChannel03 = 3;

        // Gryo
        public static final int gyro00 = 0;
    }
}
