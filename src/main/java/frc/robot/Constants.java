// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final class SwerveConstants{
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 5.25;
        public static final double TURN_MOTOR_GEAR_RATIO = 55.965;
        public static final double DRIVE_MOTOR_PCONVERSION = Math.PI * WHEEL_DIAMETER / (2048.0 * DRIVE_MOTOR_GEAR_RATIO);
        public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
        public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
        public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
        public static final double KP_TURNING = 1;
    
        public static final double DRIVETRAIN_MAX_SPEED = 5.0;
        public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 2 * 2 * Math.PI;

        public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 3.5;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.5;
        public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

        public static final double TRACK_WIDTH = Units.inchesToMeters(13.173279);
        public static final double WHEEL_BASE = Units.inchesToMeters(11.173279);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2),
            new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2),
            new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2),
            new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2)
        );
    }
}
