// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 9;
    public static final int kFrontLeftTurningMotorPort = 8;

    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 6;

    public static final int kRearLeftDriveMotorPort = 5;
    public static final int kRearLeftTurningMotorPort = 4;

    public static final int kRearRightDriveMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 2;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] { 3, 2 };
    public static final int[] kRearLeftTurningEncoderPorts = new int[] { 5, 4 };

    public static final int[] kFrontRightTurningEncoderPorts = new int[] { 9, 8 };
    public static final int[] kRearRightTurningEncoderPorts = new int[] { 7, 6 };

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftMotorIsVictor = true;
    public static final boolean kRearLeftMotorIsVictor = false;
    public static final boolean kFrontRightMotorIsVictor = true;
    public static final boolean kRearRightMotorIsVictor = false;

    public static final boolean kFrontLeftDriveInvert = true;
    public static final boolean kRearLeftDriveInvert = true;
    public static final boolean kFrontRightDriveInvert = false;
    public static final boolean kRearRightDriveInvert = false;

    public static final double wheelDistFromCenter = 13.5;

    public static final double kWheelRadius = 5.4;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 3.52;

  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 414;
    public static final double kWheelDiameterMeters = 0.095;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kModuleTurningP = 0.2;
    public static final double kModuleTurningI = 0.3;
    public static final double kModuleTurningD = 0;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.52;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
