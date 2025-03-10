// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
public class Constants {
  public static final double TAU = 6.28318530717958647692;
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = TAU; // radians per second

    public static final double kDirectionSlewRate = 4.0; // radians per second
    public static final double kMagnitudeSlewRate = 3.0; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 3.0; // percent per second (1 = 100%)

    // Chassis configuration

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    // drive - turn
    // 1 - 2
    // 3 - 4
    // 5 - 6
    // 7 - 8

    
    boolean isTesting = false;

    
      // public static int[] IDs = {

      //    1 , 11 , 3 , 5 , 2 , 12 , 4 , 6 , 10 , 9 , 7 , 17 , 13 , 15 , 8 , 18 , 14 , 16 

      //   };

      // public static int drivetrainNum = 2; // 1 or 2

      // public static int kFrontLeftDrivingCanId = IDs[(0 + ((drivetrainNum - 1) * 10))];
      // public static int kRearLeftDrivingCanId = IDs[(1 + ((drivetrainNum - 1) * 10))];
      // public static int kFrontRightDrivingCanId = IDs[(2 + ((drivetrainNum - 1) * 10))];
      // public static int kRearRightDrivingCanId = IDs[(3 + ((drivetrainNum - 1) * 10))];

      // public static int kFrontLeftTurningCanId = IDs[(4 + ((drivetrainNum - 1) * 10))];   
      // public static int kRearLeftTurningCanId = IDs[(5 + ((drivetrainNum - 1) * 10))];
      // public static int kFrontRightTurningCanId = IDs[(6 + ((drivetrainNum - 1) * 10))];
      // public static int kRearRightTurningCanId = IDs[(7 + ((drivetrainNum - 1) * 10))];
    
    
    public static int kFrontLeftDrivingCanId = 11;
    public static int kFrontLeftTurningCanId = 12;   

    public static int kRearLeftDrivingCanId = 7;
    public static int kRearLeftTurningCanId = 14;

    public static int kFrontRightDrivingCanId = 3;
    public static int kFrontRightTurningCanId = 4;

    public static int kRearRightDrivingCanId = 13;
    public static int kRearRightTurningCanId = 8;

    
    
    

    public static final boolean kGyroReversed = false;
  
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 990.0 / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = TAU; // radians
    public static final double kTurningEncoderVelocityFactor = TAU / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    //public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    //public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 45; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
