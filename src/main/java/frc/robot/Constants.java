// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
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

/*
CAN ID's
FrontLeftDrivingCanId = 10
RearLeftDrivingCanId = 8
FrontRightDrivingCanId = 2
RearRightDrivingCanId = 4

FrontLeftTurningCanId = 11
RearLeftTurningCanId = 9
FrontRightTurningCanId = 3
RearRightTurningCanId = 5

intake = 13
conveyer = 14
top shooter = 15
bottom shooter = 16

 */
public final class Constants {
    public static final class DefaultConfig {
        public static final boolean isCone = false;

        public static final double rampRate = 2;
        public static final double stickDeadband = 0.1;

        public static final double lowSpeedMultiplier = 0.2;

        public static final boolean fieldRelative = true;

        public static final int LEDPort = 0;

        public static final boolean VisionTrackingStrafe = true;
    }

    public static final class Balance { // TODO move to balance command
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 1;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 3;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    }

    public static final class ShooterPresets {
        public static final class High {
            public static final double topWheelSpeed = 4300;
            public static final double bottomWheelSpeed = 5600;
        }

        public static final class Mid {
            public static final double topWheelSpeed = 3500 * 0.75 * 1.15;
            public static final double bottomWheelSpeed = 4200 * 0.85 * 1.15;
        }

        public static final class Far {
            public static final double topWheelSpeed = 100000;
            public static final double bottomWheelSpeed = 5600;
        }
    }

    public static final class Shooter {
        public static final int kTopShooterID = 15;
        public static final int kBottomShooterID = 16;
        public static final double kP = 0.000017;
        public static final double kD = 0;
        public static final double kI = 0;
        public static final double kFF = 0.00018;
    }

    public static final class ElevatorConveyerThing {
        public static final int ConveyerCanID = 14;
        public static final double kP = 0.000017;
        public static final double kD = 0;
        public static final double kI = 0;
        public static final double kFF = 0.00022;

        public static final double bingChillinVelocity = 450; // the constant motor velocity
        public static final double bingFastinVelocity = 2000; // the elevator velocity while shooting


        public static final double r = 54;
        public static final double g = 80;
        public static final double b = 120;

        public static final double WiggleRoom = 25;
    }

    public static final class intake {
        public static final int intakeCanID = 13;
        public static final double kP = 0.3;
        public static final double kD = 0;
        public static final double kI = 0;

        public static final double intakeVelocity = 1; // the constant motor velocity
    }

    public static final class LimeLight {
        public static final double AprilTagHeight = 0.50;
        public static final double LimelightHeight = 0.46;
        public static final double LimelightAngle = 14;
    }

    public static final class Drivetrain {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        // public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        // public static final double kFrontRightChassisAngularOffset = 0;
        // public static final double kBackLeftChassisAngularOffset = Math.PI;
        // public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final double kFrontLeftChassisAngularOffset = Math.PI; // ! Pract 0
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = 0; // ! Pract Math.PI;;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 9;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kRearRightTurningCanId = 5;

        public static final boolean kGyroReversed = true;

        public static final double stallCurrentLimit =
                40; // If the current for any of the motors exceeds this limit, isSTall returns true
        // !! TODO check if the motor has been still for a set time as well. Currently it will think its
        // stalling when it starts accelerating.
    }

    public static final class SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoMotorFreeRpm = 5676;
        public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction =
                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
                kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.4;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class Auton {
        public static final double kMaxSpeedMetersPerSecond = 4.8; // 4
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 2

        public static final double kPXYController = 3; // TODO: tune PID for autos
        public static final double kPThetaController = 1.5; // ! 1
    }

    public static final class Intake { // TODO move into intake subsytem
        // Time in milliseconds that it should take to eject a cube / cone
        public static final long IntakeEjectionTime = 500;
        // Time in milliseconds that it should take to intake a cube / cone
        public static final long IntakeTime = 500;
    }
}
