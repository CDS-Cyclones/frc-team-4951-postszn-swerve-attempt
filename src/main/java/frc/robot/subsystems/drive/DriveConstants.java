// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(27);
  public static final double wheelBase = Units.inchesToMeters(32.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);
  // Device CAN IDs
  public static final int pigeonCanId = 38;
  public static final String pigeonCanBus = "pigeonbus";

  public static final int frontLeftDriveCanId = 21;
  public static final int backLeftDriveCanId = 27;
  public static final int frontRightDriveCanId = 23;
  public static final int backRightDriveCanId = 25;

  public static final int frontLeftTurnCanId = 22;
  public static final int backLeftTurnCanId = 28;
  public static final int frontRightTurnCanId = 24;
  public static final int backRightTurnCanId = 26;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = 0.040704575067725526;
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.18406757595208734;
  public static final double driveKv = 0.09511381538864068;
  public static final double driveSimP = 5.00;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.04038;
  public static final double driveSimKv = 0.11972;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 40;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.05;
  public static final double turnSimP = 10;
  public static final double turnSimD = 0.2;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;

  // PID controllers
  public static final double anglePIDCKp = 3.2;
  public static final double anglePIDCKi = 0;
  public static final double anglePIDCKd = 0;
  public static final double anglePIDCTolerance = 0.02;
  public static final double anglePIDCMaxSpeed = 6; // in radians per second

  public static final double translationXPIDCKp = 3;
  public static final double translationXPIDCKi = 0;
  public static final double translationXPIDCKd = 0;
  public static final double translationXPIDCTolerance = 0.05;
  public static final double translationXPIDCMaxSpeed = 3; // in meters per second

  public static final double translationYPIDCKp = 3;
  public static final double translationYPIDCKi = 0;
  public static final double translationYPIDCKd = 0;
  public static final double translationYPIDCTolerance = 0.0;
  public static final double translationYPIDCMaxSpeed = 3; // in meters per second

  public static final PIDController angleController =
      new PIDController(anglePIDCKp, anglePIDCKi, anglePIDCKd);

  public static final PIDController translationXController =
      new PIDController(translationXPIDCKp, translationXPIDCKi, translationXPIDCKd);
  public static final PIDController translationYController =
      new PIDController(translationYPIDCKp, translationYPIDCKi, translationYPIDCKd);

  // PathPlanner configuration
  public static final double robotMassKg = 59.0531907;
  public static final double robotMOI = 7.31266;
  public static final double wheelCOF = 1.6;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // Maple-SIM configuration
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}
