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

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;
import lombok.AllArgsConstructor;
import lombok.RequiredArgsConstructor;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  private static final double inFrontOfTag = Units.inchesToMeters(23);
  private static final double rightOfTag = Units.inchesToMeters(6.6);
  private static final double leftOfTag = -Units.inchesToMeters(7.15);
  private static final double inFrontOfTagSim = 0.4;

  public static final int elevatorMotor1Id = 31;
  public static final int elevatorMotor2Id = 32;

  public static final boolean elevatorMotorInverted = false;
  public static final boolean elevatorMotorFollowerInverted = false;
  public static final double elevatorMinPosition = 0.0;
  public static final double elevatorMaxPosition = 1.8;
  public static final double elevatorKp = 2.85;
  public static final double elevatorKd = 0.0;
  public static final double elevatorKff = 1 / 473; // not arbFF, inverse of motor specific Kv value
  public static final double elevatorKs = 0.58291;
  public static final double elevatorKg = 0.62411;
  public static final double elevatorKv = 3.2946;
  public static final double elevatorKa = 0.62268;
  public static final double elevatorDistancePerRevolution = Units.inchesToMeters(63) / 52.25;
  public static final double elevatorVelocityMetersPerSecond = elevatorDistancePerRevolution / 60.0;
  public static final double elevatorMinSpeed = -0.5; // max speed going down
  public static final double elevatorMaxSpeed = 0.61; // max speed going up
  public static final double elevatorPositionTolerance = 0.05;
  public static final int pivotMotorId = 56;
  public static final double pivotKff = 1 / 473; // not arbFF, inverse of motor specific Kv value
  public static final double pivotKs = 0.29184;
  public static final double pivotKg = 0.37151;
  public static final double pivotKv = 1.6051;
  public static final double pivotKa = 0;
  public static final double pivotOffsetFromHorizontal = -2.899 + Math.PI / 2;
  public static final boolean pivotMotorInverted = true;
  public static final double pivotMaxPosition = 2.28;
  public static final double pivotMinPosition = -1;
  public static final double pivotAbsoluteEncoderRadiansPerRevolution =
      Units.degreesToRadians(90) / 0.25;
  public static final double pivotAbsoluteEncoderAngularVelocityRadiansPerSecond =
      pivotAbsoluteEncoderRadiansPerRevolution / 60.0;
  public static final double pivotRelativeEncoderRadiansPerRevolution =
      Units.degreesToRadians(90) / 8.0;
  public static final double pivotRelativeEncoderAngularVelocityRadiansPerSecond =
      pivotRelativeEncoderRadiansPerRevolution / 60.0;
  public static final double pivotKp = 0.36;
  public static final double pivotKd = 0.07;
  public static final double pivotMaxSpeed = 0.18; // max speed going into the bot
  public static final double pivotMinSpeed = -0.17; // max speed leaving bot
  public static final double pivotPositionTolerance = 0.01;
  public static final double pivotMinPositionToMoveElevator = 2;
  public static final int intakeMotorId = 57;
  public static final int coralInflow = 41;
  public static final int coralOutflow = 41;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Stolen enum from andrii for pivot positions.
  @RequiredArgsConstructor
  public static enum PivotPosition {
    INTAKE_READY(2.27),
    ELEVATOR_CLEAR(1.7), // when empty
    ELEVATOR_CLEAR_WITH_ALGA(0.6),
    DEALGAEFY(1.5),
    L1(0.0),
    L2(1.79),
    L3(1.79),
    L4(1.86),
    REEF_ALGA_L2(-.5),
    REEF_ALGA_L3(-.5),
    BARGE_START(0.95),
    BARGE_END(1.43),
    PROCESSOR(-0.87),
    FULLY_OUT(-1),
    TUNABLE(Double.NaN); // Special value for tunable position
    private final double position;

    public double getAsDouble() {
      return position;
    }

    @Override
    public String toString() {
      return name() + " (" + ")";
    }
  }

  /** An enum to represent all desired field poses of the robot. */
  @RequiredArgsConstructor
  public enum FieldPose {
    // CORAL SCORING POSES MUST REMAIN FIRST 12!
    A(21, 10, inFrontOfTag, rightOfTag, Math.PI, false),
    B(21, 10, inFrontOfTag, leftOfTag, Math.PI, false),
    C(22, 9, inFrontOfTag, rightOfTag, Math.PI, false),
    D(22, 9, inFrontOfTag, leftOfTag, Math.PI, false),
    E(17, 8, inFrontOfTag, rightOfTag, Math.PI, false),
    F(17, 8, inFrontOfTag, leftOfTag, Math.PI, false),
    G(18, 7, inFrontOfTag, rightOfTag, Math.PI, false),
    H(18, 7, inFrontOfTag, leftOfTag, Math.PI, false),
    I(19, 6, inFrontOfTag, rightOfTag, Math.PI, false),
    J(19, 6, inFrontOfTag, leftOfTag, Math.PI, false),
    K(20, 11, inFrontOfTag, rightOfTag, Math.PI, false),
    L(20, 11, inFrontOfTag, leftOfTag, Math.PI, false),
    Z1(21, 10, inFrontOfTag, 0, Math.PI, false),
    Z2(22, 9, inFrontOfTag, 0, Math.PI, false),
    Z3(17, 8, inFrontOfTag, 0, Math.PI, false),
    Z4(18, 7, inFrontOfTag, 0, Math.PI, false),
    Z5(19, 6, inFrontOfTag, 0, Math.PI, false),
    Z6(20, 11, inFrontOfTag, 0, Math.PI, false);

    public final int tagBlueId;
    public final int tagRedId;
    public final double away;
    public final double side;
    public final double rotation; // in radians
    public final boolean orientationOnly;

    public int getTagID() {
      return DriverStation.getAlliance().get() == Alliance.Red ? tagRedId : tagBlueId;
    }

    public Pose3d getTagPose() {
      return VisionConstants.aprilTagLayout
          .getTagPose(getTagID())
          .orElseThrow(
              () -> new IllegalStateException("Pose3d not found for tag id: " + getTagID()));
    }

    public Pose3d getDesiredPose() {
      return getDesiredPose(false, false, false);
    }

    public Pose3d getDesiredPose(
        boolean ignoreForwards, boolean ignoreSideways, boolean ignoreRotation) {
      Pose3d tagPose = getTagPose();
      double tagAngle = tagPose.getRotation().toRotation2d().getRadians();
      double tagX = tagPose.getTranslation().getX();
      double tagY = tagPose.getTranslation().getY();

      // Ensure the angle is between 0 and 2pi
      if (tagAngle < 0) {
        tagAngle = 2 * Math.PI + tagAngle;
      }

      double cos = Math.cos(tagAngle);
      double sin = Math.sin(tagAngle);

      double newX = tagX;
      double newY = tagY;
      Pose3d newPose;

      switch (Constants.currentMode) {
        case SIM:
          newX += inFrontOfTagSim * cos;
          newY += inFrontOfTagSim * sin;
        default:
          newX += away * cos;
          newY += away * sin;
      }

      // now do transformation to the left or right of the tag
      newX += side * -sin;
      newY += side * cos;

      newPose =
          new Pose3d(
              new Translation3d(ignoreForwards ? tagX : newX, ignoreForwards ? tagY : newY, 0),
              new Rotation3d(0, 0, ignoreRotation ? tagAngle : tagAngle + rotation));

      return newPose;
    }

    /**
     * Return the desired rotation of the robot.
     *
     * @return {@link Rotation2d} of the robot.
     */
    public Rotation2d getDesiredRotation2d() {
      return getDesiredPose().getRotation().toRotation2d();
    }

    /**
     * Return whether the desired pose is orientation only meaning the robot should only rotate to
     * the desired angle and not move.
     *
     * @return True if the desired pose is orientation only.
     */
    public boolean isOrientationOnly() {
      return orientationOnly;
    }

    @Override
    public String toString() {
      return name();
    }
  }

  @AllArgsConstructor
  public static enum IntakeAction {
    NONE(0.0, 0.0),
    OCCUPIED(0.0, 0.0), // Special value for when the intake is occupied by another command
    SCORE_L1(0.2, 3.0),
    SCORE_L2(1.0, 1.1),
    SCORE_L3(1.0, 1.1),
    SCORE_L4(1.0, 1.2),
    SCORE_BARGE(1.0, 2.0),
    SCORE_PROCESSOR(1.0, 2.0),
    INTAKE_REEF_ALGA(-0.9, 1.5),
    OUTTAKE_REEF_ALGAE(1, 1.0),
    INTAKE_CORAL(0.56, 1.2), // Time is redundant; uses Canrange sensor
    TUNABLE(Double.NaN, Double.NaN); // Special values for tunable speed and duration

    private final double speed;
    private final Double time;

    /**
     * Returns the speed at which this action should run.
     *
     * @return The speed at which this action should run.
     */
    public double getSpeed() {
      return speed;
    }

    /**
     * Returns the time for which this action should run.
     *
     * @return The time for which this action should run.
     */
    public Double getTime() {
      return time;
    }
  }

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
    DOWN(0.0),
    L1(0.0),
    L2(0.31),
    L3(0.82),
    L4(1.58),
    REEF_ALGA_L2(0.6),
    REEF_ALGA_L3(0.95),
    BARGE(1.79),
    PROCESSOR(0.0),
    TUNABLE(Double.NaN); // Special value for tunable position

    private final double position;

    public double getPosition() {
      return position;
    }
  }
}
