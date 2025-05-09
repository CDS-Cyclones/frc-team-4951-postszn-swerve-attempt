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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
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
}
