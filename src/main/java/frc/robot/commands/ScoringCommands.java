package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.Constants.FieldPose;
import frc.robot.Constants.IntakeAction;
import frc.robot.Constants.PivotPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class ScoringCommands {
  private ScoringCommands() {}

  // TODO: IMPLEMENT CANRANGES FOR PIVOT.
  public static Command scoreL4(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Pivot pivot,
      Intake intake,
      Supplier<FieldPose> targetSupplier) {
    return new SequentialCommandGroup(
        DriveCommands.driveToPoseIfClose(
            drive, vision, targetSupplier, DriveConstants.DriveToPoseThreshold),
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L4),
        pivot.moveToPosition(() -> PivotPosition.L4),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L4.getSpeed())),
        new WaitCommand(1.5),
        new InstantCommand(() -> intake.setSpeed(0.0)));
  }

  public static Command scoreL3(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Pivot pivot,
      Intake intake,
      Supplier<FieldPose> targetSupplier) {
    return new SequentialCommandGroup(
        DriveCommands.driveToPoseIfClose(
            drive, vision, targetSupplier, DriveConstants.DriveToPoseThreshold),
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L3),
        pivot.moveToPosition(() -> PivotPosition.L3),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L3.getSpeed())),
        new WaitCommand(1.5),
        new InstantCommand(() -> intake.setSpeed(0.0)));
  }

  public static Command scoreL2(
      Drive drive,
      Vision vision,
      Elevator elevator,
      Pivot pivot,
      Intake intake,
      Supplier<FieldPose> targetSupplier) {
    return new SequentialCommandGroup(
        DriveCommands.driveToPoseIfClose(
            drive, vision, targetSupplier, DriveConstants.DriveToPoseThreshold),
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L2),
        pivot.moveToPosition(() -> PivotPosition.L2),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L2.getSpeed())),
        new WaitCommand(1.5),
        new InstantCommand(() -> intake.setSpeed(0.0)));
  }
}
