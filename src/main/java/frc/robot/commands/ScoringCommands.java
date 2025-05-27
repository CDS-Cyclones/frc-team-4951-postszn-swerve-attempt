package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.Constants.IntakeAction;
import frc.robot.Constants.PivotPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;

public class ScoringCommands {
  private ScoringCommands() {}

  public static Command scoreL4(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
            // Step 1: Move pivot to ELEVATOR_CLEAR.
            pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
            // Step 2: If pivot is clear, run the scoreL2 sequence; otherwise do nothing.
            // If the condition is true:
            new SequentialCommandGroup(
                elevator.moveToPosition(pivot, () -> ElevatorPosition.L4),
                pivot.moveToPosition(() -> PivotPosition.L2),
                Commands.runOnce(()-> intake.setSpeed(IntakeAction.SCORE_L4.getSpeed())),
                new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
                Commands.runOnce((() -> intake.setSpeed(0.0))),
                Commands.runOnce(() -> retractElevatorPivot(elevator, pivot))))
        .onlyIf(pivot::isOutOfElevatorWay);
  }

  public static Command scoreL3(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
            // Step 1: Move pivot to ELEVATOR_CLEAR.
            pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
            // Step 2: If pivot is clear, run the scoreL2 sequence; otherwise do nothing.
            // If the condition is true:
            new SequentialCommandGroup(
                elevator.moveToPosition(pivot, () -> ElevatorPosition.L3),
                pivot.moveToPosition(() -> PivotPosition.L3),
                Commands.runOnce(()-> intake.setSpeed(IntakeAction.SCORE_L3.getSpeed())),
                new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
                Commands.runOnce((() -> intake.setSpeed(0.0))),
                Commands.runOnce(() -> retractElevatorPivot(elevator, pivot))))
        .onlyIf(pivot::isOutOfElevatorWay);
  }
  }

  public static Command scoreL2(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
            // Step 1: Move pivot to ELEVATOR_CLEAR.
            pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
            // Step 2: If pivot is clear, run the scoreL2 sequence; otherwise do nothing.
            // If the condition is true:
            new SequentialCommandGroup(
                elevator.moveToPosition(pivot, () -> ElevatorPosition.L2),
                pivot.moveToPosition(() -> PivotPosition.L2),
                Commands.runOnce(()-> intake.setSpeed(IntakeAction.SCORE_L2.getSpeed())),
                new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
                Commands.runOnce((() -> intake.setSpeed(0.0))),
                Commands.runOnce(() -> retractElevatorPivot(elevator, pivot))))
        .onlyIf(pivot::isOutOfElevatorWay);
  }

  public static Command intakeFromStation(Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> intake.setSpeed(IntakeAction.INTAKE_CORAL.getSpeed())),
        new WaitUntilCommand(() -> intake.coralDetectedAtOutflow()),
        new InstantCommand(() -> intake.setSpeed(0.0)));
  }

  public static Command retractElevatorPivot(Elevator elevator, Pivot pivot) {
    return new SequentialCommandGroup(
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.DOWN),
        pivot.moveToPosition(() -> PivotPosition.INTAKE_READY));
  }

  public static Command retractElevator(Elevator elevator, Pivot pivot) {
    return new SequentialCommandGroup(elevator.moveToPosition(pivot, () -> ElevatorPosition.DOWN));
  }

  public static Command deAlgafyL2(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
        // Step 1: Move pivot to ELEVATOR_CLEAR.
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        // Step 2: Check if pivot is now out of the elevator way.
        new ConditionalCommand(
            // If true, run the following sequence.
            new SequentialCommandGroup(
                elevator.moveToPosition(pivot, () -> ElevatorPosition.REEF_ALGA_L2),
                pivot.moveToPosition(() -> PivotPosition.REEF_ALGA_L2),
                new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L2.getSpeed())),
                new WaitCommand(1.5),
                new InstantCommand(() -> intake.stop()),
                new InstantCommand(() -> retractElevator(elevator, pivot))),
            // If false, do nothing.
            Commands.none(),
            // Condition to check.
            pivot::isOutOfElevatorWay));
  }

  public static Command deAlgafyL3(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
        // Step 1: Move pivot to ELEVATOR_CLEAR.
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        // Step 2: Check if pivot is now out of the elevator way.
        new ConditionalCommand(
            // If true run the following sequence.
            new SequentialCommandGroup(
                elevator.moveToPosition(pivot, () -> ElevatorPosition.REEF_ALGA_L3),
                pivot.moveToPosition(() -> PivotPosition.REEF_ALGA_L2),
                new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L3.getSpeed())),
                new WaitCommand(1.5),
                new InstantCommand(() -> intake.stop()),
                new InstantCommand(() -> retractElevator(elevator, pivot))),
            // If false do nothing.
            Commands.none(),
            // Condition to check.
            pivot::isOutOfElevatorWay));
  }
}
