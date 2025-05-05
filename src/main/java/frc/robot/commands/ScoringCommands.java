package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L4),
        pivot.moveToPosition(() -> PivotPosition.L4),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L4.getSpeed())),
        new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
        new InstantCommand(() -> intake.setSpeed(0.0)),
        new InstantCommand(() -> retractElevatorPivot(elevator, pivot)));
  }

  public static Command intakeCoralFromStation(Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> intake.setSpeed(IntakeAction.INTAKE_CORAL.getSpeed())),
        new WaitUntilCommand(() -> intake.coralDetectedAtOutflow()),
        new InstantCommand(() -> intake.setSpeed(0.0)));
  }

  public static Command scoreL3(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L3),
        pivot.moveToPosition(() -> PivotPosition.L3),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L3.getSpeed())),
        new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
        new InstantCommand(() -> intake.setSpeed(0.0)),
        new InstantCommand(() -> retractElevatorPivot(elevator, pivot)));
  }

  public static Command scoreL2(
      Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake) {
    return new SequentialCommandGroup(
        elevator.moveToPosition(pivot, () -> ElevatorPosition.L2),
        pivot.moveToPosition(() -> PivotPosition.L2),
        new InstantCommand(() -> intake.setSpeed(IntakeAction.SCORE_L2.getSpeed())),
        new WaitUntilCommand(() -> !intake.coralDetectedAtOutflow()),
        new InstantCommand(() -> intake.setSpeed(0.0)),
        new InstantCommand(() -> retractElevatorPivot(elevator, pivot)));
  }

  public static Command retractElevatorPivot(Elevator elevator, Pivot pivot) {
    return new SequentialCommandGroup(
        pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
        elevator.moveToPosition(pivot, () -> ElevatorPosition.DOWN),
        pivot.moveToPosition(() -> PivotPosition.INTAKE_READY));
  }

  // TODO : Finish Alga Commands

//   public static Command deAlgafyL2(Drive drive, Vision vision, Elevator elevator, Pivot pivot, Intake intake){
//   return new SequentialCommandGroup(
//         pivot.moveToPosition(() -> PivotPosition.ELEVATOR_CLEAR),
//         elevator.moveToPosition(pivot, () -> ElevatorPosition.REEF_ALGA_L2), 
//         pivot.moveToPosition(() -> PivotPosition.REEF_ALGA_L2),





// );

//   }
}
