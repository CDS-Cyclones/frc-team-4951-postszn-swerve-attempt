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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldPose;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.simulation.GyroIOSim;
import frc.robot.subsystems.drive.simulation.ModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public final GenericHID m_operatorBoard = new GenericHID(1);
  private final Vision visionSim;
  private final Vision vision;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
        vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.limeLightName, drive::getRotation));
        visionSim = new Vision(drive);
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        // new Vision(
        //     drive,
        //     new VisionIOPhotonVisionSim(
        //         "camera",
        //         VisionConstants.botToCamTransformSim,
        //         driveSimulation::getSimulatedDriveTrainPose));
        visionSim =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    "camera",
                    VisionConstants.botToCamTransformSim,
                    driveSimulation::getSimulatedDriveTrainPose));

        vision = new Vision(drive);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        visionSim =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    "camera",
                    VisionConstants.botToCamTransformSim,
                    driveSimulation::getSimulatedDriveTrainPose));

        vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.limeLightName, drive::getRotation));

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Drive in half-speed when left bumper is held
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * 0.5,
                () -> -controller.getLeftX() * 0.5,
                () -> -controller.getRightX() * 0.5));
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    switch (Constants.currentMode) {
        // ************************************************************************************************************************
        // ************************************************************************************************************************
        //                                             Sim Robot
        // ************************************************************************************************************************
        // ************************************************************************************************************************.
      case SIM:
        // Button 1 -> FieldPose.A
        new JoystickButton(m_operatorBoard, 1)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.A.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 1 pressed: Within 1.5 m. Driving to FieldPose.A");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.A).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 1 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 2 -> FieldPose.B
        new JoystickButton(m_operatorBoard, 2)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.B.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 2 pressed: Within 1.5 m. Driving to FieldPose.B");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.B).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 2 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 3 -> FieldPose.C
        new JoystickButton(m_operatorBoard, 3)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.C.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 3 pressed: Within 1.5 m. Driving to FieldPose.C");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.C).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 3 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 4 -> FieldPose.D
        new JoystickButton(m_operatorBoard, 4)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.D.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 4 pressed: Within 1.5 m. Driving to FieldPose.D");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.D).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 4 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 5 -> FieldPose.E
        new JoystickButton(m_operatorBoard, 5)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.E.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 5 pressed: Within 1.5 m. Driving to FieldPose.E");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.E).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 5 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 6 -> FieldPose.F
        new JoystickButton(m_operatorBoard, 6)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.F.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 6 pressed: Within 1.5 m. Driving to FieldPose.F");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.F).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 6 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 7 -> FieldPose.G
        new JoystickButton(m_operatorBoard, 7)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.G.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 7 pressed: Within 1.5 m. Driving to FieldPose.G");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.G).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 7 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 8 -> FieldPose.H
        new JoystickButton(m_operatorBoard, 8)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.H.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 8 pressed: Within 1.5 m. Driving to FieldPose.H");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.H).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 8 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 9 -> FieldPose.I
        new JoystickButton(m_operatorBoard, 9)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.I.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 9 pressed: Within 1.5 m. Driving to FieldPose.I");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.I).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 9 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 10 -> FieldPose.J
        new JoystickButton(m_operatorBoard, 10)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.J.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 10 pressed: Within 1.5 m. Driving to FieldPose.J");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.J).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 10 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 11 -> FieldPose.K
        new JoystickButton(m_operatorBoard, 11)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.K.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 11 pressed: Within 1.5 m. Driving to FieldPose.K");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.K).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 11 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 12 -> FieldPose.L
        new JoystickButton(m_operatorBoard, 12)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.L.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 12 pressed: Within 1.5 m. Driving to FieldPose.L");
                        DriveCommands.DriveToPose(drive, visionSim, () -> FieldPose.L).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 12 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        break;

        // ************************************************************************************************************************
        // ************************************************************************************************************************
        //                                             REAL ROBOT
        // ************************************************************************************************************************
        // ************************************************************************************************************************
      case REAL:
        // Button 1 -> FieldPose.A
        new JoystickButton(m_operatorBoard, 1)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.A.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 1 pressed: Within 1.5 m. Driving to FieldPose.A");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.A).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 1 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 2 -> FieldPose.B
        new JoystickButton(m_operatorBoard, 2)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.B.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 2 pressed: Within 1.5 m. Driving to FieldPose.B");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.B).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 2 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 3 -> FieldPose.C
        new JoystickButton(m_operatorBoard, 3)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.C.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 3 pressed: Within 1.5 m. Driving to FieldPose.C");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.C).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 3 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 4 -> FieldPose.D
        new JoystickButton(m_operatorBoard, 4)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.D.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 4 pressed: Within 1.5 m. Driving to FieldPose.D");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.D).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 4 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 5 -> FieldPose.E
        new JoystickButton(m_operatorBoard, 5)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.E.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 5 pressed: Within 1.5 m. Driving to FieldPose.E");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.E).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 5 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 6 -> FieldPose.F
        new JoystickButton(m_operatorBoard, 6)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.F.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 6 pressed: Within 1.5 m. Driving to FieldPose.F");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.F).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 6 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 7 -> FieldPose.G
        new JoystickButton(m_operatorBoard, 7)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.G.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 7 pressed: Within 1.5 m. Driving to FieldPose.G");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.G).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 7 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 8 -> FieldPose.H
        new JoystickButton(m_operatorBoard, 8)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.H.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 8 pressed: Within 1.5 m. Driving to FieldPose.H");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.H).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 8 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 9 -> FieldPose.I
        new JoystickButton(m_operatorBoard, 9)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.I.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 9 pressed: Within 1.5 m. Driving to FieldPose.I");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.I).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 9 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 10 -> FieldPose.J
        new JoystickButton(m_operatorBoard, 10)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.J.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 10 pressed: Within 1.5 m. Driving to FieldPose.J");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.J).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 10 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 11 -> FieldPose.K
        new JoystickButton(m_operatorBoard, 11)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.K.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 11 pressed: Within 1.5 m. Driving to FieldPose.K");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.K).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 11 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        // Button 12 -> FieldPose.L
        new JoystickButton(m_operatorBoard, 12)
            .onTrue(
                Commands.runOnce(
                    () -> {
                      Pose2d targetPose = FieldPose.L.getDesiredPose().toPose2d();
                      Pose2d currentPose = drive.getPose();
                      double distance =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      if (distance < 1.5) {
                        System.out.println(
                            "OP Board button 12 pressed: Within 1.5 m. Driving to FieldPose.L");
                        DriveCommands.DriveToPose(drive, vision, () -> FieldPose.L).schedule();
                      } else {
                        System.out.println(
                            "OP Board button 12 pressed: NOT within 1.5 m (" + distance + " m)");
                      }
                    }));
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}
