package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements ElevatorIO {
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  protected final SparkMax motor = new SparkMax(Constants.elevatorMotor1Id, MotorType.kBrushless);
  protected final SparkMax motorFollower =
      new SparkMax(Constants.elevatorMotor2Id, MotorType.kBrushless);
  private final RelativeEncoder encoder, encoderFollower;
  private final SparkClosedLoopController motorController;
  private static final SparkBaseConfig motorConfig = new SparkMaxConfig();
  private static final SparkBaseConfig motorConfigFollower = new SparkMaxConfig();
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.elevatorKs, Constants.elevatorKg, Constants.elevatorKv, Constants.elevatorKa);
  private final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.4).per(Second), Volts.of(1.3), null),
          new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this));

  public Elevator() {
    configMotors();
    encoder = motor.getEncoder();
    encoderFollower = motorFollower.getEncoder();
    motorController = motor.getClosedLoopController();

    zeroEncoders();
  }

  /** Configures the motors for the elevator. */
  public void configMotors() {
    motorConfig
        .smartCurrentLimit(40)
        .secondaryCurrentLimit(40)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(Constants.elevatorMotorInverted);
    motorConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.elevatorMaxPosition)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.elevatorMinPosition);
    motorConfig
        .encoder
        .positionConversionFactor(Constants.elevatorDistancePerRevolution)
        .velocityConversionFactor(Constants.elevatorVelocityMetersPerSecond);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.elevatorKp, 0.0, Constants.elevatorKd, Constants.elevatorKff)
        .outputRange(Constants.elevatorMinSpeed, Constants.elevatorMaxSpeed);
    motorConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.elevatorPositionTolerance);
    motorConfigFollower
        .apply(motorConfig)
        .inverted(Constants.elevatorMotorFollowerInverted)
        .follow(motor);
    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    motorFollower.configure(
        motorConfigFollower,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    try {
      //  SmartDashboard.putString("Mutables/Elevator Position",
      // RobotStateManager.getDesiredElevatorPosition().toString());
      SmartDashboard.putNumber("Elevator Height", getPosition());
    } catch (Exception e) {
    }
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getPosition() {
    return (encoder.getPosition() + encoderFollower.getPosition()) / 2.0;
  }

  public double getVelocity() {
    return (encoder.getVelocity() + encoderFollower.getVelocity()) / 2.0;
  }

  public double calculateFeedforward(double velocity) {
    return feedforward.calculate(velocity) * 0.5;
  }

  public void setReference(double position) {
    motorController.setReference(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        calculateFeedforward(getVelocity()),
        ArbFFUnits.kVoltage);
  }

  public boolean isAtPosition(double position) {
    return Math.abs(getPosition() - position) < Constants.elevatorPositionTolerance;
  }

  public boolean isAtPosition(ElevatorPosition position) {
    return isAtPosition(getPosition());
  }

  public Command moveToPosition(Pivot pivot, Supplier<ElevatorPosition> position) {
    return Commands.sequence(
        Commands.runOnce(
                () -> {
                  double targetPosition = getPosition();

                  setReference(targetPosition);
                },
                this)
            .onlyIf(pivot::isOutOfElevatorWay),
        Commands.waitUntil(() -> isAtPosition(position.get())));
  }
  ;

  public void logMotors(SysIdRoutineLog log) {
    log.motor("elevator-motor-1")
        .voltage(Volts.of(motor.getBusVoltage() * motor.getAppliedOutput()));
    log.motor("elevator-motor-1").linearPosition(Meters.of(encoder.getPosition()));
    log.motor("elevator-motor-1").linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
    log.motor("elevator-motor-2")
        .voltage(Volts.of(motorFollower.getBusVoltage() * motorFollower.getAppliedOutput()));
    log.motor("elevator-motor-2").linearPosition(Meters.of(encoderFollower.getPosition()));
    log.motor("elevator-motor-2").linearVelocity(MetersPerSecond.of(encoderFollower.getVelocity()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
