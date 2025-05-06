package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.PivotPosition;
import java.util.function.Supplier;

public class Pivot extends SubsystemBase implements PivotIO {
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final SparkMax motor = new SparkMax(Constants.pivotMotorId, MotorType.kBrushless);
  private final AbsoluteEncoder absolute;
  private final RelativeEncoder relative;
  private final SparkClosedLoopController motorController;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.pivotKs, Constants.pivotKg, Constants.pivotKv, Constants.pivotKa);
  private static final SparkBaseConfig motorConfig = new SparkMaxConfig();
  private final SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(0.8), null),
          new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this));

  public Pivot() {
    configMotor();
    absolute = motor.getAbsoluteEncoder();
    relative = motor.getEncoder();
    motorController = motor.getClosedLoopController();

    relative.setPosition(absolute.getPosition() + Constants.pivotOffsetFromHorizontal);
  }

  public void configMotor() {
    motorConfig
        .smartCurrentLimit(80)
        .secondaryCurrentLimit(80)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(Constants.pivotMotorInverted);
    motorConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.pivotMaxPosition)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.pivotMinPosition);
    motorConfig
        .absoluteEncoder
        .positionConversionFactor(Constants.pivotAbsoluteEncoderRadiansPerRevolution)
        .velocityConversionFactor(Constants.pivotAbsoluteEncoderAngularVelocityRadiansPerSecond)
        .inverted(false);
    motorConfig
        .encoder
        .positionConversionFactor(Constants.pivotRelativeEncoderRadiansPerRevolution)
        .velocityConversionFactor(Constants.pivotRelativeEncoderAngularVelocityRadiansPerSecond);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Constants.pivotKp, 0.0, Constants.pivotKd, Constants.pivotKff)
        .outputRange(Constants.pivotMinSpeed, Constants.pivotMaxSpeed);
    motorConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.pivotPositionTolerance);
    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    relative.setPosition(absolute.getPosition() + Constants.pivotOffsetFromHorizontal);
    updateInputs(pivotInputs);
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

  public double getPosition() {
    return relative.getPosition();
  }

  public double getVelocity() {
    return relative.getVelocity();
  }

  // realistically i need to research this alot more.
  public double calculateFeedforward(double velocity) {
    return feedforward.calculate(getPosition(), velocity);
  }

  public boolean isOutOfElevatorWay() {
    return getPosition() <= Constants.pivotMinPositionToMoveElevator;
  }

  public boolean isAtPosition(double position) {
    return Math.abs(getPosition() - position) < Constants.elevatorPositionTolerance;
  }

  public boolean isAtPosition(PivotPosition position) {
    return isAtPosition(position.getAsDouble());
  }

  public Command moveToPosition(Supplier<PivotPosition> position) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              double targetPosition = position.get().getAsDouble();

              setReference(targetPosition);
            },
            this),
        Commands.waitUntil(() -> isAtPosition(position.get().getAsDouble())));
  }

  public void setReference(double position) {
    motorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorConnected = motor.getFirmwareVersion() != 0;
    inputs.motorSpeed = motor.getAppliedOutput();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorVoltage = motor.getBusVoltage();
    inputs.motorTemperature = motor.getMotorTemperature();
    inputs.motorAbsolutePosition = absolute.getPosition();
    inputs.motorRelativePosition = relative.getPosition();
    inputs.motorVelocity = relative.getVelocity();
  }

  public void logMotors(SysIdRoutineLog log) {
    log.motor("pivot-motor").voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()));
    log.motor("pivot-motor").angularPosition(Radians.of(getPosition()));
    log.motor("pivot-motor").angularVelocity(RadiansPerSecond.of(getVelocity()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void stop() {
    motor.stopMotor();
  }
}
