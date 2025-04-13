package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private final SparkMax motor = new SparkMax(Constants.pivotMotorId, MotorType.kBrushless);
  private final AbsoluteEncoder absolute;
  private final RelativeEncoder relative;
  private final SparkClosedLoopController motorController;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.pivotKs, Constants.pivotKg, Constants.pivotKv, Constants.pivotKa);
  private static final SparkBaseConfig motorConfig = new SparkMaxConfig();

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
}
