package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements IntakeIO {
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
  private final SparkMax intakeMotor = new SparkMax(Constants.intakeMotorId, MotorType.kBrushless);
  public final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  private static final CANrange coralInflow = new CANrange(Constants.coralInflow, "pigeonbus");
  private static final CANrange coralOutflow = new CANrange(Constants.coralOutflow, "pigeonbus");

  private boolean coralDetectedAtInflow;
  private boolean coralDetectedAtOutflow;
  private boolean intakeContainsCoral;

  public Intake() {
    intakeMotorConfig
        .smartCurrentLimit(80)
        .secondaryCurrentLimit(90)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);
    intakeMotor.configure(
        intakeMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // fart.
    updateSensorStatus();

    updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setVoltage(Voltage voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public boolean coralDetectedAtOutflow() {
    return coralOutflow.getDistance().getValue().in(Meters) < 0.1;
  }

  private boolean coralDetectedAtInflow() {
    return coralInflow.getDistance().getValue().in(Meters) < 0.1;
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  private void updateSensorStatus() {
    coralDetectedAtInflow = coralDetectedAtInflow();
    coralDetectedAtOutflow = coralDetectedAtOutflow();

    // If coral is detected at either the inflow or outflow, the intake contains coral.
    intakeContainsCoral = coralDetectedAtInflow || coralDetectedAtOutflow;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = intakeMotor.getFirmwareVersion() != 0;
    inputs.intakeSpeed = intakeMotor.getAppliedOutput();
    inputs.intakeCurrent = intakeMotor.getOutputCurrent();
    inputs.intakeVoltage = intakeMotor.getBusVoltage();
    inputs.intakeTemperature = intakeMotor.getMotorTemperature();
    inputs.coralInflowCanrangeConnected = coralInflow.isConnected();
    inputs.coralOutflowCanrangeConnected = coralOutflow.isConnected();
    inputs.coralInflowCanrangeDistance = coralInflow.getDistance().getValue().in(Meters);
    inputs.coralOutflowCanrangeDistance = coralOutflow.getDistance().getValue().in(Meters);
    inputs.coralDetectedOnInflow = coralDetectedAtInflow();
    inputs.coralDetectedOnOutflow = coralDetectedAtOutflow();
    inputs.intakeContainsCoral = intakeContainsCoral;
  }
}
