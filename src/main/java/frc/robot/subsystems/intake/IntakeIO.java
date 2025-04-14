package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double intakeSpeed = 0.0;
    public double intakeCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeTemperature = 0.0;
    public boolean coralInflowCanrangeConnected = false;
    public boolean coralOutflowCanrangeConnected = false;
    public double coralInflowCanrangeDistance = 0.0;
    public double coralOutflowCanrangeDistance = 0.0;
    public boolean coralDetectedOnInflow = false;
    public boolean coralDetectedOnOutflow = false;
    public boolean intakeContainsCoral = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
