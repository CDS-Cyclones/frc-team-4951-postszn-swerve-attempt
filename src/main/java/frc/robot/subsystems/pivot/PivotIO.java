package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean motorConnected = false;
    public double motorSpeed = 0.0;
    public double motorCurrent = 0.0;
    public double motorVoltage = 0.0;
    public double motorTemperature = 0.0;
    public double motorAbsolutePosition = 0.0;
    public double motorRelativePosition = 0.0;
    public double motorVelocity = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}
}
