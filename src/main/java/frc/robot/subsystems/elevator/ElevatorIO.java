package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean[] motorConnected = {false, false};
    public double[] motorSpeed = {0.0, 0.0};
    public double[] motorCurrent = {0.0, 0.0};
    public double[] motorVoltage = {0.0, 0.0};
    public double[] motorTemperature = {0.0, 0.0};
    public double[] motorPosition = {0.0, 0.0};
    public double[] motorVelocity = {0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
