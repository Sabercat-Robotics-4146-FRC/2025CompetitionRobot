package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  // -- Power Ports used by Elevators -- //
  public final int[] powerPorts = {};

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = true;
    public boolean followerConnected = true;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  // -- Run elevator output shaft to positionRad with addition feedforward output -- //
  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
  ;
}
