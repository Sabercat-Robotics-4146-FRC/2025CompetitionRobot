package frc.robot.subsystems.squidward;

import org.littletonrobotics.junction.AutoLog;

public interface SquidwardIO {
  // -- Power Ports used by Elevators -- //
  public final int[] powerPorts = {};

  @AutoLog
  public static class SquidwardIOInputs {
    public boolean motorConnected = true;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(SquidwardIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void runVelocity(double velocityRadPerSec) {}

  default void stop() {}

  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(
      double kP, double kI, double kD, double kG, double kV, double kA, double kS) {}

  default void setBrakeMode(boolean enabled) {}

  default void zeroPosition() {}
  ;
}
