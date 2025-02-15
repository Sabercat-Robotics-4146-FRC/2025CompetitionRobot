package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  public final int[] powerPorts = {};

  @AutoLog
  public static class IndexerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double servoPosition = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void setServo(double position) {}

  public default void stop() {}

  public default void conifgurePID(double kP, double kI, double kD) {}
}
