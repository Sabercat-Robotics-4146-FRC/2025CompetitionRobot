package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldOffsetCompensation {
  public static final Transform2d REEF =
      new Transform2d(new Translation2d(0, 0), new Rotation2d(0));
  public static final Transform2d FEEDER =
      new Transform2d(new Translation2d(0, 0), new Rotation2d(0));
}
