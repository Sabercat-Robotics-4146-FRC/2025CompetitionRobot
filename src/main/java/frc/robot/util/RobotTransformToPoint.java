package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotTransformToPoint {
  public RobotTransformToPoint() {}

  public static Pose2d Convert(Pose2d robotPose, Translation2d translation) {
    return robotPose.transformBy(
        new Transform2d(
            translation.rotateBy(robotPose.getRotation()).times(-1.0), new Rotation2d()));
  }
}
