package frc.robot.commands;

import static frc.robot.Constants.RobotDesiredPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotDesiredPositions.DesiredPosition;
import frc.robot.FieldOffsetCompensation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestFeeder extends Command {
  Drive drive;
  RobotContainer container;

  public static final DesiredPosition[] FEEDERS = {
    new DesiredPosition(
        "FEEDER_1",
        new Pose2d(new Translation2d(16.48, 1.04), new Rotation2d(Units.degreesToRadians(130.23)))
            .transformBy(FieldOffsetCompensation.FEEDER)),
    new DesiredPosition(
        "FEEDER_2",
        new Pose2d(new Translation2d(16.33, 7.05), new Rotation2d(Units.degreesToRadians(-126.20)))
            .transformBy(FieldOffsetCompensation.FEEDER)),
    new DesiredPosition(
        "FEEDER_12",
        new Pose2d(new Translation2d(1.23, 1.00), new Rotation2d(Units.degreesToRadians(53.00)))
            .transformBy(FieldOffsetCompensation.FEEDER)),
    new DesiredPosition(
        "FEEDER_13",
        new Pose2d(new Translation2d(1.08, 6.97), new Rotation2d(Units.degreesToRadians(-49.66)))
            .transformBy(FieldOffsetCompensation.FEEDER))
  };

  public AlignNearestFeeder(Drive drive, RobotContainer container) {
    this.drive = drive;
    this.container = container;
  }

  @Override
  public void execute() {
    double shortestDistance = 1000;
    String shortestName = "";
    Supplier<Pose2d> shortestPose = () -> new Pose2d();
    for (DesiredPosition feeder : FEEDERS) {
      double distance = drive.getPose().getTranslation().getDistance(feeder.pose.getTranslation());
      if (distance < shortestDistance) {
        shortestDistance = distance;
        shortestName = feeder.name;
        shortestPose = () -> feeder.pose;
      }
    }

    System.out.println("aligning with tag : " + shortestName);
    drive.setAutoAlignGoal(shortestPose, false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
