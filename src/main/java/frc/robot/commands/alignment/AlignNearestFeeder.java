package frc.robot.commands.alignment;

import static frc.robot.Constants.RobotDesiredPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotDesiredPositions.DesiredPosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestFeeder extends Command {
  Drive drive;
  RobotContainer container;

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
