package frc.robot.commands;

import static frc.robot.Constants.RobotDesiredPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotDesiredPositions.DesiredPosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestTag extends Command {
  Drive drive;
  RobotContainer container;

  public AlignNearestTag(Drive drive, RobotContainer container) {
    this.drive = drive;
    this.container = container;
  }

  @Override
  public void execute() {
    System.out.println("Made it here");
    double shortestDistance = 1000;
    String shortestName = "";
    Supplier<Pose2d> shortestPose = () -> new Pose2d();
    for (DesiredPosition position : POSITIONS) {
      double distance =
          drive.getPose().getTranslation().getDistance(position.pose.getTranslation());
      if (distance < shortestDistance) {
        shortestDistance = distance;
        shortestName = position.name;
        shortestPose = () -> position.pose;
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
