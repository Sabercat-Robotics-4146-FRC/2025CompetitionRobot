package frc.robot.commands.alignment;

import static frc.robot.Constants.RobotDesiredPositions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotDesiredPositions.DesiredPosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestRightReef extends Command {
  Drive drive;
  RobotContainer container;

  public AlignNearestRightReef(Drive drive, RobotContainer container) {
    this.drive = drive;
    this.container = container;
  }

  @Override
  public void execute() {
    double shortestDistance = 1000;
    Supplier<Pose2d> shortestPose = () -> new Pose2d();
    for (DesiredPosition reef : RIGHT_REEFS) {
      double distance = drive.getPose().getTranslation().getDistance(reef.pose.getTranslation());
      if (distance < shortestDistance) {
        shortestDistance = distance;
        shortestPose = () -> reef.pose;
      }
    }
    drive.setAutoAlignGoal(shortestPose, false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
