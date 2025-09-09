package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoreSide;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestLeftReef;
import frc.robot.commands.alignment.AlignNearestRightReef;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestReef extends SequentialCommandGroup {
  public AlignNearestReef(Drive drive, RobotContainer container, Supplier<ScoreSide> side) {
    addCommands(
        new ConditionalCommand(
            new AlignNearestLeftReef(drive, container),
            new AlignNearestRightReef(drive, container),
            () -> side.get() == ScoreSide.LEFT));
  }
}
