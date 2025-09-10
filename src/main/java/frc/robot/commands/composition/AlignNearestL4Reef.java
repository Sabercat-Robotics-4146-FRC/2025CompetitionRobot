package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoreSide;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestLeftReefL4;
import frc.robot.commands.alignment.AlignNearestRightReefL4;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignNearestL4Reef extends SequentialCommandGroup {
  public AlignNearestL4Reef(Drive drive, RobotContainer container, Supplier<ScoreSide> side) {
    addCommands(
        new ConditionalCommand(
            new AlignNearestLeftReefL4(drive, container),
            new AlignNearestRightReefL4(drive, container),
            () -> side.get() == ScoreSide.LEFT));
  }
}
