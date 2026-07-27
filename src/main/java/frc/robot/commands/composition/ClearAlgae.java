package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeLevel;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestRightReef;
import frc.robot.commands.squidward.ClearBottom;
import frc.robot.commands.squidward.ClearTop;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.squidward.Squidward;

public class ClearAlgae extends SequentialCommandGroup {
  public ClearAlgae(Drive drive, RobotContainer container, Squidward squid) {
    addCommands(
        new AlignNearestRightReef(drive, container),
        new ConditionalCommand(
            new ClearTop(squid, drive, container),
            new ClearBottom(squid, drive, container),
            () -> squid.getNearestAlgaeLevel(drive.getPose()) == AlgaeLevel.TOP));
  }
}
