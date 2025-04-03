package frc.robot.commands.squidward;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignNearestAlgae;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.squidward.Squidward;

public class ClearTop extends SequentialCommandGroup {
  public ClearTop(Squidward squid, Drive drive, RobotContainer container) {
    addCommands(
        new AlignNearestAlgae(drive, container),
        new WaitCommand(1),
        Commands.runOnce(
            () -> {
              squid.runPosition(100);
            },
            squid),
        new WaitCommand(1.5),
        Commands.runOnce(
            () -> {
              squid.runPosition(0.0);
            },
            squid));
  }
}
