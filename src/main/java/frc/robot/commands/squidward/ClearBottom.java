package frc.robot.commands.squidward;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.squidward.Squidward;

public class ClearBottom extends SequentialCommandGroup {
  public ClearBottom(Squidward squid, Drive drive, RobotContainer container) {
    addCommands(
        new WaitCommand(1),
        Commands.runOnce(
            () -> {
              squid.runPosition(-120.0);
            },
            squid),
        new WaitCommand(3),
        Commands.runOnce(
            () -> {
              squid.runPosition(0.0);
            },
            squid));
  }
}
