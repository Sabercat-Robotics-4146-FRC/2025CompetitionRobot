package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestFeederTag;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class AutoFeed extends SequentialCommandGroup {

  public AutoFeed(RobotContainer container, Drive drive, Indexer indexer, Elevator elevator) {
    addCommands(
        elevator.goHome(),
        new AlignNearestFeederTag(drive, container),
        new WaitCommand(2),
        Commands.runOnce(
            () -> {
              indexer.runVoltage(2.2);
            },
            indexer),
        new WaitCommand(0.8),
        new StopIndexerCommand(indexer));
  }
}
