package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignNearestTag;
import frc.robot.commands.indexer.IntakeCommand;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class Feed extends SequentialCommandGroup {
  public Feed(RobotContainer container, Drive drive, Indexer indexer, Elevator elevator) {
    addCommands(
        elevator.goHome(),
        new AlignNearestTag(drive, container),
        new ParallelCommandGroup(new IntakeCommand(indexer)),
        new StopIndexerCommand(indexer));
  }
}
