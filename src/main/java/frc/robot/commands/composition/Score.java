package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignNearestTag;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class Score extends SequentialCommandGroup {
  public Score(Elevator elevator, Indexer indexer, Drive drive, RobotContainer container) {
    addCommands(
        elevator.goHome(),
        new AlignNearestTag(drive, container),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new RunIndexer(indexer),
        new WaitCommand(1),
        new StopIndexerCommand(indexer),
        elevator.goHome());
  }
}
