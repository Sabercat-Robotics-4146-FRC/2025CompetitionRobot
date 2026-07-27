package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.elevator.RunElevatorExplicit;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreNoAlign extends SequentialCommandGroup {
  public ScoreNoAlign(Elevator elevator, Indexer indexer, RobotContainer container) {
    addCommands(
        elevator.goHome(),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new RunIndexer(indexer),
        new WaitCommand(1.5),
        new StopIndexerCommand(indexer),
        new RunElevatorExplicit(elevator, 100),
        new WaitCommand(0.2),
        new RunElevatorExplicit(elevator, 0.5),
        elevator.goHome());
  }
}
