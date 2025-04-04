package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ScoreSide;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestLeftReef;
import frc.robot.commands.alignment.AlignNearestRightReef;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.elevator.RunElevatorExplicit;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import java.util.function.Supplier;

public class Score extends SequentialCommandGroup {
  public Score(
      Elevator elevator,
      Indexer indexer,
      Drive drive,
      RobotContainer container,
      Supplier<ScoreSide> side) {
    addCommands(
        elevator.goHome(),
        new ConditionalCommand(
            new AlignNearestLeftReef(drive, container),
            new AlignNearestRightReef(drive, container),
            () -> side.get() == ScoreSide.LEFT),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new RunIndexer(indexer),
        new WaitCommand(1.5),
        new StopIndexerCommand(indexer),
        new RunElevatorExplicit(elevator, 100),
        new WaitCommand(0.2),
        new RunElevatorExplicit(elevator, 0.0),
        elevator.goHome());
  }
}
