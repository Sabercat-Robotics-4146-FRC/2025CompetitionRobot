package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestLeftReef;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.elevator.RunElevatorExplicit;
import frc.robot.commands.indexer.ScoreIndexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class AutoScoreLeft extends SequentialCommandGroup {
  public AutoScoreLeft(Elevator elevator, Indexer indexer, Drive drive, RobotContainer container) {
    addCommands(
        elevator.goHome(),
        new AlignNearestLeftReef(drive, container),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new WaitCommand(0.2),
        new ScoreIndexer(indexer),
        new RunElevatorExplicit(elevator, 100),
        new WaitCommand(0.2),
        new RunElevatorExplicit(elevator, 0.5),
        elevator.goHome());
  }
}
