package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestRightReef;
import frc.robot.commands.alignment.AlignNearestRightReefL4;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.elevator.RunElevatorExplicit;
import frc.robot.commands.indexer.ScoreCoral;
import frc.robot.commands.indexer.ScoreCoralL4;
import frc.robot.commands.indexer.StopIndexerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.indexer.Indexer;

public class AutoScoreRight extends SequentialCommandGroup {
  public AutoScoreRight(Elevator elevator, Indexer indexer, Drive drive, RobotContainer container) {
    addCommands(
        elevator.goHome(),
        new AlignNearestRightReef(drive, container),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new WaitCommand(0.2),
        new ConditionalCommand(
            new ScoreCoralL4(indexer),
            new ScoreCoral(indexer),
            () -> elevator.getSelectedPosition() == ElevatorPosition.L4),
        new WaitCommand(1.5),
        new StopIndexerCommand(indexer),
        new RunElevatorExplicit(elevator, 100),
        new WaitCommand(0.2),
        new RunElevatorExplicit(elevator, 0.5),
        elevator.goHome());
  }
}
