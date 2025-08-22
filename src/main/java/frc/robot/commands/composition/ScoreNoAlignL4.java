package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.RunElevatorCommand;
import frc.robot.commands.elevator.RunElevatorExplicit;
import frc.robot.commands.indexer.ScoreCoralL4;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreNoAlignL4 extends SequentialCommandGroup{
 public ScoreNoAlignL4(Elevator elevator, Indexer indexer, RobotContainer container) {
    addCommands(
        elevator.goHome(),
        new RunElevatorCommand(elevator),
        new WaitUntilCommand(() -> elevator.getAtDesiredPose()),
        new ScoreCoralL4(indexer),
        new RunElevatorExplicit(elevator, 100),
        new WaitCommand(0.2),
        new RunElevatorExplicit(elevator, 0.5),
        elevator.goHome());
  }
}
