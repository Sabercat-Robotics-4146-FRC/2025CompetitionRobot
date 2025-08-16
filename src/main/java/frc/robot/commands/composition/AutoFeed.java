package frc.robot.commands.composition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.alignment.AlignNearestFeederTag;
import frc.robot.commands.indexer.IntakeCommand;
import frc.robot.commands.indexer.LinearActuatorRetractCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;

public class AutoFeed extends SequentialCommandGroup {

  public AutoFeed(RobotContainer container, Drive drive, Indexer indexer, Elevator elevator) {
    addCommands(
        elevator.goHome(),
        new LinearActuatorRetractCommand(indexer),
        new AlignNearestFeederTag(drive, container),
        new IntakeCommand(indexer));
  }
}
