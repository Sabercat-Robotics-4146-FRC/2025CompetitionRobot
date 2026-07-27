package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.indexer.Indexer;

public class TestIndexer extends SequentialCommandGroup {
  public TestIndexer(Indexer indexer) {
    addCommands(new WaitCommand(2), new RunIndexer(indexer));
  }
}
