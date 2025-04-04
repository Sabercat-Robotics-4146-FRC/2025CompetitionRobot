package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class RunIndexer extends Command {
  private final Indexer indexer;

  public RunIndexer(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.runVoltage(6);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
