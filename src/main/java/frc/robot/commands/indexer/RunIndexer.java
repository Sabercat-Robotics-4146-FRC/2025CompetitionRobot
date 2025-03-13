package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class RunIndexer extends Command {
  private final Indexer indexer;
  private Timer timer;

  public RunIndexer(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.runVoltage();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
