package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class RunIndexerBackwordCommand extends Command {
  private final Indexer indexer;

  public RunIndexerBackwordCommand(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.runVoltageBackword();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
