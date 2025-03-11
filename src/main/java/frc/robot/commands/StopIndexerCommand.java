package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class StopIndexerCommand extends Command {

  private final Indexer indexer;

  public StopIndexerCommand(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.stopVoltage();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
