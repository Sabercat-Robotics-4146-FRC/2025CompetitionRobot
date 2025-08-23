package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class RunIndexer extends Command {
  private final Indexer indexer;
  private double voltage;
  private double speed;

  public RunIndexer(Indexer indexer, double voltage, double speed) {
    this.indexer = indexer;
    this.voltage = voltage;
    this.speed = speed;
  }

  @Override
  public void execute() {
    indexer.runVoltage(voltage);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
