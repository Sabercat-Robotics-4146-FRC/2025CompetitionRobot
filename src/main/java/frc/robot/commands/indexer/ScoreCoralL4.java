package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreCoralL4 extends Command {
  private Indexer indexer;

  public ScoreCoralL4(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.runVoltage(2.7);
  }

  @Override
  public boolean isFinished() {
    return !(indexer.hasGamePiece());
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopVoltage();
  }
}
