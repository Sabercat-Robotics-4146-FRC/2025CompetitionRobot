package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreCoral extends Command {
  private Indexer indexer;

  public ScoreCoral(Indexer indexer) {
    this.indexer = indexer;
  }

  @Override
  public void execute() {
    indexer.runVoltage(6);
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
