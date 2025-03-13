package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class LinearActuatorExtendCommand extends Command {
  private final Indexer linearActuator;

  public LinearActuatorExtendCommand(Indexer linearActuator) {
    this.linearActuator = linearActuator;
  }

  @Override
  public void execute() {
    linearActuator.setExtended(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
