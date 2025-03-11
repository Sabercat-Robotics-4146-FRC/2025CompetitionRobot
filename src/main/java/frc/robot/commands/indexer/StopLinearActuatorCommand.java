package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class StopLinearActuatorCommand extends Command {
  private final Indexer linearActuator;

  public StopLinearActuatorCommand(Indexer linearActuator) {
    this.linearActuator = linearActuator;
  }

  @Override
  public void execute() {
    linearActuator.stopLinearActuator();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
