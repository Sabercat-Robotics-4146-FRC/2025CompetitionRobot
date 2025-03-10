package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class LinearActuatorRetractCommand extends Command {
  private final Indexer linearActuator;

  public LinearActuatorRetractCommand(Indexer linearActuator) {
    this.linearActuator = linearActuator;
  }

  @Override
  public void execute() {
    linearActuator.retractLinearActuator();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
