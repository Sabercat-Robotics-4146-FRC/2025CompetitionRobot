package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorCommand extends Command {
  Elevator elevator;

  public RunElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setDesiredPosition(elevator.getSelectedPosition());
  }

  @Override
  public boolean isFinished() {
    return elevator.getAtDesiredPose();
  }
}
