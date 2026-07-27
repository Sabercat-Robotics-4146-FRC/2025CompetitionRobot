package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class RunElevatorExplicit extends Command {
  Elevator elevator;
  double position;

  public RunElevatorExplicit(Elevator elevator, double positon) {
    this.elevator = elevator;
    this.position = positon;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setExplicitPosition(position);
    elevator.setDesiredPosition(ElevatorPosition.EXPLICIT);
  }

  @Override
  public boolean isFinished() {
    return elevator.getAtDesiredPose();
  }
}
