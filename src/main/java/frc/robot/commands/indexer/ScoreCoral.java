package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreCoral extends Command {
  private Indexer indexer;
  private boolean startingState;
  private Elevator elevator;

  public ScoreCoral(Indexer indexer, Elevator elevator) {
    this.indexer = indexer;
    this.elevator = elevator;
  }

  @Override
  public void execute() {
      if (elevator.getDesiredPosition() == ElevatorPosition.L2) {
        indexer.runVoltage(1.0);
      } else if (elevator.getDesiredPosition() == ElevatorPosition.L4) {
        indexer.runVoltage(1.5);
      } else {
        indexer.runVoltage(2.5);
      }
  }

  @Override
  public boolean isFinished() {
    return !(indexer.hasGamePiece());
  }

  /* 
  @Override
  public void end(boolean interrupted) {
    indexer.stopVoltage();
  }
    */
  
}
