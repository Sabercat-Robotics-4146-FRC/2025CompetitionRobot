package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class ScoreIndexer extends Command{
private Indexer indexer;
private boolean startingState;

public ScoreIndexer(Indexer indexer){
  this.indexer = indexer; 
}

@Override
public void initialize(){
  startingState = indexer.hasGamePiece();

}

@Override
public void execute(){
  if(startingState = true){
  indexer.runVoltage(1.8);
  }

}

@Override
public boolean isFinished(){
return startingState = !(indexer.hasGamePiece());
}


@Override
public void end(boolean interrupted){
  indexer.stopVoltage();

}


  

  
}
