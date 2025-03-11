package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// new Indexer class that extends RBSI subsystembase class
public class Indexer extends RBSISubsystem {

  private final IndexerIOTalonFX io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final DigitalInput irSensor = new DigitalInput(CANandPowerPorts.IR_SENSOR);
  private final Servo linearActuator = new Servo(CANandPowerPorts.LINEAR_ACTUATOR);
  private double indexerVoltageOne = IndexerConstants.indexerVoltageOne;
  private final double indexerVoltageTwo = IndexerConstants.indexerVoltageTwo;
  private final double linearActuatorExtend = IndexerConstants.linearActuatorExtend;
  private final double linearActuatorRetract = IndexerConstants.linearActuatorRetract;
  private final Debouncer limitDebouncer = new Debouncer(0.2);

  // Indexer constructor which takes in IndexerIOTalonFX object
  public Indexer(IndexerIOTalonFX io) {
    this.io = io;
    linearActuator.setBoundsMicroseconds(2000, 1, 1500, 0, 1000);
  }

  // returns state of IR sensor
  public boolean hasGamePiece() {
    return !limitDebouncer.calculate(irSensor.get());
  }

  // run indexer motor voltage (run forward)
  public void runVoltage() {
    io.setVoltage(indexerVoltageOne);
    System.out.println("the motor is running");
  }

  // emergency indexer voltage (run backword)
  public void runVoltageBackword() {
    io.setVoltage(indexerVoltageTwo);
    System.out.println("the motor is running backword");
  }

  // stop indexer motor
  public void stopVoltage() {
    io.stop();
    System.out.println("The motor is stopping");
  }

  // extends the linear actuator
  public void extendLinearActuator() {
    linearActuator.setSpeed(linearActuatorExtend);
    System.out.println("Actuator is running at " + linearActuator.getSpeed());
  }

  // retracts the linear actuator
  public void retractLinearActuator() {
    linearActuator.setSpeed(linearActuatorRetract);
  }

  // stop the linear actuator
  public void stopLinearActuator() {
    linearActuator.setDisabled();
    System.out.println("Linear actuator has stopped");
  }

  // log velocity rpm on advantage scope
  @AutoLogOutput(key = "Mechanism/Indexer")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  // log current on advantage scope
  public double getCurrent() {
    return inputs.currentAmps[inputs.currentAmps.length - 1];
  }

  // periodic to update inputs and process inputs onto USB in real robot
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
