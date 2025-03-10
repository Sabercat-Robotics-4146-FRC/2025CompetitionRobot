package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSISubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends RBSISubsystem {
  // -- PID & FeedForward values -- //
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 200);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 5);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);
  private static final LoggedTunableNumber maxSoftCurrent =
      new LoggedTunableNumber("Elevator/maxSoftCurrent", 80);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator Motor Disconnected!", Alert.AlertType.kWarning);

  // -- Important logged Booleans -- //
  @AutoLogOutput private boolean brakeModeEnabled = true;
  @AutoLogOutput private boolean manualOverride = true;

  @AutoLogOutput private boolean homed = false;
  @AutoLogOutput private boolean homing = false;

  @AutoLogOutput private boolean exceedsMaxCurrent = false;

  // -- Limit Switch Stuff -- //
  private final DigitalInput limitSwitch = new DigitalInput(2);
  private final Debouncer limitDebouncer = new Debouncer(0.1);

  // -- Position Handling -- //
  public enum ElevatorPosition {
    STOWED,
    L1,
    L2,
    L3,
    L4
  }

  @AutoLogOutput private ElevatorPosition elevatorDesiredPosition = ElevatorPosition.STOWED;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setPID(kP.get(), 0.0, kD.get(), kG.get(), kV.get(), kA.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    System.out.println(io.getPosition());
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set((!inputs.motorConnected));

    // -- Update Logged Tunable Numbers -- //
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get(), kG.get(), kV.get(), kA.get());
    }

    // -- Zero Position Upon Hitting Limit -- //
    if (!limitDebouncer.calculate(limitSwitch.get()) && io.getPosition() != 0.0) {
      io.zeroPosition();
      elevatorDesiredPosition = ElevatorPosition.STOWED;
    }

    // TODO: Soft Current limits to keep the robot from destroying itself
    if (io.getCurrent() > maxSoftCurrent.get()) {
      System.out.println("Current Above Max for elevator");
      exceedsMaxCurrent = true;
      io.stop();
    } else {
      exceedsMaxCurrent = false;
    }
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void zeroPosition() {
    io.zeroPosition();
    System.out.println("Zeroed Position, new pose is: " + io.getPosition());
  }

  public Command goHome() {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              homing = true;
              homed = false;
            },
            this),
        Commands.repeatingSequence(Commands.runOnce(() -> runVolts(-2)))
            .until(() -> limitSwitch.get())
            .andThen(
                () -> {
                  homed = true;
                  homing = false;
                  elevatorDesiredPosition = ElevatorPosition.STOWED;
                },
                this));
  }

  public void runPosition(double position) {
    System.out.println(
        "Setting Position to:" + position + " Current position: " + inputs.positionRad);
    io.runPosition(position, 100);
  }

  // -- Default Method - Constantly Update Position -- //
  public void updatePosition(DoubleSupplier manualVolts) {
    if (!homing && homed && !manualOverride && !exceedsMaxCurrent) {
      switch (elevatorDesiredPosition) {
        case STOWED:
          if (limitSwitch.get()) {
            runPosition(0);
          }
          break;
        case L4:
          runPosition(99);
          break;
        case L3:
          runPosition(50);
          break;
        case L2:
          runPosition(25);
          break;
      }
    } else if (manualOverride) {
      runVolts(manualVolts.getAsDouble());
    }
  }

  public ElevatorPosition getDesiredPosition() {
    return elevatorDesiredPosition;
  }

  // -- Set the setpoint of the Elevator -- //
  public void setDesiredPosition(ElevatorPosition pose) {
    elevatorDesiredPosition = pose;
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void setManualOverRide(boolean override) {
    manualOverride = override;
  }
}
