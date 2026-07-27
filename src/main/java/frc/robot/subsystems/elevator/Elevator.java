package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSISubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends RBSISubsystem {
  // -- PID & FeedForward values -- //
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.55);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.84);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);

  private static final LoggedTunableNumber maxSoftCurrent =
      new LoggedTunableNumber("Elevator/maxSoftCurrent", 80);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator Motor Disconnected!", Alert.AlertType.kWarning);

  // -- Important logged Booleans -- //
  @AutoLogOutput private boolean brakeModeEnabled = true;
  @AutoLogOutput private boolean manualOverride = false;

  @AutoLogOutput private boolean homed = false;
  @AutoLogOutput private boolean homing = false;

  @AutoLogOutput private boolean exceedsMaxCurrent = false;
  @AutoLogOutput private boolean loggedLimitSwitch = false;

  SysIdRoutine sysId;

  // -- Limit Switch Stuff -- //
  private final DigitalInput limitSwitch = new DigitalInput(2);
  private final Debouncer limitDebouncer = new Debouncer(0.1);
  @AutoLogOutput private boolean atDesiredPose = false;
  @AutoLogOutput private ElevatorPosition selectedPose = ElevatorPosition.L2;

  // -- Position Handling -- //
  public enum ElevatorPosition {
    STOWED,
    L1,
    L2,
    L3,
    L4,
    EXPLICIT
  }

  public double heightL1 = 20.0;
  public double heightL2 = 40.0;
  public double heightL3 = 79.0;
  public double heightL4 = 146.0;

  private double explicitPosition = 5.0;

  DoubleSupplier manualVolts;

  @AutoLogOutput private ElevatorPosition elevatorDesiredPosition = ElevatorPosition.STOWED;

  public Elevator(ElevatorIO io, DoubleSupplier manualVolts) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kG.get(), kV.get(), kA.get(), kS.get());

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volt)), null, this));

    sysId.quasistatic(SysIdRoutine.Direction.kForward);
    sysId.quasistatic(SysIdRoutine.Direction.kReverse);
    sysId.dynamic(SysIdRoutine.Direction.kForward);
    sysId.dynamic(SysIdRoutine.Direction.kReverse);

    this.manualVolts = manualVolts;
  }

  public Command getQuasiForward() {
    return sysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command getQuasiReverse() {
    return sysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command getDynamicForward() {
    return sysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command getDynamicReverse() {
    return sysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // System.out.println(inputs.positionRad);
    Logger.processInputs("Elevator", inputs);
    loggedLimitSwitch = limitSwitch.get();

    motorDisconnectedAlert.set((!inputs.motorConnected));

    // -- Update Logged Tunable Numbers -- //
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kI.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kG.get(), kV.get(), kA.get(), kS.get());
    }

    // -- Zero Position Upon Hitting Limit -- //
    if (limitDebouncer.calculate(limitSwitch.get())
        && io.getPosition() != 0.0
        && inputs.velocityRadPerSec < 0) {
      io.zeroPosition();
      // elevatorDesiredPosition = ElevatorPosition.STOWED;
    }

    // TODO: Soft Current limits to keep the robot from destroying itself
    if (io.getCurrent() > maxSoftCurrent.get()) {
      System.out.println("Current Above Max for elevator");
      exceedsMaxCurrent = true;
      io.stop();
    } else {
      exceedsMaxCurrent = false;
    }

    if (elevatorDesiredPosition == ElevatorPosition.L3) {
      if (inputs.positionRad > heightL3 - 5 && inputs.positionRad < heightL3 + 5) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }
    if (elevatorDesiredPosition == ElevatorPosition.L1) {
      if (inputs.positionRad > heightL1 - 5 && inputs.positionRad < heightL1 + 5) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }
    if (elevatorDesiredPosition == ElevatorPosition.L2) {
      if (inputs.positionRad > heightL2 - 5 && inputs.positionRad < heightL2 + 5) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }
    if (elevatorDesiredPosition == ElevatorPosition.L4) {
      if (inputs.positionRad > heightL4 - 10 && inputs.positionRad < heightL4) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }
    if (elevatorDesiredPosition == ElevatorPosition.STOWED) {
      if (inputs.positionRad > -5 && inputs.positionRad < 5) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }
    if (elevatorDesiredPosition == ElevatorPosition.EXPLICIT) {
      if (inputs.positionRad > explicitPosition - 5 && inputs.positionRad < explicitPosition + 5) {
        atDesiredPose = true;
      } else {
        atDesiredPose = false;
      }
    }

    if (!homing) {
      updatePosition(manualVolts);
    }
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void zeroPosition() {
    io.zeroPosition();
    System.out.println("Zeroed Position, new pose is: " + io.getPosition());
  }

  public boolean getManualOverride() {
    return manualOverride;
  }

  public Command goHome() {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              homing = true;
              homed = false;
            },
            this),
        Commands.repeatingSequence(Commands.runOnce(() -> runVolts(-0.2)))
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
    io.runPosition(position, 0);
  }

  public boolean getAtDesiredPose() {
    return atDesiredPose;
  }

  // -- Default Method - Constantly Update Position -- //
  public void updatePosition(DoubleSupplier manualVolts) {
    if (!homing && homed && !manualOverride && !exceedsMaxCurrent) {
      switch (elevatorDesiredPosition) {
        case STOWED:
          if (!limitSwitch.get()) {
            runPosition(0);
          }
          break;
        case L4:
          runPosition(heightL4);
          break;
        case L3:
          runPosition(heightL3);
          break;
        case L2:
          runPosition(heightL2);
          break;
        case L1:
          runPosition(heightL1);
          break;
        case EXPLICIT:
          runPosition(explicitPosition);
          break;
      }
    } else if (manualOverride && !exceedsMaxCurrent) {
      runVolts(manualVolts.getAsDouble());
    }
  }

  public ElevatorPosition getDesiredPosition() {
    return elevatorDesiredPosition;
  }

  public void setSelectedPosition(ElevatorPosition pose) {
    selectedPose = pose;
  }

  public ElevatorPosition getSelectedPosition() {
    return selectedPose;
  }

  public void setHoming(boolean state) {
    homing = state;
  }

  // -- Set the setpoint of the Elevator -- //
  public void setDesiredPosition(ElevatorPosition pose) {
    elevatorDesiredPosition = pose;
  }

  public Double enumToPosition(ElevatorPosition e) {
    if (e == ElevatorPosition.L1) {
      return heightL1;
    }
    if (e == ElevatorPosition.L2) {
      return heightL2;
    }
    if (e == ElevatorPosition.L3) {
      return heightL3;
    }
    if (e == ElevatorPosition.L4) {
      return heightL4;
    }
    if (e == ElevatorPosition.STOWED) {
      return 0.0;
    }

    return 0.0;
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void setExplicitPosition(double position) {
    explicitPosition = position;
  }

  public void setManualOverRide(boolean override) {
    manualOverride = override;
  }
}
