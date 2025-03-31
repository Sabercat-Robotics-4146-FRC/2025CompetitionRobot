package frc.robot.subsystems.squidward;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSISubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Squidward extends RBSISubsystem {
  // -- PID & FeedForward values -- //
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Squidward/kP", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Squidward/kD", 0.0);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Squidward/kG", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Squidward/kV", 0.0);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Squidward/kA", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Squidward/kS", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Squidward/kI", 0.0);

  private static final LoggedTunableNumber maxSoftCurrent =
      new LoggedTunableNumber("Squidward/maxSoftCurrent", 30);

  private final SquidwardIO io;
  private final SquidwardIOInputsAutoLogged inputs = new SquidwardIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Squidward Motor Disconnected!", Alert.AlertType.kWarning);

  // -- Important logged Booleans -- //
  @AutoLogOutput private boolean brakeModeEnabled = true;
  @AutoLogOutput private boolean manualOverride = false;

  @AutoLogOutput private boolean exceedsMaxCurrent = false;

  // -- Limit Switch Stuff -- //
  @AutoLogOutput private SquidwardPostition selectedPose = SquidwardPostition.STOWED;

  // -- Position Handling -- //
  public enum SquidwardPostition {
    STOWED,
    UPPER,
    LOWER
  }

  public double stowedPosition = 0.0;
  public double UpperPosition = 40.0;
  public double LowerPosition = 40.0;

  DoubleSupplier manualVolts;

  @AutoLogOutput private SquidwardPostition armDesiredPosition = SquidwardPostition.STOWED;

  public Squidward(SquidwardIO io, DoubleSupplier manualVolts) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kG.get(), kV.get(), kA.get(), kS.get());

    this.manualVolts = manualVolts;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // System.out.println(inputs.positionRad);
    Logger.processInputs("Squidward", inputs);

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

    // TODO: Soft Current limits to keep the robot from destroying itself
    if (inputs.currentAmps > maxSoftCurrent.get()) {
      System.out.println("Current Above Max for squidward");
      exceedsMaxCurrent = true;
      io.stop();
    } else {
      exceedsMaxCurrent = false;
    }

    updatePosition(manualVolts);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  public boolean getManualOverride() {
    return manualOverride;
  }

  public void runPosition(double position) {
    System.out.println(
        "Setting Squid Position to:" + position + " Current Squid position: " + inputs.positionRad);
    io.runPosition(position, 0);
  }

  // -- Default Method - Constantly Update Position -- //
  public void updatePosition(DoubleSupplier manualVolts) {
    if (!manualOverride && !exceedsMaxCurrent) {
      switch (armDesiredPosition) {
        case STOWED:
          runPosition(stowedPosition);
          break;
        case UPPER:
          runPosition(UpperPosition);
          break;
        case LOWER:
          runPosition(LowerPosition);
          break;
      }
    } else if (manualOverride && !exceedsMaxCurrent) {
      runVolts(manualVolts.getAsDouble());
    }
  }

  public SquidwardPostition getDesiredPosition() {
    return armDesiredPosition;
  }

  public void setSelectedPosition(SquidwardPostition pose) {
    selectedPose = pose;
  }

  public SquidwardPostition getSelectedPosition() {
    return selectedPose;
  }

  // -- Set the setpoint of the Elevator -- //
  public void setDesiredPosition(SquidwardPostition pose) {
    armDesiredPosition = pose;
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
