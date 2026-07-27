package frc.robot.subsystems.squidward;

import static frc.robot.Constants.RobotDesiredPositions.RIGHT_REEFS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants.AlgaeLevel;
import frc.robot.Constants.RobotDesiredPositions.DesiredPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSISubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Squidward extends RBSISubsystem {
  // -- PID & FeedForward values -- //
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Squidward/kP", 1.0);
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
  DoubleSupplier manualVolts;
  Drive drive;

  public Squidward(SquidwardIO io, DoubleSupplier manualVolts) {
    this.io = io;
    io.setPID(kP.get(), kI.get(), kD.get(), kG.get(), kV.get(), kA.get(), kS.get());

    this.manualVolts = manualVolts;
    io.zeroPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
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

    if (inputs.currentAmps > maxSoftCurrent.get()) {
      System.out.println("Current Above Max for squidward");
      exceedsMaxCurrent = true;
      io.stop();
    } else {
      exceedsMaxCurrent = false;
    }
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void runPercent(double percent) {
    io.runPercent(percent);
  }

  public void zeroPosition() {
    io.zeroPosition();
  }

  public boolean getManualOverride() {
    return manualOverride;
  }

  public void runPosition(double position) {
    io.runPosition(position, 0);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public AlgaeLevel getNearestAlgaeLevel(Pose2d pose) {
    double shortestDistance = 1000;
    AlgaeLevel shortestLevel = null;
    for (DesiredPosition reef : RIGHT_REEFS) {
      double distance = pose.getTranslation().getDistance(reef.pose.getTranslation());
      if (distance < shortestDistance) {
        shortestDistance = distance;
        shortestLevel = reef.algaeLevel;
      }
    }
    return shortestLevel;
  }
}
