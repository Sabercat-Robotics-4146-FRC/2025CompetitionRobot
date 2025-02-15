package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
  // -- BE SURE TO SET REDUCTION UPON PHYSICAL ROBOT COMPLETION!!! -- //
  public static final double reduction = 4.0;

  private final TalonFX leader;
  private final TalonFX follower;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer debouncer = new Debouncer(0.5);

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public ElevatorIOTalonFX() {
    leader = new TalonFX(20, "");
    follower = new TalonFX(21, "");
    follower.setControl(new Follower(leader.getDeviceID(), true));

    // -- All Sorts of Configuration -- //
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio =
        reduction; // -- Wondering if this is necessarry with out appraoch -- //
    config.TorqueCurrent.withPeakForwardTorqueCurrent(80.0);
    config.TorqueCurrent.withPeakReverseTorqueCurrent(-80.0);
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    torqueCurrent = leader.getTorqueCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();

    followerAppliedVolts = follower.getMotorVoltage();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0, position, velocity, appliedVolts, torqueCurrent, temp);
      ParentDevice.optimizeBusUtilizationForAll(leader);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp).isOK();
    boolean followerConnected = BaseStatusSignal.refreshAll(followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp).isOK();

    inputs.motorConnected = debouncer.calculate(connected);
    inputs.followerConnected = debouncer.calculate(followerConnected);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = new double[] {appliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
    inputs.torqueCurrentAmps = new double[] {torqueCurrent.getValueAsDouble(), followerTorqueCurrent.getValueAsDouble()};
    inputs.supplyCurrentAmps = new double[] {supplyCurrent.getValueAsDouble(), supplyCurrent.getValueAsDouble()};
    inputs.tempCelsius = new double[] {temp.getValueAsDouble(), followerTemp.getValueAsDouble()};
  }

  @Override
  public void runOpenLoop(double output) {
    leader.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    leader.setControl(positionTorqueCurrentRequest.withPosition(Units.radiansToRotations(positionRad)).withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled){
    new Thread(
      () -> leader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)
      ).start();
  }
}
