package frc.robot.subsystems.squidward;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PhoenixUtil;

public class SquidwardIOTalonFX implements SquidwardIO {
  // -- BE SURE TO SET REDUCTION UPON PHYSICAL ROBOT COMPLETION!!! -- //
  public static final double reduction = ElevatorConstants.kElevatorGearRatio;

  private final TalonFX leader;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer debouncer = new Debouncer(0.5);

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public SquidwardIOTalonFX() {
    leader =
        new TalonFX(
            CANandPowerPorts.SQUIDWARD_MOTOR.getDeviceNumber(),
            CANandPowerPorts.SQUIDWARD_MOTOR.getBus());

    // -- All Sorts of Configuration -- //
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    // config.Feedback.SensorToMechanismRatio =
    //     reduction; // -- Wondering if this is necessarry with out appraoch -- //
    config.TorqueCurrent.withPeakForwardTorqueCurrent(40.0);
    config.TorqueCurrent.withPeakReverseTorqueCurrent(-40.0);
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10;
    motionMagicConfigs.MotionMagicAcceleration = 10;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    // follower.setControl(new Follower(leader.getDeviceID(), true));

    position = leader.getPosition();

    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    torqueCurrent = leader.getTorqueCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, torqueCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(leader);

    motionMagicVoltage.EnableFOC = true;
  }

  @Override
  public void updateInputs(SquidwardIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();
    inputs.motorConnected = debouncer.calculate(connected);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = new double[] {temp.getValueAsDouble()};
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
    leader.setControl(
        motionMagicVoltage.withPosition(Units.radiansToRotations(positionRad)).withFeedForward(0));
  }

  @Override
  public void zeroPosition() {
    leader.setPosition(0.0);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kG, double kV, double kA, double kS) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kS = kS;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> leader.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
