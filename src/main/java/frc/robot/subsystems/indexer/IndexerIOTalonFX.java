package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.CANandPowerPorts;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor =
      new TalonFX(
          CANandPowerPorts.INDEXER_MOTOR.getDeviceNumber(),
          CANandPowerPorts.INDEXER_MOTOR.getBus());
  private final Servo linearActuator = new Servo(CANandPowerPorts.INDEXER_SERVO);

  public final int[] powerPorts = {
    CANandPowerPorts.INDEXER_MOTOR.getPowerPort(),
  };

  private final StatusSignal<Angle> motorPosition = motor.getPosition();
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity();
  private final StatusSignal<Voltage> motorAppliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent();

  public IndexerIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode =
        switch (IndexerConstants.kIndexerIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(motorPosition.getValueAsDouble())
            / IndexerConstants.kIndexerGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(motorVelocity.getAppliedUpdateFrequency())
            / IndexerConstants.kIndexerGearRatio;
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {motorCurrent.getValueAsDouble(), motorCurrent.getValueAsDouble()};
    inputs.servoPosition = linearActuator.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setServo(double position) {
    linearActuator.setPosition(position);
  }

  @Override
  public void stop() {
    motor.stopMotor();
    linearActuator.setSpeed(0);
  }

  @Override
  public void conifgurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motor.getConfigurator().apply(config);
  }
}
