// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel_example;
/*


public class FlywheelIOTalonFX implements FlywheelIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final TalonFX leader =
      new TalonFX(
          CANandPowerPorts.FLYWHEEL_LEADER.getDeviceNumber(),
          CANandPowerPorts.FLYWHEEL_LEADER.getBus());
  private final TalonFX follower =
      new TalonFX(
          CANandPowerPorts.FLYWHEEL_FOLLOWER.getDeviceNumber(),
          CANandPowerPorts.FLYWHEEL_FOLLOWER.getBus());
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {
    CANandPowerPorts.FLYWHEEL_LEADER.getPowerPort(),
    CANandPowerPorts.FLYWHEEL_FOLLOWER.getPowerPort()
  };

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kFlywheelIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    // If follower rotates in the opposite direction, set "OpposeMasterDirection" to true
    follower.setControl(new Follower(leader.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / kFlywheelGearRatio;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    leader.getConfigurator().apply(config);
  }
}
  */
