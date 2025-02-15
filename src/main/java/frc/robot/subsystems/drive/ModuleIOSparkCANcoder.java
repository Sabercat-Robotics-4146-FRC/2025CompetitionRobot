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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.SwerveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.AutoConstants;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSparkCANcoder implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final CANcoder cancoder;
  private final boolean turnInverted;
  private final boolean turnEncoderInverted;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSparkCANcoder(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> new Rotation2d(kFLEncoderOffset);
          case 1 -> new Rotation2d(kFREncoderOffset);
          case 2 -> new Rotation2d(kBLEncoderOffset);
          case 3 -> new Rotation2d(kBREncoderOffset);
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> kFLDriveMotorId;
              case 1 -> kFRDriveMotorId;
              case 2 -> kBLDriveMotorId;
              case 3 -> kBRDriveMotorId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFLSteerMotorId;
              case 1 -> kFRSteerMotorId;
              case 2 -> kBLSteerMotorId;
              case 3 -> kBRSteerMotorId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnInverted =
        switch (module) {
          case 0 -> kFLSteerInvert;
          case 1 -> kFRSteerInvert;
          case 2 -> kBLSteerInvert;
          case 3 -> kBRSteerInvert;
          default -> false;
        };
    turnEncoderInverted =
        switch (module) {
          case 0 -> kFLEncoderInvert;
          case 1 -> kFREncoderInvert;
          case 2 -> kBLEncoderInvert;
          case 3 -> kBREncoderInvert;
          default -> false;
        };
    driveEncoder = driveSpark.getEncoder();
    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> kFLEncoderId;
              case 1 -> kFREncoderId;
              case 2 -> kBLEncoderId;
              case 3 -> kBREncoderId;
              default -> 0;
            },
            kCANbusName);
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) kDriveCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            AutoConstants.kPPdrivePID.kP,
            AutoConstants.kPPdrivePID.kI,
            AutoConstants.kPPdrivePID.kD,
            0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) SwerveConstants.kSteerCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(
            AutoConstants.kPPsteerPID.kP,
            AutoConstants.kPPsteerPID.kI,
            AutoConstants.kPPsteerPID.kD,
            0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create turn status signals
    turnVelocity = cancoder.getVelocity();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = cancoder.getPosition();

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(cancoder.getPosition());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh all signals
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        SwerveConstants.kDriveFrictionVoltage * Math.signum(velocityRadPerSec)
            + driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }
}
