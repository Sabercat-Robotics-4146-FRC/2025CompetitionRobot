// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024-2025 Cross The Road Electronics
// https://github.com/CrossTheRoadElec/Phoenix6-Examples
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.hal.HAL;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FusedCANcoderTests {
  final double SET_DELTA = 0.1;
  final int CONFIG_RETRY_COUNT = 5;

  TalonFX talon;
  CANcoder cancoder;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    talon = new TalonFX(0);
    cancoder = new CANcoder(0);
  }

  @Test
  public void testIndividualPos() {
    final double TALON_POSITION = 0.4;
    final double CANCODER_POSITION = -3.1;

    /* Factory-default Talon and CANcoder */
    retryConfigApply(() -> talon.getConfigurator().apply(new TalonFXConfiguration()));
    retryConfigApply(() -> cancoder.getConfigurator().apply(new CANcoderConfiguration()));

    /* Get sim states */
    var talonSimState = talon.getSimState();
    var cancoderSimState = cancoder.getSimState();

    /* Wait for signal to update and assert they match the set positions */
    var talonPos = talon.getPosition();
    var cancoderPos = cancoder.getPosition();

    /* Make sure both are initially set to 0 before messing with sim state */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(0));
    retryConfigApply(() -> cancoderSimState.setRawPosition(0));
    retryConfigApply(() -> talon.setPosition(0));
    retryConfigApply(() -> cancoder.setPosition(0));
    /* Wait for sets to take affect */
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    /* Set them to different values */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(TALON_POSITION));
    retryConfigApply(() -> cancoderSimState.setRawPosition(CANCODER_POSITION));

    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    System.out.println("Talon Pos vs expected: " + talonPos + " vs " + TALON_POSITION);
    System.out.println("CANcoder Pos vs expected: " + cancoderPos + " vs " + CANCODER_POSITION);
    assertEquals(talonPos.getValue(), TALON_POSITION, SET_DELTA);
    assertEquals(cancoderPos.getValue(), CANCODER_POSITION, SET_DELTA);
  }

  @Test
  public void testFusedCANcoderPos() {
    final double TALON_POSITION = 0.4;
    final double CANCODER_POSITION = -3.1;

    /* Configure Talon to use Fused CANcoder, factory default CANcoder */
    var configs = new TalonFXConfiguration();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    retryConfigApply(() -> talon.getConfigurator().apply(configs));
    retryConfigApply(() -> cancoder.getConfigurator().apply(new CANcoderConfiguration()));

    /* Get sim states */
    var talonSimState = talon.getSimState();
    var cancoderSimState = cancoder.getSimState();

    /* Wait for signal to update and assert they match the set positions */
    var talonPos = talon.getPosition();
    var cancoderPos = cancoder.getPosition();

    /* Make sure both are initially set to 0 before messing with sim state */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(0));
    retryConfigApply(() -> cancoderSimState.setRawPosition(0));
    retryConfigApply(() -> talon.setPosition(0));
    retryConfigApply(() -> cancoder.setPosition(0));
    /* Wait for sets to take affect */
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    /* Set them to different values */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(TALON_POSITION));
    retryConfigApply(() -> cancoderSimState.setRawPosition(CANCODER_POSITION));

    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    /* Further wait for Talon, since it probably just received the new CANcoder position frame */
    talonPos.waitForUpdate(1.0);

    /* Make sure Talon matches CANcoder, since it should be using CANcoder's position */
    System.out.println("Talon Pos vs expected: " + talonPos + " vs " + CANCODER_POSITION);
    System.out.println("CANcoder Pos vs expected: " + cancoderPos + " vs " + CANCODER_POSITION);
    assertEquals(talonPos.getValue(), CANCODER_POSITION, SET_DELTA);
    assertEquals(cancoderPos.getValue(), CANCODER_POSITION, SET_DELTA);
  }

  @Test
  public void testRemoteCANcoderPos() {
    final double TALON_POSITION = 0.4;
    final double CANCODER_POSITION = -3.1;

    /* Configure Talon to use Fused CANcoder, factory default CANcoder */
    var configs = new TalonFXConfiguration();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    configs.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    retryConfigApply(() -> talon.getConfigurator().apply(configs));
    retryConfigApply(() -> cancoder.getConfigurator().apply(new CANcoderConfiguration()));

    /* Get sim states */
    var talonSimState = talon.getSimState();
    var cancoderSimState = cancoder.getSimState();

    /* Wait for signal to update and assert they match the set positions */
    var talonPos = talon.getPosition();
    var cancoderPos = cancoder.getPosition();

    /* Make sure both are initially set to 0 before messing with sim state */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(0));
    retryConfigApply(() -> cancoderSimState.setRawPosition(0));
    retryConfigApply(() -> talon.setPosition(0));
    retryConfigApply(() -> cancoder.setPosition(0));
    /* Wait for sets to take affect */
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    /* Set them to different values */
    retryConfigApply(() -> talonSimState.setRawRotorPosition(TALON_POSITION));
    retryConfigApply(() -> cancoderSimState.setRawPosition(CANCODER_POSITION));

    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);
    BaseStatusSignal.waitForAll(1.0, talonPos, cancoderPos);

    /* Further wait for Talon, since it probably just received the new CANcoder position frame */
    talonPos.waitForUpdate(1.0);

    /* Make sure Talon matches CANcoder, since it should be using CANcoder's position */
    System.out.println("Talon Pos vs expected: " + talonPos + " vs " + CANCODER_POSITION);
    System.out.println("CANcoder Pos vs expected: " + cancoderPos + " vs " + CANCODER_POSITION);
    assertEquals(talonPos.getValue(), CANCODER_POSITION, SET_DELTA);
    assertEquals(cancoderPos.getValue(), CANCODER_POSITION, SET_DELTA);
  }

  private void retryConfigApply(Supplier<StatusCode> toApply) {
    StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
    int triesLeftOver = CONFIG_RETRY_COUNT;
    do {
      finalCode = toApply.get();
    } while (!finalCode.isOK() && --triesLeftOver > 0);
    assert (finalCode.isOK());
  }
}
