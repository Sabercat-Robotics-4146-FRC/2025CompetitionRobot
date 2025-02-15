// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) -2025 Cross The Road Electronics
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
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LatencyCompensationTests {
  final double DOUBLE_DELTA = 0.01;

  TalonFX talonfx;
  CANcoder cancoder;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    talonfx = new TalonFX(0);
    cancoder = new CANcoder(0);
  }

  @Test
  public void testLatencyCompensator() {
    final double position = 35;
    final double velocity = 24;
    /* Initialize by making all the positions 0 */
    talonfx.setPosition(0);
    cancoder.setPosition(0);

    /* Set the simulated state of device positions */
    talonfx.getSimState().setRawRotorPosition(position);
    talonfx.getSimState().setRotorVelocity(velocity);
    cancoder.getSimState().setRawPosition(position);
    cancoder.getSimState().setVelocity(velocity);

    /* Perform latency compensation */
    /* Start by getting signals */
    var talonPos = talonfx.getPosition();
    var talonVel = talonfx.getVelocity();
    var cancoderPos = cancoder.getPosition();
    var cancoderVel = cancoder.getVelocity();

    /* Wait for an update on all of them so they're synchronized */
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < 5; ++i) {
      System.out.println("Waiting on signals");
      status = BaseStatusSignal.waitForAll(1, talonPos, talonVel, cancoderPos, cancoderVel);
      if (status.isOK()) break;
    }
    assertTrue(status.isOK());

    /* Wait a bit longer for the latency to actually do some work */
    try {
      Thread.sleep(10);
    } catch (Exception ex) {
    }
    /* Calculate how much latency we'd expect */
    double talonLatency = talonPos.getTimestamp().getLatency();
    double compensatedTalonPos = position + (velocity * talonLatency);
    double cancoderLatency = cancoderPos.getTimestamp().getLatency();
    double compensatedCANcoderPos = position + (velocity * cancoderLatency);

    /* Calculate compensated values before the assert to avoid timing issue related to it */
    double functionCompensatedTalon =
        BaseStatusSignal.getLatencyCompensatedValue(talonPos, talonVel);
    double functionCompensatedCANcoder =
        BaseStatusSignal.getLatencyCompensatedValue(cancoderPos, cancoderVel);

    /* Assert the two methods match */
    System.out.println("Talon Pos: " + compensatedTalonPos + " - " + functionCompensatedTalon);
    System.out.println(
        "CANcoder Pos: " + compensatedCANcoderPos + " - " + functionCompensatedCANcoder);
    assertEquals(compensatedTalonPos, functionCompensatedTalon, DOUBLE_DELTA);
    assertEquals(compensatedCANcoderPos, functionCompensatedCANcoder, DOUBLE_DELTA);
  }
}
