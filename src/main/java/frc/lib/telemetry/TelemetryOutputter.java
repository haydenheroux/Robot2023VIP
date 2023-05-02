// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.telemetry;

/** A TelemetryOutputter is a subsystem which can be placed on a dashboard and outputs telemetry. */
public interface TelemetryOutputter {
  /** Initializes the subsystem's dashboard tab. */
  public void initializeDashboard();

  /** Outputs the subsystem's telemetry. */
  public void outputTelemetry();
}
