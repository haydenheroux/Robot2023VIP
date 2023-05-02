// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Manages automatic initialization and outputting of {@link TelemetryOutputter} objects. */
public class TelemetryManager implements TelemetryOutputter {
  // Singleton instance
  private static TelemetryManager instance = null;

  private List<TelemetryOutputter> telemetryOutputters = new ArrayList<>();

  private TelemetryManager() {}

  public static TelemetryManager getInstance() {
    if (instance == null) {
      instance = new TelemetryManager();
    }
    return instance;
  }

  /**
   * Registers multiple {@link TelemetryOutputter} objects to the manager.
   *
   * @param telemetryOutputters objects to register.
   */
  public void register(TelemetryOutputter... telemetryOutputters) {
    this.telemetryOutputters = Arrays.asList(telemetryOutputters);
  }

  @Override
  public void initializeDashboard() {
    telemetryOutputters.forEach(TelemetryOutputter::initializeDashboard);
  }

  @Override
  public void outputTelemetry() {
    telemetryOutputters.forEach(TelemetryOutputter::outputTelemetry);
  }
}
