package frc.lib.telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Manages automatic initialization and outputting of {@link TelemetryOutputter} objects. */
public class TelemetryManager {
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
   * @return the telemetry manager.
   */
  public TelemetryManager register(TelemetryOutputter... telemetryOutputters) {
    this.telemetryOutputters = Arrays.asList(telemetryOutputters);

    return this;
  }

  public TelemetryManager initializeDashboards() {
    telemetryOutputters.forEach(TelemetryOutputter::initializeDashboard);

    return this;
  }

  public void outputTelemetry() {
    telemetryOutputters.forEach(TelemetryOutputter::outputTelemetry);
  }
}
