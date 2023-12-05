package frc.lib.telemetry;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Manages automatic initialization and outputting of {@link TelemetryOutputter} objects. */
public class Telemetry {

  public enum Level {
    ODOMETRY,
    DRIVE,
    MECHANISMS
  }

  // Singleton instance
  private static Telemetry instance = null;

  private Map<Level, List<TelemetryOutputter>> outputters =
      new HashMap<Level, List<TelemetryOutputter>>();

  private Telemetry() {}

  public static Telemetry getInstance() {
    if (instance == null) {
      instance = new Telemetry();
    }
    return instance;
  }

  /**
   * Registers multiple {@link TelemetryOutputter} objects to the manager.
   *
   * @param telemetryOutputters objects to register.
   * @return the telemetry manager.
   */
  public Telemetry register(Level level, TelemetryOutputter... telemetryOutputters) {
    if (this.outputters.get(level) == null) {
      this.outputters.put(level, Arrays.asList(telemetryOutputters));

      return this;
    }

    this.outputters.put(
        level,
        Stream.concat(
                this.outputters.get(level).stream(), Arrays.asList(telemetryOutputters).stream())
            .collect(Collectors.toList()));

    return this;
  }

  public Telemetry initializeDashboards(Level level) {
    List<TelemetryOutputter> outputters = this.outputters.get(level);

    if (outputters == null) return this;

    outputters.forEach(TelemetryOutputter::initializeDashboard);

    return this;
  }

  public void outputTelemetry(Level level) {
    List<TelemetryOutputter> outputters = this.outputters.get(level);

    if (outputters == null) return;

    outputters.forEach(TelemetryOutputter::outputTelemetry);
  }
}
