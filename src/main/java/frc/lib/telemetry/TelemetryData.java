package frc.lib.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TelemetryData {

  /**
   * Gets an array of four swerve module states as a double array.
   *
   * @see SwerveModuleState
   * @see <a
   *     href="https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/SWERVE.md#states"></a>
   * @param states
   * @return a double array representing four swerve module states.
   */
  public static double[] asDoubleArray(SwerveModuleState[] states) {
    double[] doubles = new double[8];

    for (int i = 0; i < 4; i++) {
      SwerveModuleState state = states[i];
      doubles[2 * i] = state.angle.getRadians();
      doubles[2 * i + 1] = state.speedMetersPerSecond;
    }

    return doubles;
  }
}
