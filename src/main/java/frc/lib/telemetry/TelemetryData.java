package frc.lib.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TelemetryData {
   
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
