package frc.lib.feedforward;

import edu.wpi.first.math.geometry.Rotation2d;

public class TelescopingArmFeedforward {

  public double kA;
  public double kG;
  public double kS;
  public double kV;
  public double kO;

  public TelescopingArmFeedforward() {}

  public static TelescopingArmFeedforward createGravityCompensation(double kgMin, double kgMax) {
    TelescopingArmFeedforward config = new TelescopingArmFeedforward();
    config.kG = (kgMin + kgMax) / 2.0;
    config.kS = kgMax - kgMin;

    return config;
  }

  public static TelescopingArmFeedforward createPivotGravityCompensation(
      double kgMin, double kgMax, double leverLengthMeters) {
    kgMin /= leverLengthMeters;
    kgMax /= leverLengthMeters;

    return createGravityCompensation(kgMin, kgMax);
  }

  public static TelescopingArmFeedforward createTelescopingGravityCompensation(
      double kgMin, double kgMax, Rotation2d angle) {
    kgMin /= angle.getSin();
    kgMax /= angle.getSin();

    return createGravityCompensation(kgMin, kgMax);
  }

  public double calculatePivot(
      Rotation2d theta, double length, Rotation2d omega, Rotation2d alpha) {
    return kS * Math.signum(omega.getRadians())
        + kG * theta.getCos() * length
        + kV * omega.getRadians()
        + kA * alpha.getRadians()
        + kO;
  }

  public double calculatePivot(Rotation2d theta, double length) {
    return calculatePivot(theta, length, Rotation2d.fromRadians(0), Rotation2d.fromRadians(0));
  }

  public double calculateTelescoping(Rotation2d theta, double velocity, double acceleration) {
    return kS * Math.signum(velocity)
        + kG * theta.getSin()
        + kV * velocity
        + kA * acceleration
        + kO;
  }

  public double calculateTelescoping(Rotation2d theta) {
    return calculateTelescoping(theta, 0, 0);
  }
}
