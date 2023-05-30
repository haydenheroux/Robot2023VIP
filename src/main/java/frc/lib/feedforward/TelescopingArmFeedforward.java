package frc.lib.feedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.ArmPosition;

public class TelescopingArmFeedforward {

  public double kA;
  public double kG;
  public double kS;
  public double kV;
  public double kO;

  public TelescopingArmFeedforward() {}

  private static TelescopingArmFeedforward gravityCompensation(double kgMin, double kgMax) {
    TelescopingArmFeedforward config = new TelescopingArmFeedforward();
    config.kG = (kgMin + kgMax) / 2.0;
    config.kS = kgMax - kgMin;

    return config;
  }

  public static TelescopingArmFeedforward pivotGravityCompensation(
      double kgMin, double kgMax, double length) {
    kgMin /= length;
    kgMax /= length;

    return gravityCompensation(kgMin, kgMax);
  }

  public static TelescopingArmFeedforward telescopingGravityCompensation(
      double kgMin, double kgMax, Rotation2d angle) {
    kgMin /= angle.getSin();
    kgMax /= angle.getSin();

    return gravityCompensation(kgMin, kgMax);
  }

  public double calculatePivot(
      Rotation2d theta, double length, Rotation2d omega, Rotation2d alpha) {
    return kS * Math.signum(omega.getRadians())
        + kG * theta.getCos() * length
        + kV * omega.getRadians()
        + kA * alpha.getRadians()
        + kO;
  }

  public double calculatePivot(ArmPosition position) {
    return calculatePivot(position.getAngle(), position.getLength(), Rotation2d.fromRadians(0), Rotation2d.fromRadians(0));
  }

  public double calculateTelescoping(Rotation2d theta, double velocity, double acceleration) {
    return kS * Math.signum(velocity)
        + kG * theta.getSin()
        + kV * velocity
        + kA * acceleration
        + kO;
  }

  public double calculateTelescoping(ArmPosition position) {
    return calculateTelescoping(position.getAngle(), 0, 0);
  }
}
