package frc.lib.controllers.feedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.ArmPosition;

public class TelescopingArmFeedforward {

  public double kG;
  public double kL;
  public double kO;

  public static double pivotKG(double volts, Rotation2d angle) {
    return volts / angle.getCos();
  }

  public static double pivotKG(double volts, ArmPosition position) {
    return pivotKG(volts, position.getAngle());
  }

  public static double telescopingKG(double volts, Rotation2d angle) {
    return volts / angle.getSin();
  }

  public static double telescopingKG(double volts, ArmPosition position) {
    return telescopingKG(volts, position.getAngle());
  }

  public TelescopingArmFeedforward() {}

  public double calculatePivot(Rotation2d theta, double length) {
    return kG * theta.getCos() + kL * theta.getCos() * length + kO;
  }

  public double calculatePivot(ArmPosition position) {
    return calculatePivot(position.getAngle(), position.getExtension());
  }

  public double calculateTelescoping(Rotation2d theta) {
    return kG * theta.getSin() + kO;
  }

  public double calculateTelescoping(ArmPosition position) {
    return calculateTelescoping(position.getAngle());
  }
}
