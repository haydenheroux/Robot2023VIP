package frc.robot.arm;

public class ExtensionRotationFeedforward {

  public ExtensionRotationFeedforward() {}

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public double calculateExtension(ArmPosition position, double volts) {
    final double kS = 0.0;
    final double kG = 0.0;

    double s = kS * Math.signum(volts);

    double g = kG * position.getAngle().getSin();

    return s + g;
  }

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public double calculateRotationVoltageToOvercomeGravity(ArmPosition position) {
    final double kG = 0.764;

    return kG * position.getAngle().getCos() * position.getLength();
  }
}
