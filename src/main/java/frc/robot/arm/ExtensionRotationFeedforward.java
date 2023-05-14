package frc.robot.arm;

public class ExtensionRotationFeedforward {

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public static double calculateExtensionG(ArmPosition position) {
    final double kG = 2;

    return kG * position.getAngle().getSin();
  }

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public static double calculateRotationG(ArmPosition position) {
    final double kG = 0.764;

    return kG * position.getAngle().getCos() * position.getLength();
  }
}
