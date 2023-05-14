package frc.robot.arm;

import frc.robot.Constants;

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
    return Constants.Arm.Rotation.Feedforward.KG * position.getAngle().getCos() * position.getLength();
  }
}
