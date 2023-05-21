package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ExtensionRotationFeedforward {

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public static double calculateExtensionG(Rotation2d angle) {
    return Constants.Arm.Extension.Feedforward.KG * angle.getSin();
  }

  /**
   * Calculate the voltage required to overcome resistive forces at the given position.
   *
   * @return the voltage required to overcome resistive forces at the given position.
   */
  public static double calculateRotationG(Rotation2d angle, double leverLengthMeters) {
    return Constants.Arm.Rotation.Feedforward.KG * angle.getCos() * leverLengthMeters;
  }
}
